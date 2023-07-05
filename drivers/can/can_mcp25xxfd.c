/*
 * Copyright (c) 2020 Abram Early
 * Copyright (c) 2023 Andriy Gelman
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp25xxfd

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_CAN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mcp25xxfd_can);

#include "can_mcp25xxfd.h"

#define SP_IS_SET(inst) DT_INST_NODE_HAS_PROP(inst, sample_point) ||

/* Macro to exclude the sample point algorithm from compilation if not used
 * Without the macro, the algorithm would always waste ROM
 */
#define USE_SP_ALGO (DT_INST_FOREACH_STATUS_OKAY(SP_IS_SET) 0)

/* Timeout for changing mode */
#define MODE_CHANGE_TIMEOUT_USEC (CONFIG_CAN_MCP25XXFD_MODE_CHANGE_TIMEOUT_MSEC * 1000U)
#define MODE_CHANGE_RETRIES	   100

#define PLLRDY_TIMEOUT_USEC 100000
#define PLLRDY_RETRIES	   100

/* when a single mailbox is used access to is protected by tx_sem */
#if MCP25XXFD_TX_FIFO_ITEMS > 1
#define MCP25XXFD_MAILBOX_LOCK(mutex)   k_mutex_lock(mutex, K_FOREVER)
#define MCP25XXFD_MAILBOX_UNLOCK(mutex) k_mutex_unlock(mutex)
#define MCP25XXFD_TXFIFO_LOCK(mutex)    k_mutex_lock(mutex, K_FOREVER)
#define MCP25XXFD_TXFIFO_UNLOCK(mutex)  k_mutex_unlock(mutex)
#else
#define MCP25XXFD_MAILBOX_LOCK(mutex)
#define MCP25XXFD_MAILBOX_UNLOCK(mutex)
#define MCP25XXFD_TXFIFO_LOCK(mutex)
#define MCP25XXFD_TXFIFO_UNLOCK(mutex)
#endif

/* rxfifo/tefifo access is run in a cooperative thread. */
/* It won't be interrupted unless SMP is enabled */
#if CONFIG_SMP
#define MCP25XXFD_FIFO_LOCK(mutex)   k_mutex_lock(mutex, K_FOREVER)
#define MCP25XXFD_FIFO_UNLOCK(mutex) k_mutex_unlock(mutex)
#else
#define MCP25XXFD_FIFO_LOCK(mutex)
#define MCP25XXFD_FIFO_UNLOCK(mutex)
#endif

static int mcp25xxfd_reset(const struct device *dev)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	uint16_t cmd = sys_cpu_to_be16(MCP25XXFD_SPI_INSTRUCTION_RESET);

	const struct spi_buf tx_buf = {.buf = &cmd, .len = sizeof(cmd),};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	return spi_write_dt(&dev_cfg->bus, &tx);
}

static void mcp25xxfd_canframe_to_txobj(const struct can_frame *src, struct mcp25xxfd_txobj *dst)
{
	memset(dst, 0, offsetof(struct mcp25xxfd_txobj, DATA));

	if (src->flags & CAN_FRAME_IDE) {
		dst->SID = src->id >> 18;
		dst->EID = src->id;
		dst->IDE = 1;
	} else {
		dst->SID = src->id;
	}

	if (src->flags & CAN_FRAME_BRS) {
		dst->BRS = 1;
	}

	if (src->flags & CAN_FRAME_RTR) {
		dst->RTR = 1;
	}
	dst->DLC = src->dlc;
#if defined(CONFIG_CAN_FD_MODE)
	if (src->flags & CAN_FRAME_FDF) {
		dst->FDF = 1;
	}
#endif
	memcpy(dst->DATA, src->data, MIN(can_dlc_to_bytes(src->dlc), CAN_MAX_DLEN));
}

void *mcp25xxfd_read_reg(const struct device *dev, uint16_t addr, int len)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_spi_data *spi_data = &dev_data->spi_data;
	int ret;

	spi_data->spi_cmd = sys_cpu_to_be16(MCP25XXFD_SPI_INSTRUCTION_READ | addr);

	struct spi_buf tx_buf = {.buf = spi_data, .len = 2};
	struct spi_buf rx_buf = {.buf = spi_data, .len = 2 + len};

	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

	ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
	if (ret) {
		return NULL;
	}

	return &spi_data->buf[0];
}

static inline void *mcp25xxfd_get_spi_buf_ptr(const struct device *dev, int zero_init_size)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_spi_data *spi_data = &dev_data->spi_data;
	uint8_t *data;

	data = &spi_data->buf[0];
	memset(data, 0, zero_init_size);

	return data;
}

static int mcp25xxfd_write_reg(const struct device *dev, uint16_t addr, int len)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_spi_data *spi_data = &dev_data->spi_data;

	struct spi_buf tx_buf = {.buf = spi_data, .len = 2 + len};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	spi_data->spi_cmd = sys_cpu_to_be16(MCP25XXFD_SPI_INSTRUCTION_WRITE | addr);

	return spi_write_dt(&dev_cfg->bus, &tx);
}

static int mcp25xxfd_fifo_write(const struct device *dev, int mailbox_idx, uint16_t fifo_address,
				const struct can_frame *msg)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_fiforegs *fiforegs;
	struct mcp25xxfd_txobj *txobj;
	struct mcp25xxfd_fifocon_b1 *fifocon_b1;
	uint16_t address;
	int tx_len;
	int ret;

	MCP25XXFD_TXFIFO_LOCK(&dev_data->mutex);
#if MCP25XXFD_TX_FIFO_ITEMS == 1
	ARG_UNUSED(dev_data);
#endif

	fiforegs = mcp25xxfd_read_reg(dev, fifo_address, sizeof(*fiforegs));
	if (!fiforegs) {
		LOG_ERR("Failed to read 12 bytes from FIFOCON");
		ret = -EINVAL;
		goto done;
	}

	if (!fiforegs->fifosta.b0.FNEIF) {
		ret = -ENOMEM;
		goto done;
	}

	/* todo check address */
	address = 0x400 + fiforegs->ua;

	txobj = mcp25xxfd_get_spi_buf_ptr(dev, 0);
	mcp25xxfd_canframe_to_txobj(msg, txobj);

	txobj->SEQ = mailbox_idx;
	tx_len = 8 + ROUND_UP(can_dlc_to_bytes(msg->dlc), 4);

	ret = mcp25xxfd_write_reg(dev, address, tx_len);
	if (ret) {
		goto done;
	}

	fifocon_b1 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon_b1));
	fifocon_b1->UINC = 1;
	fifocon_b1->TXREQ = 1;

	ret = mcp25xxfd_write_reg(dev, fifo_address + 1, sizeof(*fifocon_b1));
	if (ret) {
		goto done;
	}
done:
	MCP25XXFD_TXFIFO_UNLOCK(&dev_data->mutex);
	return ret;
}

static void mcp25xxfd_rxobj_to_canframe(const struct mcp25xxfd_rxobj *src, struct can_frame *dst)
{
	memset(dst, 0, offsetof(struct can_frame, data));

	if (src->IDE) {
		dst->id = src->EID | (src->SID << 18);
		dst->flags |= CAN_FRAME_IDE;
	} else {
		dst->id = src->SID;
	}

	if (src->BRS) {
		dst->flags |= CAN_FRAME_BRS;
	}

	if (src->RTR) {
		dst->flags |= CAN_FRAME_RTR;
	}

#if defined(CONFIG_CAN_FD_MODE)
	if (src->FDF) {
		dst->flags |= CAN_FRAME_FDF;
	}
#endif

	dst->dlc = src->DLC;

#if defined(CONFIG_CAN_RX_TIMESTAMP)
	dst->timestamp = src->RXMSGTS;
#endif

	memcpy(dst->data, src->DATA, MIN(can_dlc_to_bytes(src->DLC), CAN_MAX_DLEN));
}

static int mcp25xxfd_get_mode_internal(const struct device *dev, uint8_t *mode)
{
	struct mcp25xxfd_con_b2 *con_b2;

	con_b2 = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_CON + 2, sizeof(*con_b2));
	if (!con_b2) {
		return -EINVAL;
	}

	*mode = con_b2->OPMOD;

	return 0;
}

/* helper function to set a value in one u8 register and */
/* validate that the value in another register */
static int mcp25xxfd_set_bitmask_and_validate_reg_u8(const struct device *dev, uint16_t addr_write,
						     uint8_t value_write, uint8_t mask_write,
						     uint16_t addr_validate, uint8_t value_validate,
						     uint8_t mask_validate, uint32_t timeout_usec,
						     int retries, bool allow_yield)
{
	uint8_t *data;
	int ret;
	uint32_t delay = timeout_usec / retries;

	data = mcp25xxfd_read_reg(dev, addr_write, 1);
	if (!data) {
		return -EINVAL;
	}

	*data &= ~mask_write;
	*data |= value_write;

	ret = mcp25xxfd_write_reg(dev, addr_write, 1);
	if (ret) {
		return ret;
	}

	for (;;) {
		data = mcp25xxfd_read_reg(dev, addr_validate, 1);
		if (!data) {
			return -EINVAL;
		}

		if ((*data & mask_validate) == value_validate) {
			return 0;
		}

		if (--retries < 0) {
			LOG_ERR("Timeout validing 0x%x", addr_validate);
			return -EIO;
		}

		if (allow_yield) {
			k_sleep(K_USEC(delay));
		} else {
			k_busy_wait(delay);
		}
	}
	return 0;
}

static int mcp25xxfd_set_mode_internal(const struct device *dev, uint8_t mode)
{
	struct mcp25xxfd_con_b2 *con_b2;
	struct mcp25xxfd_con_b3 *con_b3;
	uint8_t reg_value;
	uint8_t reg_value_validate;

	con_b2 = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_CON + 2, sizeof(*con_b2));
	if (!con_b2) {
		return -EINVAL;
	}

	if (con_b2->OPMOD == mode) {
		return 0;
	}

	/* a bit of hack to get bit proper bit fields set */
	con_b2 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*con_b2));
	con_b2->OPMOD = mode;
	memcpy(&reg_value_validate, con_b2, sizeof(*con_b2));

	con_b3 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*con_b3));
	con_b3->REQOP = mode;
	memcpy(&reg_value, con_b3, sizeof(*con_b3));

	/* we need to request the mode in one register and validate it another */
	return mcp25xxfd_set_bitmask_and_validate_reg_u8(dev,
		MCP25XXFD_REG_CON + 3, reg_value, MCP25XXFD_CON_B3_REQOP_MASK,
		MCP25XXFD_REG_CON + 2, reg_value_validate, MCP25XXFD_CON_B2_OPMOD_MASK,
		MODE_CHANGE_TIMEOUT_USEC, MODE_CHANGE_RETRIES, true);
}

static int mcp25xxfd_set_mode(const struct device *dev, can_mode_t mode)
{
	struct mcp25xxfd_data *dev_data = dev->data;

	if (dev_data->started) {
		return -EBUSY;
	}

	switch (mode) {
	case CAN_MODE_NORMAL:
		dev_data->mcp25xxfd_mode = MCP25XXFD_OPMODE_NORMAL_CAN2;
		break;
	case CAN_MODE_FD:
#if defined(CONFIG_CAN_FD_MODE)
		dev_data->mcp25xxfd_mode = MCP25XXFD_OPMODE_NORMAL_CANFD;
		break;
#else
		return -ENOTSUP;
#endif
	case CAN_MODE_LISTENONLY:
		dev_data->mcp25xxfd_mode = MCP25XXFD_OPMODE_LISTEN_ONLY;
		break;
	case CAN_MODE_LOOPBACK:
		dev_data->mcp25xxfd_mode = MCP25XXFD_OPMODE_EXT_LOOPBACK;
		break;
	default:
		LOG_ERR("Unsupported CAN Mode %u", mode);
		return -ENOTSUP;
	}

	return 0;
}

static int mcp25xxfd_set_timing(const struct device *dev, const struct can_timing *timing,
				bool is_nominal)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_nbtcfg *nominal_timing_reg;
	struct mcp25xxfd_dbtcfg *data_timing_reg;
	struct mcp25xxfd_tdc *tdc;
	uint8_t *sjw_cached;
	uint16_t addr_timing_reg, reg_size;

	int ret;

	if (!timing) {
		return -EINVAL;
	}

	/* we have to be in a configuration mode at this point */
	if (dev_data->started) {
		return -EBUSY;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	sjw_cached = is_nominal ? &dev_data->sjw : &dev_data->sjw_data;
	if (timing->sjw != CAN_SJW_NO_CHANGE) {
		*sjw_cached = timing->sjw;
	}

	addr_timing_reg = is_nominal ?  MCP25XXFD_REG_NBTCFG : MCP25XXFD_REG_DBTCFG;

	if (is_nominal) {
		nominal_timing_reg = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*nominal_timing_reg));
		nominal_timing_reg->BRP = timing->prescaler - 1;
		nominal_timing_reg->TSEG1 = timing->prop_seg + timing->phase_seg1 - 1;
		nominal_timing_reg->TSEG2 = timing->phase_seg2 - 1;
		nominal_timing_reg->SJW = *sjw_cached - 1;
		reg_size = sizeof(*nominal_timing_reg);
	} else {
		data_timing_reg = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*data_timing_reg));
		data_timing_reg->BRP = timing->prescaler - 1;
		data_timing_reg->TSEG1 = timing->prop_seg + timing->phase_seg1 - 1;
		data_timing_reg->TSEG2 = timing->phase_seg2 - 1;
		data_timing_reg->SJW = *sjw_cached - 1;
		reg_size = sizeof(*data_timing_reg);
	}

	ret = mcp25xxfd_write_reg(dev, addr_timing_reg, reg_size);
	if (ret < 0) {
		LOG_ERR("Failed to write device configuration [%d]", ret);
		goto done;
	}

	tdc = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*tdc));
	if (is_nominal) {
		tdc->TDCMOD = MCP25XXFD_TDCMOD_DISABLED;
	} else {
		tdc->TDCMOD = MCP25XXFD_TDCMOD_AUTO;
		tdc->TDCO = timing->prescaler * (timing->prop_seg + timing->phase_seg1);
	}

	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_TDC, sizeof(*tdc));
	if (ret < 0) {
		LOG_ERR("Failed to write device configuration [%d]", ret);
		goto done;
	}

#if defined(CONFIG_CAN_RX_TIMESTAMP)
	struct mcp25xxfd_tscon *tscon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*tscon));

	tscon->TBCEN = 1;
	tscon->TBCPRE = timing->prescaler - 1;

	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_TSCON, sizeof(*tscon));
	if (ret < 0) {
		LOG_ERR("Failed to write device configuration [%d]", ret);
		goto done;
	}
#endif

done:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int mcp25xxfd_set_timing_nominal(const struct device *dev, const struct can_timing *timing)
{
	return mcp25xxfd_set_timing(dev, timing, true);
}


#if defined(CONFIG_CAN_FD_MODE)
static int mcp25xxfd_set_timing_data(const struct device *dev, const struct can_timing *timing)
{
	return mcp25xxfd_set_timing(dev, timing, false);
}
#endif

static int mcp25xxfd_send(const struct device *dev, const struct can_frame *msg,
			  k_timeout_t timeout, can_tx_callback_t callback, void *callback_arg)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	uint8_t mailbox_idx = 0;
	int ret;

	LOG_DBG("Sending %d bytes. Id: 0x%x, ID type: %s %s %s %s", can_dlc_to_bytes(msg->dlc),
		msg->id, msg->flags & CAN_FRAME_IDE ? "extended" : "standard",
		msg->flags & CAN_FRAME_RTR ? "RTR" : "",
		msg->flags & CAN_FRAME_FDF ? "FD frame" : "",
		msg->flags & CAN_FRAME_BRS ? "BRS" : "");

	__ASSERT_NO_MSG(callback != NULL);

	if ((msg->flags & CAN_FRAME_FDF) == 0 && msg->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d without fd flag set.", msg->dlc);
		return -EINVAL;
	}

	if (k_sem_take(&dev_data->tx_sem, timeout) != 0) {
		return -EAGAIN;
	}

	MCP25XXFD_MAILBOX_LOCK(&dev_data->mutex);
	for (; mailbox_idx < MCP25XXFD_TX_FIFO_ITEMS; mailbox_idx++) {
		if ((BIT(mailbox_idx) & dev_data->mailbox_usage) == 0) {
			dev_data->mailbox_usage |= BIT(mailbox_idx);
			break;
		}
	}
	MCP25XXFD_MAILBOX_UNLOCK(&dev_data->mutex);

	if (mailbox_idx >= MCP25XXFD_TX_FIFO_ITEMS) {
		k_sem_give(&dev_data->tx_sem);
		return -EIO;
	}

	dev_data->mailbox[mailbox_idx].cb = callback;
	dev_data->mailbox[mailbox_idx].cb_arg = callback_arg;

	ret = mcp25xxfd_fifo_write(dev, mailbox_idx, MCP25XXFD_REG_FIFOCON(mailbox_idx), msg);

	if (ret < 0) {
		MCP25XXFD_MAILBOX_LOCK(&dev_data->mutex);
		dev_data->mailbox_usage &= ~BIT(mailbox_idx);
		dev_data->mailbox[mailbox_idx].cb = NULL;
		MCP25XXFD_MAILBOX_UNLOCK(&dev_data->mutex);
		k_sem_give(&dev_data->tx_sem);
		return ret;
	}

	return 0;
}

static int mcp25xxfd_add_rx_filter(const struct device *dev, can_rx_callback_t rx_cb, void *cb_arg,
				   const struct can_filter *filter)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	int filter_idx = 0;
	int ret;

	__ASSERT(rx_cb != NULL, "rx_cb can not be null");
	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	while ((BIT(filter_idx) & dev_data->filter_usage) && (filter_idx < CONFIG_CAN_MAX_FILTER)) {
		filter_idx++;
	}

	if (filter_idx < CONFIG_CAN_MAX_FILTER) {
		struct mcp25xxfd_fltcon *fltcon;
		struct mcp25xxfd_mask *mask;
		struct mcp25xxfd_fltobj *fltobj = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fltobj));

		if (filter->flags & CAN_FILTER_IDE) {
			fltobj->SID = filter->id >> 18;
			fltobj->EID = filter->id;
			fltobj->EXIDE = 1;
		} else {
			fltobj->SID = filter->id;
		}

		ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_FLTOBJ(filter_idx), sizeof(*fltobj));
		if (ret < 0) {
			LOG_ERR("Failed to write register [%d]", ret);
			goto done;
		}

		mask = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*mask));
		if (filter->flags & CAN_FILTER_IDE) {
			mask->MSID = filter->mask >> 18;
			mask->MEID = filter->mask;
		} else {
			mask->MSID = filter->mask;
		}
		mask->MIDE = 1;

		ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_MASK(filter_idx), sizeof(*mask));
		if (ret < 0) {
			LOG_ERR("Failed to write register [%d]", ret);
			goto done;
		}

		fltcon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fltcon));
		fltcon->FLTEN = 1;
		fltcon->FLTBP = MCP25XXFD_RX_FIFO_IDX;

		ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_FLTCON(filter_idx), sizeof(*fltcon));
		if (ret < 0) {
			LOG_ERR("Failed to write register [%d]", ret);
			goto done;
		}

		dev_data->filter_usage |= BIT(filter_idx);
		dev_data->filter[filter_idx] = *filter;
		dev_data->rx_cb[filter_idx] = rx_cb;
		dev_data->cb_arg[filter_idx] = cb_arg;
	} else {
		filter_idx = -ENOSPC;
	}
done:
	k_mutex_unlock(&dev_data->mutex);

	return filter_idx;
}

static void mcp25xxfd_remove_rx_filter(const struct device *dev, int filter_idx)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_fltcon *fltcon;
	int ret;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->filter_usage &= ~BIT(filter_idx);

	fltcon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fltcon));

	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_FLTCON(filter_idx), sizeof(*fltcon));
	if (ret < 0) {
		LOG_ERR("Failed to remove filter");
	}

	k_mutex_unlock(&dev_data->mutex);
}

static void mcp25xxfd_set_state_change_callback(const struct device *dev,
						can_state_change_callback_t cb, void *user_data)
{
	struct mcp25xxfd_data *dev_data = dev->data;

	dev_data->state_change_cb = cb;
	dev_data->state_change_cb_data = user_data;
}

static int mcp25xxfd_get_state(const struct device *dev, enum can_state *state,
			       struct can_bus_err_cnt *err_cnt)
{
	struct mcp25xxfd_trec *trec;

	trec = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_TREC, sizeof(*trec));
	if (!trec) {
		return -EINVAL;
	}

	if (trec->TXBO) {
		*state = CAN_STATE_BUS_OFF;
	} else if (trec->TXBP || trec->RXBP) {
		*state = CAN_STATE_ERROR_PASSIVE;
	} else if (trec->TXWARN || trec->RXWARN) {
		*state = CAN_STATE_ERROR_WARNING;
	} else {
		*state = CAN_STATE_ERROR_ACTIVE;
	}
	err_cnt->tx_err_cnt = trec->REC;
	err_cnt->rx_err_cnt = trec->TEC;

	return 0;
}

static int mcp25xxfd_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;

	*rate = dev_cfg->osc_freq;
	return 0;
}

static int mcp25xxfd_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	return CONFIG_CAN_MAX_FILTER;
}

static int mcp25xxfd_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;

	*max_bitrate = dev_cfg->max_bitrate;

	return 0;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
static void mcp25xxfd_recover(const struct device *dev, k_timeout_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);
}
#endif

static int mcp25xxfd_handle_fifo_batch_read(const struct device *dev,
					    const struct mcp25xxfd_fifo *fifo)
{
	int ret = 0;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_fifosta_ua *fifosta_ua;
	struct mcp25xxfd_fifocon_b1 *fifocon_b1;

	int len;
	int fetch_total = 0;
	int ui_inc = 0;
	uint32_t fifo_tail_index, fifo_tail_addr;
	uint8_t fifo_head_index;

	MCP25XXFD_FIFO_LOCK(&dev_data->mutex);
#ifndef CONFIG_SMP
	ARG_UNUSED(dev_data);
#endif

	/* read in FIFOSTA and FIFOUA at the same time */
	fifosta_ua = mcp25xxfd_read_reg(dev, fifo->reg_fifocon_addr + 4, sizeof(*fifosta_ua));
	if (!fifosta_ua) {
		ret = -EINVAL;
		goto done;
	}

	/* is there any data in the fifo? */
	if (!fifosta_ua->fifosta.b0.FNEIF) {
		goto done;
	}

	/* fifo_head_index points where the next message will be written. */
	/* It points to one past the end of the fifo. */
	fifo_head_index = fifosta_ua->fifosta.b1.FIFOCI;
	if (fifo_head_index == 0) {
		fifo_head_index = fifo->capacity - 1;
	} else {
		fifo_head_index -= 1;
	}

	fifo_tail_addr = fifosta_ua->ua;
	fifo_tail_index = (fifo_tail_addr - fifo->ram_start_addr) / fifo->item_size;

	if (fifo_tail_index > fifo_head_index) {
		/* fetch to the end of the memory and then wrap to the start */
		fetch_total = fifo->capacity - 1 - fifo_tail_index + 1;
		fetch_total += fifo_head_index + 1;
	} else {
		fetch_total = fifo_head_index - fifo_tail_index + 1;
	}

	while (fetch_total > 0) {
		uint16_t memory_addr;
		uint8_t *data;

		if (fifo_tail_index > fifo_head_index) {
			len = fifo->capacity - 1 - fifo_tail_index + 1;
		} else {
			len = fifo_head_index - fifo_tail_index + 1;
		}

		memory_addr = 0x400 + fifo->ram_start_addr + fifo_tail_index * fifo->item_size;

		data = mcp25xxfd_read_reg(dev, memory_addr, len * fifo->item_size);
		if (!data) {
			LOG_ERR("Error fetching batch message");
			ret = -EINVAL;
			goto done;
		}

		for (int i = 0; i < len; i++) {
			fifo->msg_handler(dev, (void *)(&data[i * fifo->item_size]));
		}

		fifo_tail_index = (fifo_tail_index + len) % fifo->capacity;
		fetch_total -= len;
		ui_inc += len;
	}

	fifocon_b1 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon_b1));
	fifocon_b1->UINC = 1;

	for (int i = 0; i < ui_inc; i++) {
		ret = mcp25xxfd_write_reg(dev, fifo->reg_fifocon_addr + 1, sizeof(*fifocon_b1));
		if (ret < 0) {
			LOG_ERR("Failed to increment pointer");
			goto done;
		}
	}

done:

	MCP25XXFD_FIFO_UNLOCK(&dev_data->mutex);
	return ret;
}

static void mcp25xxfd_reset_tx_fifos(const struct device *dev, int status)
{
	struct mcp25xxfd_data *dev_data = dev->data;

	LOG_INF("All FIFOs Reset");
	for (int i = 0; i < MCP25XXFD_TX_FIFO_ITEMS; i++) {
		can_tx_callback_t callback;

		if (!(dev_data->mailbox_usage & BIT(i))) {
			continue;
		}

		callback = dev_data->mailbox[i].cb;
		if (callback) {
			callback(dev, status, dev_data->mailbox[i].cb_arg);
		}

		MCP25XXFD_MAILBOX_LOCK(&dev_data->mutex);
		dev_data->mailbox_usage &= ~BIT(i);
		dev_data->mailbox[i].cb = NULL;
		MCP25XXFD_MAILBOX_UNLOCK(&dev_data->mutex);
		k_sem_give(&dev_data->tx_sem);
	}
}

/* CERRIF will be set each time a threshold in the TEC/REC counter is crossed by the following */
/* conditions: */
/* • TEC or REC exceeds the Error Warning state threshold */
/* • The transmitter or receiver transitions to Error Passive state */
/* • The transmitter transitions to Bus Off state */
/* • The transmitter or receiver transitions from Error Passive to Error Active state */
/* • The module transitions from Bus Off to Error Active state, after the bus off recovery */
/* sequence */
/* When the user clears CERRIF, it will remain clear until a new counter crossing occurs. */
static int mcp25xxfd_handle_cerrif(const struct device *dev)
{
	struct mcp25xxfd_trec *trec;
	enum can_state new_state;
	struct mcp25xxfd_data *dev_data = dev->data;

	trec = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_TREC, sizeof(*trec));
	if (!trec) {
		return -EINVAL;
	}

	if (trec->TXBO) {
		new_state = CAN_STATE_BUS_OFF;
	} else if (trec->TXBP || trec->RXBP) {
		new_state = CAN_STATE_ERROR_PASSIVE;
	} else if (trec->TXWARN || trec->RXWARN) {
		new_state = CAN_STATE_ERROR_WARNING;
	} else {
		new_state = CAN_STATE_ERROR_ACTIVE;
	}

	if (new_state == dev_data->state) {
		return 0;
	}

	LOG_INF("State %d -> %d (tx: %d, rx: %d)", dev_data->state, new_state,
		trec->TEC, trec->REC);

	/* Upon entering bus-off, all the fifos are reset. */
	dev_data->state = new_state;
	if (new_state == CAN_STATE_BUS_OFF) {
		mcp25xxfd_reset_tx_fifos(dev, -ENETDOWN);
	}

	if (dev_data->state_change_cb) {
		struct can_bus_err_cnt err_cnt = {.rx_err_cnt = trec->REC, .tx_err_cnt = trec->TEC};

		dev_data->state_change_cb(dev, new_state, err_cnt, dev_data->state_change_cb_data);
	}

	return 0;
}

static int mcp25xxfd_handle_modif(const struct device *dev)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	uint8_t mode;
	int ret;

	ret = mcp25xxfd_get_mode_internal(dev, &mode);
	if (ret < 0) {
		return ret;
	}

	if (mode == dev_data->mcp25xxfd_mode) {
		LOG_INF("Switched to mode %d", mode);
		return 0;
	}

	/* try to transition back into our target mode */
	return mcp25xxfd_set_mode_internal(dev, dev_data->mcp25xxfd_mode);
}

static int mcp25xxfd_handle_ivmif(const struct device *dev)
{
	struct mcp25xxfd_bdiag1 *diag;

	diag = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_BDIAG1, sizeof(*diag));
	if (!diag) {
		return -EINVAL;
	}

	if (diag->TXBOERR) {
		LOG_INF("ivmif error");
		mcp25xxfd_reset_tx_fifos(dev, -ENETDOWN);
	}

	memset(diag, 0, sizeof(*diag));

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_BDIAG1, sizeof(*diag));
}

static void mcp25xxfd_handle_interrupts(const struct device *dev)
{
	struct mcp25xxfd_int_hw0 int_flags;
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;
	int ret;

	while (1) {
		struct mcp25xxfd_int_hw0 *int_flags_tmp;

		int_flags_tmp = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_INT, sizeof(*int_flags_tmp));
		if (!int_flags_tmp) {
			continue;
		}

		memcpy(&int_flags, int_flags_tmp, sizeof(*int_flags_tmp));

		/* these interrupt flags need to be explicitly cleared */
		if (int_flags_tmp->IVMIF || int_flags_tmp->WAKIF || int_flags_tmp->CERRIF ||
		    int_flags_tmp->SERRIF || int_flags_tmp->MODIF) {
			int_flags_tmp->IVMIF = 0;
			int_flags_tmp->WAKIF = 0;
			int_flags_tmp->CERRIF = 0;
			int_flags_tmp->SERRIF = 0;
			int_flags_tmp->MODIF = 0;

			ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_INT, sizeof(*int_flags_tmp));
			if (ret) {
				LOG_ERR("Error clearing interrupts");
			}
		}

		if (int_flags.RXIF) {
			ret = mcp25xxfd_handle_fifo_batch_read(dev, &dev_cfg->rx_fifo);
			if (ret < 0) {
				LOG_ERR("Error during RXIF batch read %d", ret);
			}
		}

		if (int_flags.TEFIF) {
			ret = mcp25xxfd_handle_fifo_batch_read(dev, &dev_cfg->tef_fifo);
			if (ret < 0) {
				LOG_ERR("Error during TEFIF batch read %d", ret);
			}
		}

		if (int_flags.IVMIF) {
			ret = mcp25xxfd_handle_ivmif(dev);
			if (ret < 0) {
				LOG_ERR("Error handling ivmif %d", ret);
			}
		}

		if (int_flags.MODIF) {
			ret = mcp25xxfd_handle_modif(dev);
			if (ret < 0) {
				LOG_ERR("Error handling modif %d", ret);
			}
		}

		/* From Linux mcp251xfd driver */
		/* On the MCP2527FD and MCP2518FD, we don't get a CERRIF IRQ on the transition */
		/* TX ERROR_WARNING -> TX ERROR_ACTIVE. */
		if (int_flags.CERRIF || dev_data->state > CAN_STATE_ERROR_ACTIVE) {
			ret = mcp25xxfd_handle_cerrif(dev);
			if (ret < 0) {
				LOG_ERR("Error handling cerrif");
			}
		}

		/* Break from loop if INT pin is inactive */
		ret = gpio_pin_get_dt(&dev_cfg->int_gpio_dt);
		if (ret < 0) {
			LOG_ERR("Couldn't read INT pin");
		} else if (ret == 0) {
			/* All interrupt flags handled */
			break;
		}
	}
}

static void mcp25xxfd_int_thread(const struct device *dev)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;

	while (1) {
		k_sem_take(&dev_data->int_sem, K_FOREVER);
		mcp25xxfd_handle_interrupts(dev);

		/* Re-enable pin interrupts */
		if (gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INT_LEVEL_ACTIVE)) {
			LOG_ERR("Couldn't enable pin interrupt");
			k_oops();
		}
	}
}

static void mcp25xxfd_int_gpio_callback(const struct device *dev_gpio, struct gpio_callback *cb,
					uint32_t pins)
{
	ARG_UNUSED(dev_gpio);
	struct mcp25xxfd_data *dev_data = CONTAINER_OF(cb, struct mcp25xxfd_data, int_gpio_cb);
	const struct device *dev = dev_data->dev;
	const struct mcp25xxfd_config *dev_cfg = dev->config;

	/* Disable pin interrupts */
	if (gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INT_DISABLE)) {
		LOG_ERR("Couldn't disable pin interrupt");
		k_oops();
	}

	k_sem_give(&dev_data->int_sem);
}

static int mcp25xxfd_start(const struct device *dev)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	int ret;

	if (dev_data->started) {
		return -EALREADY;
	}

	if (dev_cfg->phy != NULL) {
		ret = can_transceiver_enable(dev_cfg->phy);
		if (ret != 0) {
			LOG_ERR("Failed to enable CAN transceiver [%d]", ret);
			return ret;
		}
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = mcp25xxfd_set_mode_internal(dev, dev_data->mcp25xxfd_mode);
	if (ret < 0) {
		LOG_ERR("Failed to set the mode [%d]", ret);
		if (dev_cfg->phy != NULL) {
			/* Attempt to disable the CAN transceiver in case of error */
			(void)can_transceiver_disable(dev_cfg->phy);
		}
	} else {
		dev_data->started = true;
	}

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int mcp25xxfd_stop(const struct device *dev)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_con_b3 *con_b3;
	int ret;

	if (!dev_data->started) {
		return -EALREADY;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	/* abort all transmissions */
	con_b3 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*con_b3));
	con_b3->ABAT = 1;

	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_CON + 3, sizeof(*con_b3));
	if (ret < 0) {
		k_mutex_unlock(&dev_data->mutex);
		return ret;
	}

	/* wait for all the messages to be aborted */
	while (1) {
		/* todo check alignment */
		con_b3 = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_CON + 3, sizeof(*con_b3));

		if (!con_b3 || con_b3->ABAT == 0) {
			break;
		}
	}

	mcp25xxfd_reset_tx_fifos(dev, -ENETDOWN);

	ret = mcp25xxfd_set_mode_internal(dev, MCP25XXFD_OPMODE_CONFIGURATION);
	if (ret < 0) {
		k_mutex_unlock(&dev_data->mutex);
		return ret;
	}

	dev_data->started = false;
	k_mutex_unlock(&dev_data->mutex);

	if (dev_cfg->phy != NULL) {
		ret = can_transceiver_disable(dev_cfg->phy);
		if (ret != 0) {
			LOG_ERR("Failed to disable CAN transceiver [%d]", ret);
			return ret;
		}
	}

	return 0;
}

static const struct can_driver_api can_api_funcs = {
	.set_mode = mcp25xxfd_set_mode,
	.set_timing = mcp25xxfd_set_timing_nominal,
#if defined(CONFIG_CAN_FD_MODE)
	.set_timing_data = mcp25xxfd_set_timing_data,
#endif
	.start = mcp25xxfd_start,
	.stop = mcp25xxfd_stop,
	.send = mcp25xxfd_send,
	.add_rx_filter = mcp25xxfd_add_rx_filter,
	.remove_rx_filter = mcp25xxfd_remove_rx_filter,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = mcp25xxfd_recover,
#endif
	.get_state = mcp25xxfd_get_state,
	.set_state_change_callback = mcp25xxfd_set_state_change_callback,
	.get_core_clock = mcp25xxfd_get_core_clock,
	.get_max_filters = mcp25xxfd_get_max_filters,
	.get_max_bitrate = mcp25xxfd_get_max_bitrate,
	.timing_min = MCP25XXFD_TIMING_MIN_INITIALIZER,
	.timing_max = MCP25XXFD_TIMING_MAX_INITIALIZER,
#if defined(CONFIG_CAN_FD_MODE)
	.timing_data_min = MCP25XXFD_TIMING_DATA_MIN_INITIALIZER,
	.timing_data_max = MCP25XXFD_TIMING_DATA_MAX_INITIALIZER,
#endif
};

static void mcp25xxfd_rx_fifo_handler(const struct device *dev, void *data)
{
	struct can_frame msg;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_rxobj *rxobj = data;
	int filhit = rxobj->FILHIT;

	mcp25xxfd_rxobj_to_canframe(rxobj, &msg);
	if (dev_data->filter_usage & BIT(filhit)) {
		LOG_DBG("Received msg CAN id: 0%x", msg.id);
		dev_data->rx_cb[filhit](dev, &msg, dev_data->cb_arg[filhit]);
	}
}

static void mcp25xxfd_tef_fifo_handler(const struct device *dev, void *data)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	can_tx_callback_t callback;
	struct mcp25xxfd_tefobj *tefobj = data;
	uint8_t mailbox_idx;

	mailbox_idx = tefobj->SEQ;
	if (mailbox_idx >= MCP25XXFD_TX_FIFO_ITEMS) {
		LOG_ERR("Invalid mailbox index");
		return;
	}

	callback = dev_data->mailbox[mailbox_idx].cb;
	if (callback) {
		callback(dev, 0, dev_data->mailbox[mailbox_idx].cb_arg);
	}

	MCP25XXFD_MAILBOX_LOCK(&dev_data->mutex);
	dev_data->mailbox_usage &= ~BIT(mailbox_idx);
	dev_data->mailbox[mailbox_idx].cb = NULL;
	MCP25XXFD_MAILBOX_UNLOCK(&dev_data->mutex);
	k_sem_give(&dev_data->tx_sem);
}

static int mcp25xxfd_init_timing_struct(struct can_timing *timing,
					const struct device *dev,
					const struct mcp25xxfd_timing_params *timing_params)
{
	int ret;

	timing->sjw = timing_params->sjw;
	if (timing_params->sample_point && USE_SP_ALGO) {
		ret = can_calc_timing(dev, timing, timing_params->bus_speed,
				      timing_params->sample_point);
		if (ret < 0) {
			return ret;
		}
		LOG_DBG("Presc: %d, BS1: %d, BS2: %d", timing->prescaler, timing->phase_seg1,
			timing->phase_seg2);
		LOG_DBG("Sample-point err : %d", ret);
	} else {
		timing->prop_seg = timing_params->prop_seg;
		timing->phase_seg1 = timing_params->phase_seg1;
		timing->phase_seg2 = timing_params->phase_seg2;
		ret = can_calc_prescaler(dev, timing, timing_params->bus_speed);
		if (ret > 0) {
			LOG_WRN("Bitrate error: %d", ret);
		}
	}

	return ret;
}

static int mcp25xxfd_init_con_reg(const struct device *dev)
{
	struct mcp25xxfd_con *con;

	con = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*con));
	con->b0.ISOCRCEN = 1;
	con->b1.WAKFIL = 1;
	con->b1.WFT = MCP25XXFD_WFT_T11FILTER;
	con->b2.TXQEN = 1;
	con->b2.STEF = 1;
	con->b3.REQOP = MCP25XXFD_OPMODE_CONFIGURATION;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_CON, sizeof(*con));
}

static int mcp25xxfd_init_osc_reg(const struct device *dev)
{
	int ret;
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_osc *osc = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*osc));

	osc->b0.CLKODIV = dev_cfg->clko_div;
	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_OSC, sizeof(*osc));
	if (ret) {
		return ret;
	}

	if (dev_cfg->pll_enable) {
		struct mcp25xxfd_osc_b0 *osc_b0;
		struct mcp25xxfd_osc_b1 *osc_b1;
		uint8_t reg_value;
		uint8_t reg_value_validate;

		osc_b0 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*osc_b0));
		osc_b0->PLLEN = 1;
		memcpy(&reg_value, osc_b0, 1);

		osc_b1 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*osc_b1));
		osc_b1->PLLRDY = 1;
		osc_b1->OSCRDY = 1;
		memcpy(&reg_value_validate, osc_b1, 1);

		ret = mcp25xxfd_set_bitmask_and_validate_reg_u8(dev,
			MCP25XXFD_REG_OSC, reg_value, reg_value,
			MCP25XXFD_REG_OSC + 1, reg_value_validate, reg_value_validate,
			PLLRDY_TIMEOUT_USEC, PLLRDY_RETRIES, false);
	}

	return ret;
}

static int mcp25xxfd_init_iocon_reg(const struct device *dev)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_iocon *iocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*iocon));

	iocon->TRIS0 = 1;
	iocon->TRIS1 = 1;
	iocon->PM0 = 1;
	iocon->PM1 = 1;
	iocon->SOF = dev_cfg->sof_on_clko ? 1 : 0;

	return  mcp25xxfd_write_reg(dev, MCP25XXFD_REG_IOCON, sizeof(*iocon));
}

static int mcp25xxfd_init_int_hw1_reg(const struct device *dev)
{
	struct mcp25xxfd_int_hw1 *int_enable = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*int_enable));

	int_enable->RXIE = 1;
	int_enable->MODIE = 1;
	int_enable->TEFIE = 1;
	int_enable->CERRIE = 1;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_INT + 2, sizeof(*int_enable));
}

static int mcp25xxfd_init_tef_fifo(const struct device *dev)
{
	struct mcp25xxfd_fifocon *fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));

#if defined(CONFIG_CAN_RX_TIMESTAMP)
	fifocon->b0.TSEN = 1;
#endif
	fifocon->b0.FNEIE = 1;
	fifocon->b1.FRESET = 1;
	fifocon->b3.FSIZE = MCP25XXFD_TX_FIFO_ITEMS - 1;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_TEFCON, sizeof(*fifocon));
}

static int mcp25xxfd_init_tx_fifos(const struct device *dev)
{
	int ret = 0;
	struct mcp25xxfd_fifocon *fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));

	fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));
	fifocon->b0.TXEN = 1;
	fifocon->b1.FRESET = 1;
	fifocon->b2.TXAT = 3;
	fifocon->b3.PLSIZE = can_bytes_to_dlc(MCP25XXFD_PAYLOAD_SIZE) - 8;

	for (int i = 0; i < MCP25XXFD_TX_FIFO_ITEMS; i++) {
		ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_FIFOCON(i), sizeof(*fifocon));
		if (ret < 0) {
			return ret;
		}
	}

	return ret;
}

static int mcp25xxfd_init_rx_fifo(const struct device *dev)
{
	struct mcp25xxfd_fifocon *fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));

	fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));
#if defined(CONFIG_CAN_RX_TIMESTAMP)
	fifocon->b0.TSEN = 1;
#endif
	fifocon->b0.TXEN = 0;
	fifocon->b0.FNEIE = 1;
	fifocon->b1.FRESET = 1;
	fifocon->b3.PLSIZE = can_bytes_to_dlc(MCP25XXFD_PAYLOAD_SIZE) - 8;
	fifocon->b3.FSIZE = MCP25XXFD_RX_FIFO_ITEMS - 1;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_FIFOCON(MCP25XXFD_RX_FIFO_IDX),
				   sizeof(*fifocon));
}


static int mcp25xxfd_init(const struct device *dev)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_con *con;
	int ret;
	struct can_timing timing;

#if defined(CONFIG_CAN_FD_MODE)
	struct can_timing timing_data;
#endif

	dev_data->dev = dev;

	if (dev_cfg->clk_dev) {
		int ret;
		uint32_t clk_id = dev_cfg->clk_id;

		if (!device_is_ready(dev_cfg->clk_dev)) {
			LOG_ERR("Clock controller not ready");
			return -ENODEV;
		}

		ret = clock_control_on(dev_cfg->clk_dev, (clock_control_subsys_t)clk_id);
		if (ret) {
			LOG_ERR("Failed to enable clock");
			return ret;
		}
	}

	k_sem_init(&dev_data->int_sem, 0, 1);
	k_sem_init(&dev_data->tx_sem, MCP25XXFD_TX_FIFO_ITEMS, MCP25XXFD_TX_FIFO_ITEMS);

	k_mutex_init(&dev_data->mutex);

	if (!spi_is_ready_dt(&dev_cfg->bus)) {
		LOG_ERR("SPI bus %s not ready", dev_cfg->bus.bus->name);
		return -ENODEV;
	}

	ret = mcp25xxfd_reset(dev);
	if (ret < 0) {
		LOG_ERR("Failed to reset the device [%d]", ret);
		return -EIO;
	}

	if (!device_is_ready(dev_cfg->int_gpio_dt.port)) {
		LOG_ERR("GPIO port not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INPUT)) {
		LOG_ERR("Unable to configure GPIO pin");
		return -EINVAL;
	}

	gpio_init_callback(&(dev_data->int_gpio_cb), mcp25xxfd_int_gpio_callback,
			   BIT(dev_cfg->int_gpio_dt.pin));

	if (gpio_add_callback(dev_cfg->int_gpio_dt.port, &(dev_data->int_gpio_cb))) {
		return -EINVAL;
	}

	if (gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INT_LEVEL_ACTIVE)) {
		return -EINVAL;
	}

	k_thread_create(&dev_data->int_thread, dev_data->int_thread_stack,
			CONFIG_CAN_MCP25XXFD_INT_THREAD_STACK_SIZE,
			(k_thread_entry_t)mcp25xxfd_int_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_CAN_MCP25XXFD_INT_THREAD_PRIO), 0, K_NO_WAIT);

	dev_data->sjw = dev_cfg->timing_params.sjw;
	ret = mcp25xxfd_init_timing_struct(&timing, dev, &dev_cfg->timing_params);
	if (ret < 0) {
		LOG_ERR("Can't find timing for given param");
		return ret;
	}

#if defined(CONFIG_CAN_FD_MODE)
	dev_data->sjw_data = dev_cfg->timing_params_data.sjw;
	ret = mcp25xxfd_init_timing_struct(&timing_data, dev, &dev_cfg->timing_params_data);
	if (ret < 0) {
		LOG_ERR("Can't find timing for given param");
		return ret;
	}
#endif

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	con = mcp25xxfd_read_reg(dev, MCP25XXFD_REG_CON, sizeof(*con));
	if (!con) {
		ret = -EINVAL;
		goto done;
	} else if (con->b2.OPMOD != MCP25XXFD_OPMODE_CONFIGURATION) {
		LOG_ERR("Device did not reset into configuration mode [%d]", con->b2.OPMOD);
		ret = -EIO;
		goto done;
	}

	ret = mcp25xxfd_init_con_reg(dev);
	if (ret) {
		goto done;
	}

	ret = mcp25xxfd_init_osc_reg(dev);
	if (ret) {
		goto done;
	}

	ret = mcp25xxfd_init_iocon_reg(dev);
	if (ret) {
		goto done;
	}

	ret = mcp25xxfd_init_int_hw1_reg(dev);
	if (ret) {
		goto done;
	}

	ret = mcp25xxfd_init_tef_fifo(dev);
	if (ret) {
		goto done;
	}

	ret = mcp25xxfd_init_tx_fifos(dev);
	if (ret) {
		goto done;
	}

	ret = mcp25xxfd_init_rx_fifo(dev);
	if (ret) {
		goto done;
	}

	LOG_DBG("%d TX FIFOS: 1 element", MCP25XXFD_TX_FIFO_ITEMS);
	LOG_DBG("1 RX FIFO: %d elements", MCP25XXFD_RX_FIFO_ITEMS);
	LOG_DBG("%db of %db RAM Allocated",
		MCP25XXFD_TEF_FIFO_SIZE + MCP25XXFD_TX_FIFO_SIZE + MCP25XXFD_RX_FIFO_SIZE,
		MCP25XXFD_RAM_SIZE);

done:
	k_mutex_unlock(&dev_data->mutex);
	if (ret) {
		return ret;
	}

	ret = can_set_timing(dev, &timing);
	if (ret < 0) {
		return ret;
	}

#if defined(CONFIG_CAN_FD_MODE)
	ret = can_set_timing_data(dev, &timing_data);
	if (ret < 0) {
		return ret;
	}
#endif

#if defined(CONFIG_CAN_FD_MODE)
	ret = can_set_mode(dev, CAN_MODE_FD);
#else
	ret = can_set_mode(dev, CAN_MODE_NORMAL);
#endif

	return ret;
}

#define MCP25XXFD_SET_TIMING_MACRO(inst, type)                                                     \
	.timing_params##type = {                                                                   \
		.sjw = DT_INST_PROP(inst, sjw##type),                                              \
		.prop_seg = DT_INST_PROP_OR(inst, prop_seg##type, 0),                              \
		.phase_seg1 = DT_INST_PROP_OR(inst, phase_seg1##type, 0),                          \
		.phase_seg2 = DT_INST_PROP_OR(inst, phase_seg2##type, 0),                          \
		.bus_speed = DT_INST_PROP(inst, bus_speed##type),                                  \
		.sample_point = DT_INST_PROP_OR(inst, sample_point##type, 0),                      \
	}

#if defined(CONFIG_CAN_FD_MODE)
#define MCP25XXFD_SET_TIMING(inst)                                                                 \
	MCP25XXFD_SET_TIMING_MACRO(inst,),                                                         \
	MCP25XXFD_SET_TIMING_MACRO(inst, _data)
#else
#define MCP25XXFD_SET_TIMING(inst)                                                                 \
	MCP25XXFD_SET_TIMING_MACRO(inst,)
#endif

#define MCP25XXFD_SET_CLOCK(inst)                                                                  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clocks),                                           \
		    (.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                          \
		     .clk_id = DT_INST_CLOCKS_CELL(inst, id)),                                     \
		    ())


#define MCP25XXFD_INIT(inst)                                                                       \
	static K_KERNEL_STACK_DEFINE(mcp25xxfd_int_stack_##inst,                                   \
				     CONFIG_CAN_MCP25XXFD_INT_THREAD_STACK_SIZE);                  \
                                                                                                   \
	static struct mcp25xxfd_data mcp25xxfd_data_##inst = {                                     \
		.int_thread_stack = mcp25xxfd_int_stack_##inst,                                    \
	};                                                                                         \
	static const struct mcp25xxfd_config mcp25xxfd_config_##inst = {                           \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0),                             \
		.int_gpio_dt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                             \
                                                                                                   \
		.sof_on_clko = DT_INST_PROP(inst, sof_on_clko),                                    \
		.clko_div = DT_INST_ENUM_IDX(inst, clko_div),                                      \
		.pll_enable = DT_INST_PROP(inst, pll_enable),                                      \
                                                                                                   \
		.osc_freq = DT_INST_PROP(inst, osc_freq),                                          \
		MCP25XXFD_SET_TIMING(inst),                                                        \
		.phy = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, phys)),                         \
		.max_bitrate = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(inst, 8000000),                 \
		.rx_fifo = {.ram_start_addr = MCP25XXFD_RX_FIFO_START_ADDR,                        \
			    .reg_fifocon_addr = MCP25XXFD_REG_FIFOCON(MCP25XXFD_RX_FIFO_IDX),      \
			    .capacity = MCP25XXFD_RX_FIFO_ITEMS,                                   \
			    .item_size = MCP25XXFD_RX_FIFO_ITEM_SIZE,                              \
			    .msg_handler = mcp25xxfd_rx_fifo_handler},                             \
		.tef_fifo = {.ram_start_addr = MCP25XXFD_TEF_FIFO_START_ADDR,                      \
			     .reg_fifocon_addr = MCP25XXFD_REG_TEFCON,                             \
			     .capacity = MCP25XXFD_TEF_FIFO_ITEMS,                                 \
			     .item_size = MCP25XXFD_TEF_FIFO_ITEM_SIZE,                            \
			     .msg_handler = mcp25xxfd_tef_fifo_handler},                           \
		MCP25XXFD_SET_CLOCK(inst)                                                          \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &mcp25xxfd_init, NULL, &mcp25xxfd_data_##inst,                 \
			      &mcp25xxfd_config_##inst, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,     \
			      &can_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(MCP25XXFD_INIT)
