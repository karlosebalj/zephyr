/*
 * Copyright (c) 2020 Abram Early
 * Copyright (c) 2023 Andriy Gelman
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp25xxfd

#include "can_mcp25xxfd.h"

#include <zephyr/device.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(can_mcp25xxfd, CONFIG_CAN_LOG_LEVEL);

#define SP_IS_SET(inst) DT_INST_NODE_HAS_PROP(inst, sample_point) ||

/*
 * Macro to exclude the sample point algorithm from compilation if not used
 * Without the macro, the algorithm would always waste ROM
 */
#define USE_SP_ALGO (DT_INST_FOREACH_STATUS_OKAY(SP_IS_SET) 0)

/* Timeout for changing mode */
#define MODE_CHANGE_TIMEOUT_USEC (CONFIG_CAN_MCP25XXFD_MODE_CHANGE_TIMEOUT_MSEC * 1000U)
#define MODE_CHANGE_RETRIES      100

#define PLLRDY_TIMEOUT_USEC 100000
#define PLLRDY_RETRIES      100

#define MAX_INT_HANDLER_CALLS  10
#define INT_HANDLER_SLEEP_USEC 10000

static void mcp25xxfd_canframe_to_txobj(const struct can_frame *src, struct mcp25xxfd_txobj *dst)
{
	memset(dst, 0, offsetof(struct mcp25xxfd_txobj, DATA));

	if ((src->flags & CAN_FRAME_IDE) != 0) {
		dst->SID = src->id >> 18;
		dst->EID = src->id;
		dst->IDE = 1;
	} else {
		dst->SID = src->id;
	}

	if ((src->flags & CAN_FRAME_BRS) != 0) {
		dst->BRS = 1;
	}

	if ((src->flags & CAN_FRAME_RTR) != 0) {
		dst->RTR = 1;
	}
	dst->DLC = src->dlc;
#if defined(CONFIG_CAN_FD_MODE)
	if ((src->flags & CAN_FRAME_FDF) != 0) {
		dst->FDF = 1;
	}
#endif
	memcpy(dst->DATA, src->data, MIN(can_dlc_to_bytes(src->dlc), CAN_MAX_DLEN));
}

static void *mcp25xxfd_read_reg(const struct device *dev, uint16_t addr, int len)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_spi_data *spi_data = &dev_data->spi_data;
	uint16_t spi_cmd;
	int ret;

	spi_cmd = sys_cpu_to_be16(MCP25XXFD_SPI_INSTRUCTION_READ | addr);
	memcpy(&spi_data->header[1], &spi_cmd, sizeof(spi_cmd));

	struct spi_buf tx_buf = {.buf = &spi_data->header[1], .len = MCP25XXFD_SPI_CMD_LEN + len};
	struct spi_buf rx_buf = {.buf = &spi_data->header[1], .len = MCP25XXFD_SPI_CMD_LEN + len};

	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

	ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
	if (ret < 0) {
		return NULL;
	}

	return &spi_data->buf[0];
}

static void *mcp25xxfd_read_reg_crc(const struct device *dev, uint16_t addr, int len)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_spi_data *spi_data = &dev_data->spi_data;
	int num_retries = CONFIG_CAN_MCP25XXFD_READ_CRC_RETRIES + 1;
	int ret;

	while (num_retries-- > 0) {
		uint16_t crc_in, crc, spi_cmd;

		struct spi_buf tx_buf = {.buf = spi_data,
					 .len = MCP25XXFD_SPI_CMD_LEN +
						MCP25XXFD_SPI_LEN_FIELD_LEN + len +
						MCP25XXFD_SPI_CRC_LEN};

		struct spi_buf rx_buf = {.buf = spi_data,
					 .len = MCP25XXFD_SPI_CMD_LEN +
						MCP25XXFD_SPI_LEN_FIELD_LEN + len +
						MCP25XXFD_SPI_CRC_LEN};

		const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
		const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

		spi_cmd = sys_cpu_to_be16(MCP25XXFD_SPI_INSTRUCTION_READ_CRC | addr);
		memcpy(&spi_data->header[0], &spi_cmd, sizeof(spi_cmd));
		spi_data->header[2] = len;

		/*
		 * Evaluate initial crc over spi_cmd and length as these value will change after
		 * spi transaction is finished.
		 */
		crc_in = crc16(MCP25XXFD_CRC_POLY, MCP25XXFD_CRC_SEED, (uint8_t *)spi_data,
			       MCP25XXFD_SPI_CMD_LEN + MCP25XXFD_SPI_LEN_FIELD_LEN);

		ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
		if (ret < 0) {
			continue;
		}

		/* Continue crc calculation over the data field and the crc field */
		crc = crc16(MCP25XXFD_CRC_POLY, crc_in, &spi_data->buf[0],
			    len + MCP25XXFD_SPI_CRC_LEN);
		if (crc == 0) {
			return &spi_data->buf[0];
		}
	}

	return NULL;
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
	uint16_t spi_cmd;

	struct spi_buf tx_buf = {.buf = &spi_data->header[1], .len = MCP25XXFD_SPI_CMD_LEN + len};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	spi_cmd = sys_cpu_to_be16(MCP25XXFD_SPI_INSTRUCTION_WRITE | addr);
	memcpy(&spi_data->header[1], &spi_cmd, sizeof(spi_cmd));

	return spi_write_dt(&dev_cfg->bus, &tx);
}

static int mcp25xxfd_fifo_write(const struct device *dev, int mailbox_idx, uint16_t fifo_address,
				const struct can_frame *msg)
{
	struct mcp25xxfd_fiforegs *fiforegs;
	struct mcp25xxfd_txobj *txobj;
	struct mcp25xxfd_fifocon_b1 *fifocon_b1;
	uint16_t address;
	int tx_len;
	int ret;

	fiforegs = mcp25xxfd_read_reg_crc(dev, fifo_address, sizeof(*fiforegs));
	if (!fiforegs) {
		LOG_ERR("Failed to read 12 bytes from FIFOCON");
		return -EINVAL;
	}

	/* check if fifo is full */
	if (!fiforegs->fifosta.b0.FNEIF) {
		return -ENOMEM;
	}

	address = MCP25XXFD_RAM_START_ADDR + fiforegs->ua;

	txobj = mcp25xxfd_get_spi_buf_ptr(dev, 0);
	mcp25xxfd_canframe_to_txobj(msg, txobj);

	txobj->SEQ = mailbox_idx;
	tx_len = MCP25XXFD_TXQ_OBJ_HEADER_SIZE +
		 ROUND_UP(can_dlc_to_bytes(msg->dlc), MCP25XXFD_RAM_ALIGNMENT);

	ret = mcp25xxfd_write_reg(dev, address, tx_len);
	if (ret < 0) {
		return ret;
	}

	fifocon_b1 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon_b1));
	fifocon_b1->UINC = 1;
	fifocon_b1->TXREQ = 1;

	return mcp25xxfd_write_reg(dev, fifo_address + 1, sizeof(*fifocon_b1));
}

static void mcp25xxfd_rxobj_to_canframe(const struct mcp25xxfd_rxobj *src, struct can_frame *dst)
{
	memset(dst, 0, offsetof(struct can_frame, data));

	if (src->IDE != 0) {
		dst->id = src->EID | (src->SID << 18);
		dst->flags |= CAN_FRAME_IDE;
	} else {
		dst->id = src->SID;
	}

	if (src->BRS != 0) {
		dst->flags |= CAN_FRAME_BRS;
	}

	if (src->RTR != 0) {
		dst->flags |= CAN_FRAME_RTR;
	}

#if defined(CONFIG_CAN_FD_MODE)
	if (src->FDF != 0) {
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

	con_b2 = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_CON_B2, sizeof(*con_b2));
	if (!con_b2) {
		return -EINVAL;
	}

	*mode = con_b2->OPMOD;

	return 0;
}

/*
 * Helper function to set a value in one u8 register and
 * validate that the value in another register
 */
static int mcp25xxfd_set_bitmask_and_validate_reg_u8(const struct device *dev, uint16_t addr_write,
						     uint8_t value_write, uint8_t mask_write,
						     uint16_t addr_validate, uint8_t value_validate,
						     uint8_t mask_validate, uint32_t timeout_usec,
						     int retries, bool allow_yield)
{
	uint8_t *data;
	int ret;
	uint32_t delay = timeout_usec / retries;
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_spi_data *spi_data = &dev_data->spi_data;

	data = mcp25xxfd_read_reg_crc(dev, addr_write, 1);
	if (!data) {
		return -EINVAL;
	}

	*data &= ~mask_write;
	*data |= value_write;

	/*
	 * Move data into the correct byte location. READ_CRC uses extra length field,
	 * so the data field is no longer is correct place for write_reg call.
	 */
	spi_data->buf[0] = *data;

	ret = mcp25xxfd_write_reg(dev, addr_write, 1);
	if (ret < 0) {
		return ret;
	}

	for (;;) {
		data = mcp25xxfd_read_reg_crc(dev, addr_validate, 1);
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

static int mcp25xxfd_set_tdc(const struct device *dev, bool is_enabled, int tdc_offset)
{
	struct mcp25xxfd_tdc *tdc;

	if (is_enabled && (tdc_offset < MCP25XXFD_TDCO_MIN || tdc_offset > MCP25XXFD_TDCO_MAX)) {
		return -EINVAL;
	}

	tdc = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*tdc));

	if (is_enabled) {
		tdc->TDCMOD = MCP25XXFD_TDCMOD_AUTO;
		tdc->TDCO = tdc_offset;

	} else {
		tdc->TDCMOD = MCP25XXFD_TDCMOD_DISABLED;
	}

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_TDC, sizeof(*tdc));
}

static int mcp25xxfd_set_mode_internal(const struct device *dev, uint8_t requested_mode)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_con_b2 *con_b2;
	struct mcp25xxfd_con_b3 *con_b3;
	uint8_t reg_value;
	uint8_t reg_value_validate;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	con_b2 = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_CON_B2, sizeof(*con_b2));
	if (!con_b2) {
		ret = -EINVAL;
		goto done;
	}

	if (con_b2->OPMOD == requested_mode) {
		goto done;
	}

#if defined(CONFIG_CAN_FD_MODE)
	if (dev_data->current_mcp25xxfd_mode == MCP25XXFD_OPMODE_CONFIGURATION) {
		if (requested_mode == MCP25XXFD_OPMODE_NORMAL_CAN2 ||
		    requested_mode == MCP25XXFD_OPMODE_EXT_LOOPBACK ||
		    requested_mode == MCP25XXFD_OPMODE_INT_LOOPBACK) {
			ret = mcp25xxfd_set_tdc(dev, false, 0);
		} else if (requested_mode == MCP25XXFD_OPMODE_NORMAL_CANFD) {
			ret = mcp25xxfd_set_tdc(dev, true, dev_data->tdco);
		}

		if (ret < 0) {
			goto done;
		}
	}
#endif

	/* a bit of hack to get bit proper bit fields set */
	con_b2 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*con_b2));
	con_b2->OPMOD = requested_mode;
	memcpy(&reg_value_validate, con_b2, sizeof(*con_b2));

	con_b3 = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*con_b3));
	con_b3->REQOP = requested_mode;
	memcpy(&reg_value, con_b3, sizeof(*con_b3));

	/* we need to request the mode in one register and validate it another */
	ret = mcp25xxfd_set_bitmask_and_validate_reg_u8(
		dev, MCP25XXFD_REG_CON_B3, reg_value, MCP25XXFD_CON_B3_REQOP_MASK,
		MCP25XXFD_REG_CON_B2, reg_value_validate, MCP25XXFD_CON_B2_OPMOD_MASK,
		MODE_CHANGE_TIMEOUT_USEC, MODE_CHANGE_RETRIES, true);
done:
	k_mutex_unlock(&dev_data->mutex);
	return ret;
}

static int mcp25xxfd_set_mode(const struct device *dev, can_mode_t mode)
{
	struct mcp25xxfd_data *dev_data = dev->data;

	if (dev_data->started) {
		return -EBUSY;
	}

	/* todo: Add CAN_MODE_ONE_SHOT support */
	if ((mode & (CAN_MODE_3_SAMPLES | CAN_MODE_ONE_SHOT)) != 0) {
		return -ENOTSUP;
	}

	if (mode == CAN_MODE_NORMAL) {
		dev_data->next_mcp25xxfd_mode = MCP25XXFD_OPMODE_NORMAL_CAN2;
	}

	if ((mode & CAN_MODE_FD) != 0) {
#if defined(CONFIG_CAN_FD_MODE)
		dev_data->next_mcp25xxfd_mode = MCP25XXFD_OPMODE_NORMAL_CANFD;
#else
		return -ENOTSUP;
#endif
	}

	if ((mode & CAN_MODE_LISTENONLY) != 0) {
		dev_data->next_mcp25xxfd_mode = MCP25XXFD_OPMODE_LISTEN_ONLY;
	}

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		dev_data->next_mcp25xxfd_mode = MCP25XXFD_OPMODE_EXT_LOOPBACK;
	}

	dev_data->mode = mode;

	return 0;
}

static int mcp25xxfd_set_timing(const struct device *dev, const struct can_timing *timing,
				bool is_nominal)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_nbtcfg *nominal_timing_reg;
	struct mcp25xxfd_dbtcfg *data_timing_reg;
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

	__ASSERT_NO_MSG(timing->prop_seg == 0);
	if (is_nominal) {
		__ASSERT_NO_MSG(timing->sjw == CAN_SJW_NO_CHANGE ||
				(timing->sjw >= 1 && timing->sjw <= 128));
		__ASSERT_NO_MSG(timing->phase_seg1 >= 2 && timing->phase_seg1 <= 256);
		__ASSERT_NO_MSG(timing->phase_seg2 >= 1 && timing->phase_seg2 <= 128);
		__ASSERT_NO_MSG(timing->prescaler >= 1 && timing->prescaler <= 256);
	} else {
		__ASSERT_NO_MSG(timing->sjw == CAN_SJW_NO_CHANGE ||
				(timing->sjw >= 1 && timing->sjw <= 16));
		__ASSERT_NO_MSG(timing->phase_seg1 >= 1 && timing->phase_seg1 <= 32);
		__ASSERT_NO_MSG(timing->phase_seg2 >= 1 && timing->phase_seg2 <= 16);
		__ASSERT_NO_MSG(timing->prescaler >= 1 && timing->prescaler <= 256);
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

#ifdef CONFIG_CAN_FD_MODE
	if (!is_nominal) {
		dev_data->tdco = timing->prescaler * (timing->prop_seg + timing->phase_seg1);
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
	int ret = 0;

	LOG_DBG("Sending %d bytes. Id: 0x%x, ID type: %s %s %s %s", can_dlc_to_bytes(msg->dlc),
		msg->id, msg->flags & CAN_FRAME_IDE ? "extended" : "standard",
		msg->flags & CAN_FRAME_RTR ? "RTR" : "",
		msg->flags & CAN_FRAME_FDF ? "FD frame" : "",
		msg->flags & CAN_FRAME_BRS ? "BRS" : "");

	__ASSERT_NO_MSG(callback != NULL);

	if (!dev_data->started) {
		return -ENETDOWN;
	}

	if (dev_data->state == CAN_STATE_BUS_OFF) {
		return -ENETUNREACH;
	}

	if ((msg->flags & CAN_FRAME_FDF) == 0 && msg->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d without fd flag set.", msg->dlc);
		return -EINVAL;
	}

	if ((msg->flags & CAN_FRAME_FDF) && !(dev_data->mode & CAN_MODE_FD)) {
		return -ENOTSUP;
	}

	if (k_sem_take(&dev_data->tx_sem, timeout) != 0) {
		return -EAGAIN;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);
	for (; mailbox_idx < MCP25XXFD_TX_QUEUE_ITEMS; mailbox_idx++) {
		if ((BIT(mailbox_idx) & dev_data->mailbox_usage) == 0) {
			dev_data->mailbox_usage |= BIT(mailbox_idx);
			break;
		}
	}

	if (mailbox_idx >= MCP25XXFD_TX_QUEUE_ITEMS) {
		k_sem_give(&dev_data->tx_sem);
		ret = -EIO;
		goto done;
	}

	dev_data->mailbox[mailbox_idx].cb = callback;
	dev_data->mailbox[mailbox_idx].cb_arg = callback_arg;

	ret = mcp25xxfd_fifo_write(dev, mailbox_idx, MCP25XXFD_REG_TXQCON, msg);

	if (ret < 0) {
		dev_data->mailbox_usage &= ~BIT(mailbox_idx);
		dev_data->mailbox[mailbox_idx].cb = NULL;
		k_sem_give(&dev_data->tx_sem);
	}

done:
	k_mutex_unlock(&dev_data->mutex);
	return ret;
}

static int mcp25xxfd_add_rx_filter(const struct device *dev, can_rx_callback_t rx_cb, void *cb_arg,
				   const struct can_filter *filter)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	struct mcp25xxfd_fltcon *fltcon;
	struct mcp25xxfd_mask *mask;
	struct mcp25xxfd_fltobj *fltobj;
	int filter_idx = 0;
	int ret;

	__ASSERT(rx_cb != NULL, "rx_cb can not be null");
	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	for (; filter_idx < CONFIG_CAN_MAX_FILTER ; filter_idx++) {
		if ((BIT(filter_idx) & dev_data->filter_usage) == 0) {
			break;
		}
	}

	if (filter_idx >= CONFIG_CAN_MAX_FILTER) {
		filter_idx = -ENOSPC;
		goto done;
	}

	if ((filter->flags & CAN_FILTER_RTR) != 0) {
		filter_idx = -ENOTSUP;
		goto done;
	}

	fltobj = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fltobj));

	if ((filter->flags & CAN_FILTER_IDE) != 0) {
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
	if ((filter->flags & CAN_FILTER_IDE) != 0) {
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
	struct mcp25xxfd_data *dev_data = dev->data;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	trec = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_TREC, sizeof(*trec));
	if (!trec) {
		ret = -EINVAL;
		goto done;
	}

	if (err_cnt != NULL) {
		err_cnt->tx_err_cnt = trec->REC;
		err_cnt->rx_err_cnt = trec->TEC;
	}

	if (state == NULL) {
		goto done;
	}

	if (!dev_data->started) {
		*state = CAN_STATE_STOPPED;
		goto done;
	}

	if (trec->TXBO != 0) {
		*state = CAN_STATE_BUS_OFF;
	} else if (trec->TXBP || trec->RXBP) {
		*state = CAN_STATE_ERROR_PASSIVE;
	} else if (trec->TXWARN || trec->RXWARN) {
		*state = CAN_STATE_ERROR_WARNING;
	} else {
		*state = CAN_STATE_ERROR_ACTIVE;
	}

done:
	k_mutex_unlock(&dev_data->mutex);
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
static int mcp25xxfd_recover(const struct device *dev, k_timeout_t timeout)
{
	struct mcp25xxfd_data *dev_data = dev->data;

	ARG_UNUSED(timeout);

	if (!dev_data->started) {
		return -ENETDOWN;
	}

	return -ENOTSUP;
}
#endif

static int mcp25xxfd_handle_fifo_read(const struct device *dev, const struct mcp25xxfd_fifo *fifo,
				      uint8_t fifo_type)
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

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	/* read in FIFOSTA and FIFOUA at the same time */
	fifosta_ua = mcp25xxfd_read_reg_crc(dev,
					    MCP25XXFD_REG_FIFOCON_TO_STA(fifo->reg_fifocon_addr),
					    sizeof(*fifosta_ua));
	if (!fifosta_ua) {
		ret = -EINVAL;
		goto done;
	}

	/* is there any data in the fifo? */
	if (!fifosta_ua->fifosta.b0.FNEIF) {
		goto done;
	}

	fifo_tail_addr = fifosta_ua->ua;
	fifo_tail_index = (fifo_tail_addr - fifo->ram_start_addr) / fifo->item_size;

	if (fifo_type == MCP25XXFD_FIFO_TYPE_RX) {
		/*
		 * fifo_head_index points where the next message will be written.
		 * It points to one past the end of the fifo.
		 */
		fifo_head_index = fifosta_ua->fifosta.b1.FIFOCI;
		if (fifo_head_index == 0) {
			fifo_head_index = fifo->capacity - 1;
		} else {
			fifo_head_index -= 1;
		}

		if (fifo_tail_index > fifo_head_index) {
			/* fetch to the end of the memory and then wrap to the start */
			fetch_total = fifo->capacity - 1 - fifo_tail_index + 1;
			fetch_total += fifo_head_index + 1;
		} else {
			fetch_total = fifo_head_index - fifo_tail_index + 1;
		}
	} else if (fifo_type == MCP25XXFD_FIFO_TYPE_TEF) {
		/* FIFOCI doesn't exist for TEF queues, so fetch one message at a time */
		fifo_head_index = fifo_tail_index;
		fetch_total = 1;
	} else {
		LOG_ERR("Invalid Fifo type");
		ret = -EINVAL;
		goto done;
	}

	while (fetch_total > 0) {
		uint16_t memory_addr;
		uint8_t *data;

		if (fifo_tail_index > fifo_head_index) {
			len = fifo->capacity - 1 - fifo_tail_index + 1;
		} else {
			len = fifo_head_index - fifo_tail_index + 1;
		}

		memory_addr = MCP25XXFD_RAM_START_ADDR + fifo->ram_start_addr +
			      fifo_tail_index * fifo->item_size;

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
	k_mutex_unlock(&dev_data->mutex);
	return ret;
}

static void mcp25xxfd_reset_tx_fifos(const struct device *dev, int status)
{
	struct mcp25xxfd_data *dev_data = dev->data;

	LOG_INF("All FIFOs Reset");
	k_mutex_lock(&dev_data->mutex, K_FOREVER);
	for (int i = 0; i < MCP25XXFD_TX_QUEUE_ITEMS; i++) {
		can_tx_callback_t callback;

		if (!(dev_data->mailbox_usage & BIT(i))) {
			continue;
		}

		callback = dev_data->mailbox[i].cb;
		if (callback) {
			callback(dev, status, dev_data->mailbox[i].cb_arg);
		}

		dev_data->mailbox_usage &= ~BIT(i);
		dev_data->mailbox[i].cb = NULL;
		k_sem_give(&dev_data->tx_sem);
	}
	k_mutex_unlock(&dev_data->mutex);
}

/*
 * CERRIF will be set each time a threshold in the TEC/REC counter is crossed by the following
 * conditions:
 * • TEC or REC exceeds the Error Warning state threshold
 * • The transmitter or receiver transitions to Error Passive state
 * • The transmitter transitions to Bus Off state
 * • The transmitter or receiver transitions from Error Passive to Error Active state
 * • The module transitions from Bus Off to Error Active state, after the bus off recovery
 * sequence
 * When the user clears CERRIF, it will remain clear until a new counter crossing occurs.
 */
static int mcp25xxfd_handle_cerrif(const struct device *dev)
{
	struct mcp25xxfd_trec *trec;
	enum can_state new_state;
	struct mcp25xxfd_data *dev_data = dev->data;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	trec = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_TREC, sizeof(*trec));
	if (!trec) {
		ret = -EINVAL;
		goto done;
	}

	if (trec->TXBO != 0) {
		new_state = CAN_STATE_BUS_OFF;
	} else if (trec->TXBP || trec->RXBP) {
		new_state = CAN_STATE_ERROR_PASSIVE;
	} else if (trec->TXWARN || trec->RXWARN) {
		new_state = CAN_STATE_ERROR_WARNING;
	} else {
		new_state = CAN_STATE_ERROR_ACTIVE;
	}

	if (new_state == dev_data->state) {
		goto done;
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

done:
	k_mutex_unlock(&dev_data->mutex);
	return ret;
}

static int mcp25xxfd_handle_modif(const struct device *dev)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	uint8_t mode;
	int ret;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = mcp25xxfd_get_mode_internal(dev, &mode);
	if (ret < 0) {
		goto finish;
	}

	dev_data->current_mcp25xxfd_mode = mode;

	LOG_INF("Switched to mode %d", mode);

	if (mode == dev_data->next_mcp25xxfd_mode) {
		ret = 0;
		goto finish;
	}

	/* try to transition back into our target mode */
	if (dev_data->started) {
		LOG_INF("Switching back into mode %d", dev_data->next_mcp25xxfd_mode);
		ret =  mcp25xxfd_set_mode_internal(dev, dev_data->next_mcp25xxfd_mode);
	}

finish:
	k_mutex_unlock(&dev_data->mutex);
	return ret;
}

static int mcp25xxfd_handle_ivmif(const struct device *dev)
{
	struct mcp25xxfd_bdiag1 *diag;
	struct mcp25xxfd_data *dev_data = dev->data;
	int ret;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	diag = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_BDIAG1, sizeof(*diag));
	if (!diag) {
		ret = -EINVAL;
		goto done;
	}

	if (diag->TXBOERR != 0) {
		LOG_INF("ivmif error");
		mcp25xxfd_reset_tx_fifos(dev, -ENETDOWN);
	}

	/* Clear the values in diag */
	diag = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*diag));
	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_BDIAG1, sizeof(*diag));

done:
	k_mutex_unlock(&dev_data->mutex);
	return ret;
}

static void mcp25xxfd_handle_interrupts(const struct device *dev)
{
	struct mcp25xxfd_int_hw0 int_flags;
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;
	int ret;
	uint8_t consecutive_calls = 0;

	while (1) {
		struct mcp25xxfd_int_hw0 *int_flags_ptr;

		k_mutex_lock(&dev_data->mutex, K_FOREVER);
		int_flags_ptr = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_INT,
						       sizeof(*int_flags_ptr));

		if (!int_flags_ptr) {
			k_mutex_unlock(&dev_data->mutex);
			continue;
		}

		memcpy(&int_flags, int_flags_ptr, sizeof(*int_flags_ptr));

		/* these interrupt flags need to be explicitly cleared */
		if (int_flags_ptr->IVMIF || int_flags_ptr->WAKIF || int_flags_ptr->CERRIF ||
		    int_flags_ptr->SERRIF || int_flags_ptr->MODIF) {
			struct mcp25xxfd_int_hw0 int_flags_cleared = *int_flags_ptr;

			int_flags_cleared.IVMIF = 0;
			int_flags_cleared.WAKIF = 0;
			int_flags_cleared.CERRIF = 0;
			int_flags_cleared.SERRIF = 0;
			int_flags_cleared.MODIF = 0;

			int_flags_ptr = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*int_flags_ptr));
			memcpy(int_flags_ptr, &int_flags_cleared, sizeof(*int_flags_ptr));

			ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_INT, sizeof(*int_flags_ptr));
			if (ret) {
				LOG_ERR("Error clearing interrupts");
			}
		}

		k_mutex_unlock(&dev_data->mutex);

		if (int_flags.RXIF != 0) {
			ret = mcp25xxfd_handle_fifo_read(dev, &dev_cfg->rx_fifo,
							 MCP25XXFD_FIFO_TYPE_RX);
			if (ret < 0) {
				LOG_ERR("Error handling RXIF flag %d", ret);
			}
		}

		if (int_flags.TEFIF != 0) {
			ret = mcp25xxfd_handle_fifo_read(dev, &dev_cfg->tef_fifo,
							 MCP25XXFD_FIFO_TYPE_TEF);
			if (ret < 0) {
				LOG_ERR("Error handling TEFIF flag %d", ret);
			}
		}

		if (int_flags.IVMIF != 0) {
			ret = mcp25xxfd_handle_ivmif(dev);
			if (ret < 0) {
				LOG_ERR("Error handling ivmif %d", ret);
			}
		}

		if (int_flags.MODIF != 0) {
			ret = mcp25xxfd_handle_modif(dev);
			if (ret < 0) {
				LOG_ERR("Error handling modif %d", ret);
			}
		}

		/*
		 * From Linux mcp251xfd driver
		 * On the MCP2527FD and MCP2518FD, we don't get a CERRIF IRQ on the transition
		 * TX ERROR_WARNING -> TX ERROR_ACTIVE.
		 */
		if (int_flags.CERRIF || dev_data->state > CAN_STATE_ERROR_ACTIVE) {
			ret = mcp25xxfd_handle_cerrif(dev);
			if (ret < 0) {
				LOG_ERR("Error handling cerrif");
			}
		}

		/* Break from loop if INT pin is inactive */
		consecutive_calls++;
		ret = gpio_pin_get_dt(&dev_cfg->int_gpio_dt);
		if (ret < 0) {
			LOG_ERR("Couldn't read INT pin");
		} else if (ret == 0) {
			/* All interrupt flags handled */
			break;
		} else if (consecutive_calls % MAX_INT_HANDLER_CALLS == 0) {
			/* If there are clock signal issues, it's not possible to clear */
			/* MODIF flag. Handle this by releasing thread if there are too  */
			/* many consecutive calls */
			k_sleep(K_USEC(INT_HANDLER_SLEEP_USEC));
		}
	}
}

static void mcp25xxfd_int_thread(const struct device *dev)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_data *dev_data = dev->data;

	while (1) {
		int ret;

		k_sem_take(&dev_data->int_sem, K_FOREVER);
		mcp25xxfd_handle_interrupts(dev);

		/* Re-enable pin interrupts */
		ret = gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INT_LEVEL_ACTIVE);
		if (ret < 0) {
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
	int ret;

	/* Disable pin interrupts */
	ret = gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("Couldn't disable pin interrupt");
		k_oops();
	}

	k_sem_give(&dev_data->int_sem);
}

static int mcp25xxfd_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LISTENONLY | CAN_MODE_LOOPBACK;

#if defined(CONFIG_CAN_FD_MODE)
	*cap |= CAN_MODE_FD;
#endif

	return 0;
}

static int mcp25xxfd_start(const struct device *dev)
{
	struct mcp25xxfd_data *dev_data = dev->data;
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	int ret;

	if (dev_data->started) {
		return -EALREADY;
	}

	/* in case of a race between mcp25xxfd_send() and mcp25xxfd_stop() */
	mcp25xxfd_reset_tx_fifos(dev, -ENETDOWN);

	if (dev_cfg->phy != NULL) {
		ret = can_transceiver_enable(dev_cfg->phy);
		if (ret < 0) {
			LOG_ERR("Failed to enable CAN transceiver [%d]", ret);
			return ret;
		}
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = mcp25xxfd_set_mode_internal(dev, dev_data->next_mcp25xxfd_mode);
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

	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_CON_B3, sizeof(*con_b3));
	if (ret < 0) {
		k_mutex_unlock(&dev_data->mutex);
		return ret;
	}

	/* wait for all the messages to be aborted */
	while (1) {
		con_b3 = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_CON_B3, sizeof(*con_b3));

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
		if (ret < 0) {
			LOG_ERR("Failed to disable CAN transceiver [%d]", ret);
			return ret;
		}
	}

	return 0;
}

static const struct can_driver_api can_api_funcs = {
	.get_capabilities = mcp25xxfd_get_capabilities,
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
	if ((dev_data->filter_usage & BIT(filhit)) != 0) {
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
	if (mailbox_idx >= MCP25XXFD_TX_QUEUE_ITEMS) {
		mcp25xxfd_reset_tx_fifos(dev, -EIO);
		LOG_ERR("Invalid mailbox index");
		return;
	}

	callback = dev_data->mailbox[mailbox_idx].cb;
	if (callback != NULL) {
		callback(dev, 0, dev_data->mailbox[mailbox_idx].cb_arg);
	}

	dev_data->mailbox_usage &= ~BIT(mailbox_idx);
	dev_data->mailbox[mailbox_idx].cb = NULL;
	k_sem_give(&dev_data->tx_sem);
}

static int mcp25xxfd_init_timing_struct(struct can_timing *timing,
					const struct device *dev,
					const struct mcp25xxfd_timing_params *timing_params,
					bool is_nominal)
{
	int ret;

	timing->sjw = timing_params->sjw;
	if (USE_SP_ALGO && timing_params->sample_point > 0) {

		if (is_nominal) {
			ret = can_calc_timing(dev, timing, timing_params->bus_speed,
					      timing_params->sample_point);
		} else {
			ret = can_calc_timing_data(dev, timing, timing_params->bus_speed,
						   timing_params->sample_point);
		}
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

static inline int mcp25xxfd_init_con_reg(const struct device *dev)
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

static inline int mcp25xxfd_init_osc_reg(const struct device *dev)
{
	int ret;
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_osc *osc = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*osc));

	osc->b0.CLKODIV = dev_cfg->clko_div;
	ret = mcp25xxfd_write_reg(dev, MCP25XXFD_REG_OSC, sizeof(*osc));
	if (ret < 0) {
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

static inline int mcp25xxfd_init_iocon_reg(const struct device *dev)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	struct mcp25xxfd_iocon *iocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*iocon));

/*
 *         MCP2518FD Errata: DS80000789
 *         Writing Byte 2/3 of the IOCON register using single SPI write cleat LAT0 and LAT1.
 *         This has no effect in the current version since LAT0/1 are set to zero anyway.
 *         However, it needs to be properly handled if other values are needed. Errata suggests
 *         to do single byte writes instead.
 */

	iocon->TRIS0 = 1;
	iocon->TRIS1 = 1;
	iocon->PM0 = 1;
	iocon->PM1 = 1;
	iocon->SOF = dev_cfg->sof_on_clko ? 1 : 0;

	return  mcp25xxfd_write_reg(dev, MCP25XXFD_REG_IOCON, sizeof(*iocon));
}

static inline int mcp25xxfd_init_int_hw1_reg(const struct device *dev)
{
	struct mcp25xxfd_int_hw1 *int_enable = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*int_enable));

	int_enable->RXIE = 1;
	int_enable->MODIE = 1;
	int_enable->TEFIE = 1;
	int_enable->CERRIE = 1;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_INT + 2, sizeof(*int_enable));
}

static inline int mcp25xxfd_init_tef_fifo(const struct device *dev)
{
	struct mcp25xxfd_fifocon *fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));

	fifocon->b0.FNEIE = 1;
	fifocon->b1.FRESET = 1;
	fifocon->b3.FSIZE = MCP25XXFD_TX_QUEUE_ITEMS - 1;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_TEFCON, sizeof(*fifocon));
}

static inline int mcp25xxfd_init_tx_queue(const struct device *dev)
{
	struct mcp25xxfd_fifocon *fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));

	fifocon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*fifocon));
	fifocon->b0.TXEN = 1;
	fifocon->b1.FRESET = 1;
	/* only has affect if MCP25XXFD_REG_CON_B2.RTXAT = 1 */
	fifocon->b2.TXAT = MCP25XXFD_TXAT_UNLIMITED_RETRANSMIT;
	fifocon->b3.PLSIZE = can_bytes_to_dlc(MCP25XXFD_PAYLOAD_SIZE) - 8;
	fifocon->b3.FSIZE = MCP25XXFD_TX_QUEUE_ITEMS - 1;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_TXQCON, sizeof(*fifocon));
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

#if defined(CONFIG_CAN_RX_TIMESTAMP)
static int mcp25xxfd_init_tscon(const struct device *dev)
{
	struct mcp25xxfd_tscon *tscon = mcp25xxfd_get_spi_buf_ptr(dev, sizeof(*tscon));

	tscon->TBCEN = 1;
	tscon->TBCPRE = CONFIG_CAN_MCP25XXFD_RX_TIMESTAMP_PRESCALER - 1;

	return mcp25xxfd_write_reg(dev, MCP25XXFD_REG_TSCON, sizeof(*tscon));
}
#endif

static int mcp25xxfd_reset(const struct device *dev)
{
	const struct mcp25xxfd_config *dev_cfg = dev->config;
	uint16_t cmd = sys_cpu_to_be16(MCP25XXFD_SPI_INSTRUCTION_RESET);
	const struct spi_buf tx_buf = {.buf = &cmd, .len = sizeof(cmd),};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	int ret;

	/* device can only be reset when in configuration mode */
	ret = mcp25xxfd_set_mode_internal(dev, MCP25XXFD_OPMODE_CONFIGURATION);
	if (ret < 0) {
		return ret;
	}

	return spi_write_dt(&dev_cfg->bus, &tx);
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

	if (dev_cfg->clk_dev != NULL) {
		int ret;
		uint32_t clk_id = dev_cfg->clk_id;

		if (!device_is_ready(dev_cfg->clk_dev)) {
			LOG_ERR("Clock controller not ready");
			return -ENODEV;
		}

		ret = clock_control_on(dev_cfg->clk_dev, (clock_control_subsys_t)clk_id);
		if (ret < 0) {
			LOG_ERR("Failed to enable clock");
			return ret;
		}
	}

	k_sem_init(&dev_data->int_sem, 0, 1);
	k_sem_init(&dev_data->tx_sem, MCP25XXFD_TX_QUEUE_ITEMS, MCP25XXFD_TX_QUEUE_ITEMS);

	k_mutex_init(&dev_data->mutex);

	if (!spi_is_ready_dt(&dev_cfg->bus)) {
		LOG_ERR("SPI bus %s not ready", dev_cfg->bus.bus->name);
		return -ENODEV;
	}

	if (!device_is_ready(dev_cfg->int_gpio_dt.port)) {
		LOG_ERR("GPIO port not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INPUT) < 0) {
		LOG_ERR("Unable to configure GPIO pin");
		return -EINVAL;
	}

	gpio_init_callback(&(dev_data->int_gpio_cb), mcp25xxfd_int_gpio_callback,
			   BIT(dev_cfg->int_gpio_dt.pin));

	if (gpio_add_callback(dev_cfg->int_gpio_dt.port, &(dev_data->int_gpio_cb)) < 0) {
		return -EINVAL;
	}

	if (gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio_dt, GPIO_INT_LEVEL_ACTIVE) < 0) {
		return -EINVAL;
	}

	k_thread_create(&dev_data->int_thread, dev_data->int_thread_stack,
			CONFIG_CAN_MCP25XXFD_INT_THREAD_STACK_SIZE,
			(k_thread_entry_t)mcp25xxfd_int_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_CAN_MCP25XXFD_INT_THREAD_PRIO), 0, K_NO_WAIT);

	(void)k_thread_name_set(&dev_data->int_thread, "MCP25XXFD interrupt thread");

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = mcp25xxfd_reset(dev);
	if (ret < 0) {
		LOG_ERR("Failed to reset the device [%d]", ret);
		goto done;
	}

	dev_data->sjw = dev_cfg->timing_params.sjw;
	ret = mcp25xxfd_init_timing_struct(&timing, dev, &dev_cfg->timing_params, true);
	if (ret < 0) {
		LOG_ERR("Can't find timing for given param");
		goto done;
	}

#if defined(CONFIG_CAN_FD_MODE)
	dev_data->sjw_data = dev_cfg->timing_params_data.sjw;
	ret = mcp25xxfd_init_timing_struct(&timing_data, dev, &dev_cfg->timing_params_data, false);
	if (ret < 0) {
		LOG_ERR("Can't find timing for given param");
		goto done;
	}
#endif

	con = mcp25xxfd_read_reg_crc(dev, MCP25XXFD_REG_CON, sizeof(*con));
	if (!con) {
		ret = -EINVAL;
		goto done;
	} else if (con->b2.OPMOD != MCP25XXFD_OPMODE_CONFIGURATION) {
		LOG_ERR("Device did not reset into configuration mode [%d]", con->b2.OPMOD);
		ret = -EIO;
		goto done;
	}

	dev_data->current_mcp25xxfd_mode = MCP25XXFD_OPMODE_CONFIGURATION;

	ret = mcp25xxfd_init_con_reg(dev);
	if (ret < 0) {
		goto done;
	}

	ret = mcp25xxfd_init_osc_reg(dev);
	if (ret < 0) {
		goto done;
	}

	ret = mcp25xxfd_init_iocon_reg(dev);
	if (ret < 0) {
		goto done;
	}

	ret = mcp25xxfd_init_int_hw1_reg(dev);
	if (ret < 0) {
		goto done;
	}

	ret = mcp25xxfd_set_tdc(dev, false, 0);
	if (ret < 0) {
		goto done;
	}

#if defined(CONFIG_CAN_RX_TIMESTAMP)
	ret = mcp25xxfd_init_tscon(dev);
	if (ret < 0) {
		goto done;
	}
#endif

	ret = mcp25xxfd_init_tef_fifo(dev);
	if (ret < 0) {
		goto done;
	}

	ret = mcp25xxfd_init_tx_queue(dev);
	if (ret < 0) {
		goto done;
	}

	ret = mcp25xxfd_init_rx_fifo(dev);
	if (ret < 0) {
		goto done;
	}

	LOG_DBG("%d TX FIFOS: 1 element", MCP25XXFD_TX_QUEUE_ITEMS);
	LOG_DBG("1 RX FIFO: %d elements", MCP25XXFD_RX_FIFO_ITEMS);
	LOG_DBG("%db of %db RAM Allocated",
		MCP25XXFD_TEF_FIFO_SIZE + MCP25XXFD_TX_QUEUE_SIZE + MCP25XXFD_RX_FIFO_SIZE,
		MCP25XXFD_RAM_SIZE);

done:
	k_mutex_unlock(&dev_data->mutex);
	if (ret < 0) {
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