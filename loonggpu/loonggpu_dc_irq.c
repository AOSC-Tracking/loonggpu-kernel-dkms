#include <drm/drm_vblank.h>
#include "loonggpu.h"
#include "loonggpu_dc_crtc.h"
#include "loonggpu_dc_resource.h"
#include "loonggpu_dc_irq.h"
#include "loonggpu_dc_reg.h"
#include "loonggpu_dc_i2c.h"

#define DM_IRQ_TABLE_LOCK(adev, flags) \
	spin_lock_irqsave(&adev->dc->irq_handler_list_table_lock, flags)

#define DM_IRQ_TABLE_UNLOCK(adev, flags) \
	spin_unlock_irqrestore(&adev->dc->irq_handler_list_table_lock, flags)

static void dc_handle_hpd_irq(void *param)
{
	struct loonggpu_connector *aconnector = (struct loonggpu_connector *)param;
	struct drm_connector *connector = &aconnector->base;
	struct drm_device *dev = connector->dev;
	struct loonggpu_device *adev = dev->dev_private;
	enum drm_connector_status old_status;

	mutex_lock(&aconnector->hpd_lock);
	old_status = connector->status;
	connector->status = drm_helper_probe_detect(connector, NULL, false);

	if ((adev->vga_hpd_status == connector_status_connected)
	    && (connector->index == 0))
		connector->connector_type = DRM_MODE_CONNECTOR_VGA;
	else
		connector->connector_type = DRM_MODE_CONNECTOR_HDMIA;

	if (old_status != connector->status)
		drm_kms_helper_hotplug_event(dev);

	mutex_unlock(&aconnector->hpd_lock);
}

static void dc_handle_i2c_irq(void *param)
{
	struct loonggpu_connector *aconnector = (struct loonggpu_connector *)param;
	struct drm_connector *connector = &aconnector->base;
	struct loonggpu_device *adev = connector->dev->dev_private;
	struct loonggpu_dc_i2c *i2c = adev->i2c[connector->index];

	loonggpu_dc_i2c_irq(i2c);
}

static void dc_handle_vsync_irq(void *interrupt_params)
{
	struct loonggpu_crtc *loonggpu_crtc = interrupt_params;
	struct drm_device *dev = loonggpu_crtc->base.dev;
	unsigned long flags;

	if (loonggpu_crtc == NULL) {
		DRM_DEBUG_DRIVER("CRTC is null, returning.\n");
		return;
	}

	drm_handle_vblank(dev, loonggpu_crtc->crtc_id);
	spin_lock_irqsave(&dev->event_lock, flags);

	if (loonggpu_crtc->pflip_status != LOONGGPU_FLIP_SUBMITTED) {
		DRM_DEBUG_DRIVER("loonggpu_crtc->pflip_status = %d !=LOONGGPU_FLIP_SUBMITTED(%d) on crtc:%d[%p] \n",
						 loonggpu_crtc->pflip_status,
						 LOONGGPU_FLIP_SUBMITTED,
						 loonggpu_crtc->crtc_id,
						 loonggpu_crtc);
		spin_unlock_irqrestore(&dev->event_lock, flags);
		return;
	}

	/* wakeup usersapce */
	if (loonggpu_crtc->event) {
		/* Update to correct count/ts if racing with vblank irq */
		drm_crtc_accurate_vblank_count(&loonggpu_crtc->base);
		drm_crtc_send_vblank_event(&loonggpu_crtc->base, loonggpu_crtc->event);
		/* page flip completed. clean up */
		loonggpu_crtc->event = NULL;

	} else
		WARN_ON(1);

	loonggpu_crtc->pflip_status = LOONGGPU_FLIP_NONE;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	DRM_DEBUG_DRIVER("%s - crtc :%d[%p], pflip_stat:LOONGGPU_FLIP_NONE\n",
					__func__, loonggpu_crtc->crtc_id, loonggpu_crtc);

	drm_crtc_vblank_put(&loonggpu_crtc->base);
}

static bool dc_i2c_int_enable(struct loonggpu_dc_crtc *crtc, bool enable)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	u32 link;
	u32 value;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	value = dc_readl(adev, gdc_reg->global_reg.intr);
	switch (link) {
	case 0:
		if (enable)
			value |= DC_INT_I2C0_EN;
		else {
			value &= ~DC_INT_I2C0_EN;
		}
		break;
	case 1:
		if (enable)
			value |= DC_INT_I2C1_EN;
		else
			value &= ~DC_INT_I2C1_EN;
		break;
	default:
		return false;
	}
	dc_writel(adev, gdc_reg->global_reg.intr, value);

	return true;
}

static bool dc_hpd_enable(struct loonggpu_dc_crtc *crtc, bool enable)
{
	struct loonggpu_device *adev = crtc->dc->adev;
	struct loonggpu_connector *lconnector;
	u32 link;
	u32 int_reg;
	u32 vga_cfg;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;
	lconnector = adev->mode_info.connectors[link];

	int_reg = dc_readl(adev, gdc_reg->global_reg.intr);
	vga_cfg = dc_readl(adev, gdc_reg->global_reg.vga_hp_cfg);

	switch (link) {
	case 0:
		if (enable) {
			if (lconnector->base.polled == DRM_CONNECTOR_POLL_HPD) {
				int_reg |= DC_INT_VGA_HOTPLUG_EN;
				vga_cfg &= ~0x3;
				vga_cfg |= 0x1;
			} else {
				int_reg &= ~DC_INT_VGA_HOTPLUG_EN;
				vga_cfg = 0;
			}
			if (lconnector->base.polled)
				int_reg |= DC_INT_HDMI0_HOTPLUG_EN;
			else
				int_reg &= ~DC_INT_HDMI0_HOTPLUG_EN;
		} else {
			vga_cfg &= ~0x3;
			int_reg &= ~DC_INT_HDMI0_HOTPLUG_EN;
			int_reg &= ~DC_INT_VGA_HOTPLUG_EN;
		}
		break;
	case 1:
		if (enable && lconnector->base.polled)
			int_reg |= DC_INT_HDMI1_HOTPLUG_EN;
		else
			int_reg &= ~DC_INT_HDMI1_HOTPLUG_EN;
		break;
	default:
		return false;
	}

	dc_writel(adev, gdc_reg->global_reg.intr, int_reg);
	dc_writel(adev, gdc_reg->global_reg.vga_hp_cfg, vga_cfg);

	return true;
}

static bool dc_crtc_vblank_enable(struct loonggpu_dc_crtc *crtc, bool enable)
{
	u32 link;
	u32 value;
	struct loonggpu_device *adev = crtc->dc->adev;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	value = dc_readl(adev, gdc_reg->global_reg.intr);
	switch (link) {
	case 0:
		if (enable)
			value |= INT_VSYNC0_ENABLE;
		else
			value &= ~INT_VSYNC0_ENABLE;
		break;
	case 1:
		if (enable)
			value |= INT_VSYNC1_ENABLE;
		else
			value &= ~INT_VSYNC1_ENABLE;
		break;
	default:
		return false;
	}

	dc_writel(adev, gdc_reg->global_reg.intr, value);

	return true;
}

static bool
validate_irq_registration_params(struct dc_interrupt_params *int_params,
				 void (*ih)(void *))
{
	if (NULL == int_params || NULL == ih) {
		DRM_ERROR("DC_IRQ: invalid input!\n");
		return false;
	}

	if (int_params->int_context >= INTERRUPT_CONTEXT_NUMBER) {
		DRM_ERROR("DC_IRQ: invalid context: %d!\n",
				int_params->int_context);
		return false;
	}

	if (!DC_VALID_IRQ_SRC_NUM(int_params->irq_source)) {
		DRM_ERROR("DC_IRQ: invalid irq_source: %d!\n",
				int_params->irq_source);
		return false;
	}

	return true;
}

static void *dc_irq_register_interrupt(struct loonggpu_device *adev,
				       struct dc_interrupt_params *int_params,
				       void (*ih)(void *),
				       void *handler_args)
{
	struct list_head *hnd_list;
	struct dc_irq_handler_data *handler_data;
	unsigned long irq_table_flags;
	enum dc_irq_source irq_source;

	if (false == validate_irq_registration_params(int_params, ih))
		return DAL_INVALID_IRQ_HANDLER_IDX;

	handler_data = kzalloc(sizeof(*handler_data), GFP_KERNEL);
	if (!handler_data) {
		DRM_ERROR("DC_IRQ: failed to allocate irq handler!\n");
		return DAL_INVALID_IRQ_HANDLER_IDX;
	}

	memset(handler_data, 0, sizeof(*handler_data));

	handler_data->handler = ih;
	handler_data->handler_arg = handler_args;

	irq_source = int_params->irq_source;

	handler_data->irq_source = irq_source;

	DM_IRQ_TABLE_LOCK(adev, irq_table_flags);

	switch (int_params->int_context) {
	case INTERRUPT_HIGH_IRQ_CONTEXT:
		hnd_list = &adev->dc->irq_handler_list_high_tab[irq_source];
		break;
	case INTERRUPT_LOW_IRQ_CONTEXT:
	default:
		hnd_list = &adev->dc->irq_handler_list_low_tab[irq_source].head;
		break;
	}

	list_add_tail(&handler_data->list, hnd_list);

	DM_IRQ_TABLE_UNLOCK(adev, irq_table_flags);

	DRM_DEBUG_KMS(
		"DC_IRQ: added irq handler: %p for: irq_src=%d, irq context=%d\n",
		handler_data,
		irq_source,
		int_params->int_context);

	return handler_data;
}

static void dc_register_vsync_handlers(struct loonggpu_device *adev)
{
	struct drm_device *dev = adev->ddev;
	struct drm_crtc *crtc;
	struct loonggpu_crtc *acrtc;
	struct dc_interrupt_params int_params = {0};

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		acrtc = to_loonggpu_crtc(crtc);
		int_params.int_context = INTERRUPT_HIGH_IRQ_CONTEXT;

		if (DC_IRQ_SOURCE_INVALID != acrtc->irq_source_vsync) {
			int_params.irq_source = acrtc->irq_source_vsync;
			dc_irq_register_interrupt(adev, &int_params,
					dc_handle_vsync_irq, (void *) acrtc);
		}
	}
}

static void dc_register_i2c_handlers(struct loonggpu_device *adev)
{
	struct drm_device *dev = adev->ddev;
	struct drm_connector *connector;
	struct loonggpu_connector *aconnector;
	struct dc_interrupt_params int_params = {0};

	list_for_each_entry(connector, &dev->mode_config.connector_list, head)	{
		aconnector = to_loonggpu_connector(connector);
		int_params.int_context = INTERRUPT_HIGH_IRQ_CONTEXT;

		if (DC_IRQ_SOURCE_INVALID != aconnector->irq_source_i2c) {
			int_params.irq_source = aconnector->irq_source_i2c;
			dc_irq_register_interrupt(adev, &int_params,
					dc_handle_i2c_irq, (void *) aconnector);
		}
	}
}

static void dc_register_hpd_handlers(struct loonggpu_device *adev)
{
	struct drm_device *dev = adev->ddev;
	struct drm_connector *connector;
	struct loonggpu_connector *aconnector;
	struct dc_interrupt_params int_params = {0};

	list_for_each_entry(connector, &dev->mode_config.connector_list, head)	{
		aconnector = to_loonggpu_connector(connector);
		int_params.int_context = INTERRUPT_LOW_IRQ_CONTEXT;

		if (DC_IRQ_SOURCE_INVALID != aconnector->irq_source_hpd) {
			int_params.irq_source = aconnector->irq_source_hpd;
			dc_irq_register_interrupt(adev, &int_params,
					dc_handle_hpd_irq,
					(void *) aconnector);
		}

		if (DC_IRQ_SOURCE_INVALID != aconnector->irq_source_vga_hpd
		    && connector->index == 0) {
			int_params.irq_source = aconnector->irq_source_vga_hpd;
			dc_irq_register_interrupt(adev, &int_params,
					dc_handle_hpd_irq,
					(void *) aconnector);
		}
	}
}

static bool validate_irq_unregistration_params(enum dc_irq_source irq_source,
					       irq_handler_idx handler_idx)
{
	if (DAL_INVALID_IRQ_HANDLER_IDX == handler_idx) {
		DRM_ERROR("DC_IRQ: invalid handler_idx==NULL!\n");
		return false;
	}

	if (!DC_VALID_IRQ_SRC_NUM(irq_source)) {
		DRM_ERROR("DC_IRQ: invalid irq_source:%d!\n", irq_source);
		return false;
	}

	return true;
}

static struct list_head *remove_irq_handler(struct loonggpu_device *adev,
					    void *ih,
					    const struct dc_interrupt_params *int_params)
{
	struct list_head *hnd_list;
	struct list_head *entry, *tmp;
	struct dc_irq_handler_data *handler;
	unsigned long irq_table_flags;
	bool handler_removed = false;
	enum dc_irq_source irq_source;

	DM_IRQ_TABLE_LOCK(adev, irq_table_flags);

	irq_source = int_params->irq_source;

	switch (int_params->int_context) {
	case INTERRUPT_HIGH_IRQ_CONTEXT:
		hnd_list = &adev->dc->irq_handler_list_high_tab[irq_source];
		break;
	case INTERRUPT_LOW_IRQ_CONTEXT:
	default:
		hnd_list = &adev->dc->irq_handler_list_low_tab[irq_source].head;
		break;
	}

	list_for_each_safe(entry, tmp, hnd_list) {

		handler = list_entry(entry, struct dc_irq_handler_data, list);

		if (ih == handler) {
			/* Found our handler. Remove it from the list. */
			list_del(&handler->list);
			handler_removed = true;
			break;
		}
	}

	DM_IRQ_TABLE_UNLOCK(adev, irq_table_flags);

	if (handler_removed == false) {
		/* Not necessarily an error - caller may not
		 * know the context. */
		return NULL;
	}

	kfree(handler);

	DRM_DEBUG_KMS(
	"DC_IRQ: removed irq handler: %p for: irq_src=%d, irq context=%d\n",
		ih, int_params->irq_source, int_params->int_context);

	return hnd_list;
}

static void dc_irq_work_func(struct work_struct *work)
{
	struct list_head *entry;
	struct irq_list_head *irq_list_head =
		container_of(work, struct irq_list_head, work);
	struct list_head *handler_list = &irq_list_head->head;
	struct dc_irq_handler_data *handler_data;

	list_for_each(entry, handler_list) {
		handler_data = list_entry(entry,
				struct dc_irq_handler_data, list);

		DRM_DEBUG_KMS("DC_IRQ: work_func: for irq_src=%d\n",
				handler_data->irq_source);

		DRM_DEBUG_KMS("DC_IRQ: schedule_work: for irq_src=%d\n",
			handler_data->irq_source);

		handler_data->handler(handler_data->handler_arg);
	}
}

static void dc_irq_schedule_work(struct loonggpu_device *adev,
					enum dc_irq_source irq_source)
{
	unsigned long irq_table_flags;
	struct work_struct *work = NULL;

	DM_IRQ_TABLE_LOCK(adev, irq_table_flags);

	if (!list_empty(&adev->dc->irq_handler_list_low_tab[irq_source].head))
		work = &adev->dc->irq_handler_list_low_tab[irq_source].work;

	DM_IRQ_TABLE_UNLOCK(adev, irq_table_flags);

	if (work) {
		if (!schedule_work(work))
			DRM_INFO("Irq schedule work FAILED src %d\n", irq_source);
	}

}

static void dc_irq_immediate_work(struct loonggpu_device *adev,
					 enum dc_irq_source irq_source)
{
	struct dc_irq_handler_data *handler_data;
	struct list_head *entry;
	unsigned long irq_table_flags;

	DM_IRQ_TABLE_LOCK(adev, irq_table_flags);

	list_for_each(entry, &adev->dc->irq_handler_list_high_tab[irq_source]) {
		handler_data = list_entry(entry,
				struct dc_irq_handler_data, list);

		/* Call a subcomponent which registered for immediate
		 * interrupt notification */
		handler_data->handler(handler_data->handler_arg);
	}

	DM_IRQ_TABLE_UNLOCK(adev, irq_table_flags);
}

static enum dc_irq_source dc_interrupt_to_irq_source(u32 src_id)
{
	enum dc_irq_source dc_irq_source;

	switch (src_id) {
	case DC_INT_ID_VSYNC1:
		dc_irq_source = DC_IRQ_SOURCE_VSYNC1;
		break;
	case DC_INT_ID_VSYNC0:
		dc_irq_source = DC_IRQ_SOURCE_VSYNC0;
		break;
	case DC_INT_ID_I2C0:
		dc_irq_source = DC_IRQ_SOURCE_I2C0;
		break;
	case DC_INT_ID_I2C1:
		dc_irq_source = DC_IRQ_SOURCE_I2C1;
		break;
	case DC_INT_ID_HPD_HDMI0:
		dc_irq_source = DC_IRQ_SOURCE_HPD_HDMI0;
		break;
	case DC_INT_ID_HPD_HDMI1:
		dc_irq_source = DC_IRQ_SOURCE_HPD_HDMI1;
		break;
	case DC_INT_ID_HPD_VGA:
		dc_irq_source = DC_IRQ_SOURCE_HPD_VGA;
		break;
	default:
		DRM_ERROR("NO support this irq id:%d!\n", src_id);
		break;
	}

	return dc_irq_source;
}

static bool dc_hpd_ack(struct loonggpu_dc_crtc *crtc)
{
	u32 link;
	u32 value;
	u32 vga_reg;
	struct loonggpu_device *adev = crtc->dc->adev;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	value = dc_readl(adev, gdc_reg->global_reg.intr);

	if (value & (1 << 15)) {
		vga_reg = dc_readl(adev, gdc_reg->global_reg.vga_hp_cfg);
		if ((vga_reg & DC_VGA_HPD_STATUS_MASK) == 1) {
			adev->vga_hpd_status = connector_status_connected;
			vga_reg &= ~0x3;
			vga_reg |= 0x2;
		} else if ((vga_reg & DC_VGA_HPD_STATUS_MASK) == 2) {
			adev->vga_hpd_status = connector_status_disconnected;
			vga_reg &= ~0x3;
			vga_reg |= 0x1;
		} else {
			adev->vga_hpd_status = connector_status_disconnected;
			dc_writel(adev, gdc_reg->global_reg.vga_hp_cfg, 0x0);
			DRM_ERROR("Error VGA HPD status\n");
			return false;
		}
		dc_writel(adev, gdc_reg->global_reg.vga_hp_cfg, vga_reg);
	}

	switch (link) {
	case 0:
		value |= (1 << 13);
		value |= (1 << 15);
		break;
	case 1:
		value |= (1 << 14);
		break;
	}

	dc_writel(adev, gdc_reg->global_reg.intr, value);

	return true;
}

static bool dc_i2c_ack(struct loonggpu_dc_crtc *crtc)
{
	u32 link;
	u32 value;
	struct loonggpu_device *adev = crtc->dc->adev;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	value = dc_readl(adev, gdc_reg->global_reg.intr);

	switch (link) {
	case 0:
		value |= (1 << 11);
		break;
	case 1:
		value |= (1 << 12);
		break;
	}

	dc_writel(adev, gdc_reg->global_reg.intr, value);

	return true;
}

static bool dc_crtc_vblank_ack(struct loonggpu_dc_crtc *crtc)
{
	u32 link;
	u32 value;
	struct loonggpu_device *adev = crtc->dc->adev;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	value = dc_readl(adev, gdc_reg->global_reg.intr);

	switch (link) {
	case 0:
		value |= INT_VSYNC0_ENABLE;
		break;
	case 1:
		value |= INT_VSYNC1_ENABLE;
		break;
	}

	dc_writel(adev, gdc_reg->global_reg.intr, value);

	return true;
}

static bool dc_submit_interrupt_ack(struct loonggpu_dc *dc, enum dc_irq_source src)
{
	bool ret = false;

	if (IS_ERR_OR_NULL(dc))
		return false;

	switch (src) {
	case DC_IRQ_SOURCE_VSYNC0:
		if (dc->link_info)
			ret = dc_crtc_vblank_ack(dc->link_info[0].crtc);
		break;
	case DC_IRQ_SOURCE_VSYNC1:
		if (dc->link_info)
			ret = dc_crtc_vblank_ack(dc->link_info[1].crtc);
		break;
	case DC_IRQ_SOURCE_I2C0:
		if (dc->link_info)
			ret = dc_i2c_ack(dc->link_info[0].crtc);
		break;
	case DC_IRQ_SOURCE_I2C1:
		if (dc->link_info)
			ret = dc_i2c_ack(dc->link_info[1].crtc);
		break;
	case DC_IRQ_SOURCE_HPD_HDMI0:
	case DC_IRQ_SOURCE_HPD_HDMI1:
		if (dc->link_info)
			ret = dc_hpd_ack(dc->link_info[0].crtc);
		break;
	case DC_IRQ_SOURCE_HPD_VGA:
		if (dc->link_info)
			ret = dc_hpd_ack(dc->link_info[1].crtc);
		break;
	default:
		DRM_ERROR("%s Can not support this irq %d \n", __func__, src);
		ret = false;
		break;
	}

	return ret;
}

static int dc_irq_handler(struct loonggpu_device *adev,
				 struct loonggpu_irq_src *source,
				 struct loonggpu_iv_entry *entry)
{
	enum dc_irq_source src =
		dc_interrupt_to_irq_source(entry->src_id);

	dc_submit_interrupt_ack(adev->dc, src);

	/* Call high irq work immediately */
	dc_irq_immediate_work(adev, src);

	/*Schedule low_irq work */
	dc_irq_schedule_work(adev, src);

	return 0;
}

static bool dc_interrupt_enable(struct loonggpu_dc *dc, enum dc_irq_source src, bool enable)
{
	bool ret = false;

	if (IS_ERR_OR_NULL(dc))
		return false;

	switch (src) {
	case DC_IRQ_SOURCE_VSYNC0:
		if (dc->link_info)
			ret = dc_crtc_vblank_enable(dc->link_info[0].crtc, enable);
		break;
	case DC_IRQ_SOURCE_VSYNC1:
		if (dc->link_info)
			ret = dc_crtc_vblank_enable(dc->link_info[1].crtc, enable);
		break;
	case DC_IRQ_SOURCE_I2C0:
		if (dc->link_info)
			ret = dc_i2c_int_enable(dc->link_info[0].crtc, enable);
		break;
	case DC_IRQ_SOURCE_I2C1:
		if (dc->link_info)
			ret = dc_i2c_int_enable(dc->link_info[1].crtc, enable);
		break;
	case DC_IRQ_SOURCE_HPD_HDMI0:
	case DC_IRQ_SOURCE_HPD_VGA:
		if (dc->link_info)
			ret = dc_hpd_enable(dc->link_info[0].crtc, enable);
		break;
	case DC_IRQ_SOURCE_HPD_HDMI1:
		if (dc->link_info)
			ret = dc_hpd_enable(dc->link_info[1].crtc, enable);
		break;
	case DC_IRQ_SOURCE_HPD_HDMI0_NULL:
		if (dc->link_info)
			ret = dc_hpd_enable(dc->link_info[0].crtc, false);
		break;
	case DC_IRQ_SOURCE_HPD_HDMI1_NULL:
		if (dc->link_info)
			ret = dc_hpd_enable(dc->link_info[1].crtc, false);
		break;
	case DC_IRQ_SOURCE_INVALID:
		break;
	default:
		DRM_ERROR("%s Can not support this irq %d \n", __func__, src);
		break;
	}

	return ret;
}

static inline int dc_irq_state(struct loonggpu_device *adev,
			       struct loonggpu_irq_src *source,
			       unsigned crtc_id,
			       enum loonggpu_interrupt_state state,
			       const enum irq_type dc_irq_type,
			       const char *func)
{
	struct loonggpu_crtc *acrtc = adev->mode_info.crtcs[crtc_id];
	enum dc_irq_source irq_source;
	bool st;

	if (!acrtc) {
		DRM_ERROR("%s: crtc is NULL at id :%d\n", func, crtc_id);
		return 0;
	}

	if (acrtc->crtc_id == -1)
		return 0;

	irq_source = dc_irq_type + crtc_id;
	st = (state == LOONGGPU_IRQ_STATE_ENABLE);

	DRM_DEBUG_DRIVER("dc irq state crtc_id:%d, irq_src:%d, st:%d\n",
			 crtc_id, irq_source, st);
	dc_interrupt_enable(adev->dc, irq_source, st);

	return 0;
}

static int dc_set_pflip_irq_state(struct loonggpu_device *adev,
				  struct loonggpu_irq_src *source,
				  unsigned crtc_id,
				  enum loonggpu_interrupt_state state)
{
	return dc_irq_state(adev, source, crtc_id, state, DC_IRQ_TYPE_VSYNC,
			    __func__);
}

static int dc_set_i2c_irq_state(struct loonggpu_device *adev,
				struct loonggpu_irq_src *source,
				unsigned crtc_id,
				enum loonggpu_interrupt_state state)
{
	return dc_irq_state(adev, source, crtc_id, state, DC_IRQ_TYPE_I2C,
			    __func__);
}

static int dc_set_hpd_irq_state(struct loonggpu_device *adev,
				struct loonggpu_irq_src *source,
				unsigned crtc_id,
				enum loonggpu_interrupt_state state)
{
	return dc_irq_state(adev, source, crtc_id, state, DC_IRQ_TYPE_HPD,
			    __func__);
}

static const struct loonggpu_irq_src_funcs dc_vsync_irq_funcs = {
	.set = dc_set_pflip_irq_state,
	.process = dc_irq_handler,
};

static const struct loonggpu_irq_src_funcs dc_i2c_irq_funcs = {
	.set = dc_set_i2c_irq_state,
	.process = dc_irq_handler,
};

static const struct loonggpu_irq_src_funcs dc_hpd_irq_funcs = {
	.set = dc_set_hpd_irq_state,
	.process = dc_irq_handler,
};

static void dc_set_irq_funcs(struct loonggpu_device *adev)
{
	adev->vsync_irq.num_types = adev->mode_info.num_crtc;
	adev->vsync_irq.funcs = &dc_vsync_irq_funcs;

	adev->i2c_irq.num_types = adev->mode_info.num_i2c;
	adev->i2c_irq.funcs = &dc_i2c_irq_funcs;

	adev->hpd_irq.num_types = adev->mode_info.num_hpd;
	adev->hpd_irq.funcs = &dc_hpd_irq_funcs;
}

static void dc_hpd_init(struct loonggpu_device *adev)
{
	struct drm_device *dev = adev->ddev;
	struct drm_connector *connector;
	struct loonggpu_connector *aconnector;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		aconnector = to_loonggpu_connector(connector);
		dc_interrupt_enable(adev->dc, aconnector->irq_source_hpd, true);
	}

	adev->vga_hpd_status = connector_status_unknown;

	return;
}

static void dc_hpd_disable(struct loonggpu_device *adev)
{
	struct drm_device *dev = adev->ddev;
	struct drm_connector *connector;
	struct loonggpu_connector *aconnector;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		aconnector = to_loonggpu_connector(connector);
		dc_interrupt_enable(adev->dc, aconnector->irq_source_hpd, false);
	}
	adev->vga_hpd_status = connector_status_unknown;

	return;
}

static int dc_register_irq_handlers(struct loonggpu_device *adev)
{
	int r;
	int i;
	unsigned client_id = SOC15_IH_CLIENTID_DCE;

	/* vsync */
	for (i = DC_INT_ID_VSYNC1; i <= DC_INT_ID_VSYNC0; i += 2) {
		r = loonggpu_irq_add_id(adev, client_id, i, &adev->vsync_irq);
		if (r) {
			DRM_ERROR("Failed to add page flip irq id!\n");
			return r;
		}
	}
	dc_register_vsync_handlers(adev);

	/* I2C */
	for (i = DC_INT_ID_I2C0; i <= DC_INT_ID_I2C1; i++) {
		r = loonggpu_irq_add_id(adev, client_id, i, &adev->i2c_irq);
		if (r) {
			DRM_ERROR("Failed to add page flip irq id!\n");
			return r;
		}
	}
	dc_register_i2c_handlers(adev);

	/* HPD */
	for (i = DC_INT_ID_HPD_HDMI0; i <= DC_INT_ID_HPD_VGA; i++) {
		r = loonggpu_irq_add_id(adev, client_id, i, &adev->hpd_irq);
		if (r) {
			DRM_ERROR("Failed to add hpd irq id %d!\n", i);
			return r;
		}
	}
	dc_register_hpd_handlers(adev);

	return 0;
}

static irqreturn_t loonggpu_dc_irq_handler(int irq, void *arg)
{
	u32 int_reg;
	unsigned long base;
	struct loonggpu_iv_entry entry;
	struct loonggpu_device *adev = (struct loonggpu_device *)arg;
	int i = 1;

	base = (unsigned long)(adev->loongson_dc_rmmio_base);

	int_reg = dc_readl(adev, gdc_reg->global_reg.intr);
	dc_writel(adev, gdc_reg->global_reg.intr, int_reg);
	entry.client_id = SOC15_IH_CLIENTID_DCE;

	int_reg &= 0xffff;
	while (int_reg && (i < DC_INT_ID_MAX)) {
		if (!(int_reg & 0x1)) {
			int_reg = int_reg >> 1;
			i++;
			continue;
		}

		entry.src_id = i;
		loonggpu_irq_dispatch(adev, &entry);

		int_reg = int_reg >> 1;
		i++;
	}

	return IRQ_HANDLED;
}

static int loonggpu_dc_irq_init(struct loonggpu_device *adev)
{
	int r;
	struct pci_dev *loongson_dc = adev->loongson_dc;

	adev->ddev->max_vblank_count = 0x00ffffff;
	if (loongson_dc) {
		u32 dc_irq = loongson_dc->irq;
		r = request_irq(dc_irq, loonggpu_dc_irq_handler, 0,
				loongson_dc->driver->name, adev);
		if (r) {
			DRM_ERROR("loonggpu register dc irq failed\n");
			return r;
		}
	} else {
		DRM_ERROR("loonggpu dc device is null\n");
		return ENODEV;
	}

	return 0;
}

int dc_irq_sw_init(struct loonggpu_device *adev)
{
	int src;
	struct irq_list_head *lh;
	int r;

	spin_lock_init(&adev->dc->irq_handler_list_table_lock);
	loonggpu_dc_irq_init(adev);
	for (src = 0; src < DC_IRQ_SOURCES_NUMBER; src++) {
		/* low context handler list init */
		lh = &adev->dc->irq_handler_list_low_tab[src];
		INIT_LIST_HEAD(&lh->head);
		INIT_WORK(&lh->work, dc_irq_work_func);

		/* high context handler init */
		INIT_LIST_HEAD(&adev->dc->irq_handler_list_high_tab[src]);
	}

	dc_set_irq_funcs(adev);

	r = dc_register_irq_handlers(adev);
	if (r) {
		DRM_ERROR("DC: Failed to initialize IRQ\n");
		return r;
	}
	DRM_INFO("LOONGGPU DC irq init sources number:%d\n", src - 1);

	return 0;
}

int dc_irq_hw_init(struct loonggpu_device *adev)
{
	dc_hpd_init(adev);
	return 0;
}

int dc_irq_hw_uninit(struct loonggpu_device *adev)
{
	dc_hpd_disable(adev);
	return 0;
}

void dc_irq_fini(struct loonggpu_device *adev)
{
	int src;
	struct irq_list_head *lh;
	unsigned long irq_table_flags;

	for (src = 0; src < DC_IRQ_SOURCES_NUMBER; src++) {
		DM_IRQ_TABLE_LOCK(adev, irq_table_flags);
		/* The handler was removed from the table,
		 * it means it is safe to flush all the 'work'
		 * (because no code can schedule a new one). */
		lh = &adev->dc->irq_handler_list_low_tab[src];
		DM_IRQ_TABLE_UNLOCK(adev, irq_table_flags);
		flush_work(&lh->work);
	}
}
