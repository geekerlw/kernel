/*
 * Rockchip vga driver
 *
 * Copyright (C) 2017 Steven Lee <geekerlw@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/component.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mfd/syscon.h>
#include <linux/regulator/consumer.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>

#include "rockchip_drm_drv.h"

#define connector_to_vga(c) container_of(c, struct rockchip_vga, connector)
#define encoder_to_vga(c) container_of(c, struct rockchip_vga, encoder)

struct rockchip_vga {
	struct device *dev;
	struct gpio_desc *gpio;
	struct i2c_adapter *ddc;
	struct regulator *reg;

	struct drm_device *drm_dev;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_encoder encoder;

	struct mutex suspend_lock;
	int suspend;
};

static int rockchip_vga_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_vga *vga = connector_to_vga(connector);
	struct edid *edid;
	int ret = 0;

	if (!vga->ddc)
		return 0;

	edid = drm_get_edid(connector, vga->ddc);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	return ret;
}

static struct drm_encoder *rockchip_vga_connector_best_encoder(struct drm_connector *connector)
{
	struct rockchip_vga *vga = connector_to_vga(connector);

	return &vga->encoder;
}


static enum drm_mode_status rockchip_vga_connector_mode_valid(struct drm_connector *connector,
					struct drm_display_mode *mode)
{
	return MODE_OK;
}

static const struct drm_connector_helper_funcs rockchip_vga_connector_helper_funcs = {
	.get_modes = rockchip_vga_connector_get_modes,
	.best_encoder = rockchip_vga_connector_best_encoder,
	.mode_valid = rockchip_vga_connector_mode_valid,
};

static enum drm_connector_status rockchip_vga_connector_detect(struct drm_connector *connector,
					    bool force)
{
	struct rockchip_vga *vga = connector_to_vga(connector);

	if (!vga->ddc)
		return connector_status_unknown;

	if (drm_probe_ddc(vga->ddc))
		return connector_status_connected;

	return connector_status_disconnected;
}

void rockchip_vga_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs rockchip_vga_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = rockchip_vga_connector_detect,
	.destroy = rockchip_vga_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void rockchip_vga_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct rockchip_vga *vga = encoder_to_vga(encoder);
	int ret;

	mutex_lock(&vga->suspend_lock);

	switch(mode) {
	case DRM_MODE_DPMS_ON:
		if(!vga->suspend)
			goto out;

		if(!IS_ERR(vga->reg)) {
			ret = regulator_enable(vga->reg);
			if(ret < 0) {
				pr_debug("vga encoder regulator fail\n");
			}
		}
		if(vga->gpio)
			gpiod_direction_output(vga->gpio, 1);

		vga->suspend= false;
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (vga->suspend)
			goto out;

		if (vga->gpio)
            gpiod_direction_output(vga->gpio, 0);

		if (!IS_ERR(vga->reg)) {
			ret = regulator_enable(vga->reg);
            if(ret < 0) {
                pr_debug("vga encoder regulator fail\n");
            }
		}

		vga->suspend = true;
		break;
	default:
		break;
	}

out:
	mutex_unlock(&vga->suspend_lock);
}

static bool rockchip_vga_encoder_mode_fixup(struct drm_encoder *encoder,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void rockchip_vga_encoder_prepare(struct drm_encoder *encoder)
{
	dump_stack();
}

static void rockchip_vga_encoder_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
}

static void rockchip_vga_encoder_commit(struct drm_encoder *encoder)
{
	rockchip_vga_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void rockchip_vga_encoder_disable(struct drm_encoder *encoder)
{
	rockchip_vga_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static int rockchip_vga_encoder_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct rockchip_crtc_state *s = to_rockchip_crtc_state(crtc_state);

	s->output_mode = 0;
	s->output_type = DRM_MODE_CONNECTOR_VGA;

	return 0;
}

static const struct drm_encoder_helper_funcs rockchip_vga_encoder_helper_funcs = {
	.dpms = rockchip_vga_encoder_dpms,
	.mode_fixup = rockchip_vga_encoder_mode_fixup,
	.prepare = rockchip_vga_encoder_prepare,
	.mode_set = rockchip_vga_encoder_mode_set,
	.commit = rockchip_vga_encoder_commit,
	.disable = rockchip_vga_encoder_disable,
	.atomic_check = rockchip_vga_encoder_atomic_check,
};


static void rockchip_vga_encoder_destory(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs rockchip_vga_encoder_funcs = {
	.destroy = rockchip_vga_encoder_destory,
};
	
static const struct of_device_id rockchip_vga_dt_ids[] = {
	{
		.compatible = "adi,adv7123",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rockchip_vga_dt_ids);

static int rockchip_vga_bind(struct device *dev, struct device *master, void *data)
{
	struct rockchip_vga *vga = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	struct device_node *port, *output_node = NULL;
	int ret;

	vga->drm_dev = drm_dev;

	/* start to prepare bridge, encoder and connector */
	port = of_graph_get_port_by_id(dev->of_node, 0);
	if(port) {
		struct device_node *endpoint;

		endpoint = of_get_child_by_name(port, "endpoint");
		if(endpoint) {
			output_node = of_graph_get_remote_port_parent(endpoint);
			of_node_put(endpoint);
		}
	}
	if(!output_node) {
		DRM_ERROR("failed to find endpoint in the port\n");
		return -ENODEV;
	}
	bridge = &vga->bridge;
	bridge = of_drm_find_bridge(output_node);
	
	if(!bridge) {
		dev_err(dev, "failed to find a bridge node\n");
		of_node_put(output_node);
		return -EPROBE_DEFER;
	}

	encoder = &vga->encoder;
	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev, dev->of_node);

	drm_encoder_helper_add(encoder, &rockchip_vga_encoder_helper_funcs);
	ret = drm_encoder_init(drm_dev, encoder, &rockchip_vga_encoder_funcs,
				DRM_MODE_ENCODER_DAC, NULL);
	if(ret < 0) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	encoder->bridge = bridge;
	bridge->encoder = encoder;
	
	connector = &vga->connector;
	connector->dpms = DRM_MODE_DPMS_OFF;
	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				DRM_CONNECTOR_POLL_DISCONNECT;
	
	drm_connector_helper_add(connector, &rockchip_vga_connector_helper_funcs);
	ret = drm_connector_init(drm_dev, connector, &rockchip_vga_connector_funcs,
				DRM_MODE_CONNECTOR_VGA);
	if (ret < 0) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err_free_encoder;
	}

	ret = drm_bridge_attach(drm_dev, bridge);
	if (ret < 0) {
		goto err_free_encoder;
	}

	ret = drm_mode_connector_attach_encoder(connector, encoder);	
	if (ret < 0) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector;
	}

	pm_runtime_enable(dev);

	return 0;

err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void rockchip_vga_unbind(struct device *dev, struct device *master,
					void *data)
{
	struct rockchip_vga *vga = dev_get_drvdata(dev);

	rockchip_vga_encoder_dpms(&vga->encoder, DRM_MODE_DPMS_OFF);

	drm_connector_cleanup(&vga->connector);
	drm_encoder_cleanup(&vga->encoder);

	pm_runtime_disable(dev);
}

static const struct component_ops rockchip_vga_component_ops = {
	.bind = rockchip_vga_bind,
	.unbind = rockchip_vga_unbind,
};

static int rockchip_vga_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_vga *vga;
	struct device_node *ddc_node;
	const struct of_device_id *match;
	int ret;

	if(!dev->of_node)
		return -ENODEV;

	vga = devm_kzalloc(&pdev->dev, sizeof(*vga), GFP_KERNEL);
	if(!vga)
		return -ENOMEM;

	vga->dev = dev;
	vga->suspend = true;

	match = of_match_node(rockchip_vga_dt_ids, dev->of_node);

	dev_set_drvdata(dev, vga);
	mutex_init(&vga->suspend_lock);

	vga->gpio = devm_gpiod_get_optional(dev, "vga", GPIOD_OUT_LOW);
	if (IS_ERR(vga->gpio)) {
		ret = PTR_ERR(vga->gpio);
		dev_err(dev, "failed to request GPIO: %d\n", ret);
		return ret;
	}

	ddc_node = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
	if (ddc_node) {
		vga->ddc = of_find_i2c_adapter_by_node(ddc_node);
		of_node_put(ddc_node);
		if (!vga->ddc) {
			dev_err(vga->dev, "failed to read ddc node\n");
			return -EPROBE_DEFER;
		}
	} else {
		dev_err(vga->dev, "no ddc property found\n");
	}
	
	vga->reg = devm_regulator_get_optional(dev, "vga");

	return component_add(dev, &rockchip_vga_component_ops);
}

static int rockchip_vga_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rockchip_vga_component_ops);

	return 0;
}

static struct platform_driver rockchip_vga_driver = {
	.probe = rockchip_vga_probe,
	.remove = rockchip_vga_remove,
	.driver = {
			.name = "rockchip-vga",
			.of_match_table = rockchip_vga_dt_ids,
	},
};

module_platform_driver(rockchip_vga_driver);

MODULE_AUTHOR("Steven Lee <geekerlw@gmail.com>");
MODULE_DESCRIPTION("Rockchip Vga Driver");
MODULE_LICENSE("GPL");
