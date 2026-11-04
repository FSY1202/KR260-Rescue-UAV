// SPDX-License-Identifier: GPL-2.0-only
/* Fixed-mode DRM scanout for an already configured PYNQ-Z2 HDMI VDMA.
 * Clocks/timing and the FPGA image are owned by the desktop setup service.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

static unsigned long vdma_base = 0x43000000;
module_param(vdma_base, ulong, 0444);
MODULE_PARM_DESC(vdma_base, "Physical base of the configured HDMI AXI VDMA");

struct pynq_display {
    struct drm_device drm;
    struct drm_simple_display_pipe pipe;
    struct drm_connector connector;
    void __iomem *regs;
};

static struct platform_device *pynq_device;
static const struct drm_display_mode fixed_mode = {
    .clock = 40000,
    .hdisplay = 800, .hsync_start = 840, .hsync_end = 968, .htotal = 1056,
    .vdisplay = 600, .vsync_start = 601, .vsync_end = 605, .vtotal = 628,
    .flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
    .type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static struct pynq_display *to_display(struct drm_simple_display_pipe *pipe)
{
    return container_of(pipe, struct pynq_display, pipe);
}

static enum drm_connector_status connector_detect(struct drm_connector *c, bool force)
{
    return connector_status_connected;
}

static int connector_modes(struct drm_connector *connector)
{
    struct drm_display_mode *mode = drm_mode_duplicate(connector->dev, &fixed_mode);
    if (!mode)
        return 0;
    drm_mode_set_name(mode);
    drm_mode_probed_add(connector, mode);
    return 1;
}

static const struct drm_connector_funcs connector_funcs = {
    .detect = connector_detect,
    .fill_modes = drm_helper_probe_single_connector_modes,
    .destroy = drm_connector_cleanup,
    .reset = drm_atomic_helper_connector_reset,
    .atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
    .atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};
static const struct drm_connector_helper_funcs connector_helpers = {
    .get_modes = connector_modes,
};

static void scanout_stop(struct pynq_display *display)
{
    u32 status;
    writel(readl(display->regs) & ~1U, display->regs);
    if (readl_poll_timeout(display->regs + 4, status, status & 1, 100, 100000))
        writel(4, display->regs); /* Stop/reset before any framebuffer can be freed. */
}

static void scanout_frame(struct drm_simple_display_pipe *pipe,
                          struct drm_plane_state *state)
{
    struct pynq_display *display = to_display(pipe);
    struct drm_framebuffer *fb = state->fb;
    dma_addr_t address;
    u32 status;
    int i;
    if (!fb)
        return;
    address = drm_fb_dma_get_gem_addr(fb, state, 0);
    if (upper_32_bits(address)) {
        scanout_stop(display);
        return;
    }
    /* Every park slot references the current BO. The engine latches the new
     * addresses at a frame boundary. Wait two frame completions before the
     * atomic helper is allowed to release the preceding framebuffer.
     */
    for (i = 0; i < 3; ++i)
        writel(lower_32_bits(address), display->regs + 0x5c + i * 4);
    writel(fb->pitches[0], display->regs + 0x58);
    writel(800 * 4, display->regs + 0x54);
    writel(0x7000, display->regs + 4);
    writel(readl(display->regs + 0x28) & ~0x1fU, display->regs + 0x28);
    writel(0x00010089, display->regs); /* PYNQ park/genlock mode, IRQs masked. */
    writel(600, display->regs + 0x50);
    for (i = 0; i < 2; ++i) {
        if (readl_poll_timeout(display->regs + 4, status,
                               status & 0x1000, 200, 100000)) {
            drm_err_ratelimited(&display->drm, "HDMI frame timeout, DMA status=%08x\n", status);
            scanout_stop(display);
            return;
        }
        writel(0x1000, display->regs + 4);
    }
}

static enum drm_mode_status pipe_mode_valid(struct drm_simple_display_pipe *pipe,
                                             const struct drm_display_mode *mode)
{
    return mode->hdisplay == 800 && mode->vdisplay == 600 ? MODE_OK : MODE_BAD;
}
static void pipe_enable(struct drm_simple_display_pipe *pipe,
                         struct drm_crtc_state *crtc_state,
                         struct drm_plane_state *plane_state)
{
    scanout_frame(pipe, plane_state);
}
static void pipe_disable(struct drm_simple_display_pipe *pipe)
{
    scanout_stop(to_display(pipe));
}
static void pipe_update(struct drm_simple_display_pipe *pipe,
                         struct drm_plane_state *old_state)
{
    if (pipe->crtc.state->active)
        scanout_frame(pipe, pipe->plane.state);
}
static const struct drm_simple_display_pipe_funcs pipe_funcs = {
    .mode_valid = pipe_mode_valid,
    .enable = pipe_enable,
    .disable = pipe_disable,
    .update = pipe_update,
};
static const struct drm_mode_config_funcs mode_funcs = {
    .fb_create = drm_gem_fb_create,
    .atomic_check = drm_atomic_helper_check,
    .atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(pynq_fops);
static const struct drm_driver pynq_driver = {
    .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
    .fops = &pynq_fops,
    DRM_GEM_DMA_DRIVER_OPS,
    .name = "pynq_hdmi", .desc = "PYNQ-Z2 HDMI desktop scanout",
    .date = "20261102", .major = 1, .minor = 0,
};

static int display_probe(struct platform_device *pdev)
{
    static const u32 formats[] = { DRM_FORMAT_XRGB8888 };
    struct pynq_display *display;
    struct drm_device *drm;
    int ret;
    if (!pdev->dev.dma_mask)
        pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (ret)
        return ret;
    display = devm_drm_dev_alloc(&pdev->dev, &pynq_driver, struct pynq_display, drm);
    if (IS_ERR(display))
        return PTR_ERR(display);
    drm = &display->drm;
    display->regs = devm_ioremap(&pdev->dev, vdma_base, 0x10000);
    if (!display->regs)
        return -ENOMEM;
    ret = drmm_mode_config_init(drm);
    if (ret)
        return ret;
    drm->mode_config.funcs = &mode_funcs;
    drm->mode_config.min_width = drm->mode_config.max_width = 800;
    drm->mode_config.min_height = drm->mode_config.max_height = 600;
    drm->mode_config.preferred_depth = 24;
    ret = drm_connector_init(drm, &display->connector, &connector_funcs,
                             DRM_MODE_CONNECTOR_HDMIA);
    if (ret)
        return ret;
    drm_connector_helper_add(&display->connector, &connector_helpers);
    display->connector.display_info.width_mm = 210;
    display->connector.display_info.height_mm = 158;
    ret = drm_simple_display_pipe_init(drm, &display->pipe, &pipe_funcs,
                                        formats, ARRAY_SIZE(formats), NULL,
                                        &display->connector);
    if (ret)
        return ret;
    drm_mode_config_reset(drm);
    platform_set_drvdata(pdev, display);
    return drm_dev_register(drm, 0);
}
static int display_remove(struct platform_device *pdev)
{
    struct pynq_display *display = platform_get_drvdata(pdev);
    drm_dev_unregister(&display->drm);
    drm_atomic_helper_shutdown(&display->drm);
    scanout_stop(display);
    return 0;
}
static struct platform_driver platform_driver = {
    .probe = display_probe, .remove = display_remove,
    .driver = { .name = "pynq-hdmi-drm" },
};
static int __init display_init(void)
{
    int ret = platform_driver_register(&platform_driver);
    if (ret)
        return ret;
    pynq_device = platform_device_register_simple("pynq-hdmi-drm", -1, NULL, 0);
    if (IS_ERR(pynq_device)) {
        platform_driver_unregister(&platform_driver);
        return PTR_ERR(pynq_device);
    }
    return 0;
}
static void __exit display_exit(void)
{
    platform_device_unregister(pynq_device);
    platform_driver_unregister(&platform_driver);
}
module_init(display_init);
module_exit(display_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Fixed-mode PYNQ-Z2 AXI VDMA DRM display");
