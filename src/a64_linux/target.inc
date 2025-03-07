#
# Linux kernel configuration
#

# kernel fundamentals
LX_ENABLE += TTY SERIAL_EARLYCON SERIAL_OF_PLATFORM PRINTK HAS_IOMEM

# initrd support
LX_ENABLE += BINFMT_ELF BLK_DEV_INITRD

# SoC
LX_ENABLE += ARCH_SUNXI

# UART device
LX_ENABLE += SERIAL_8250 $(addprefix SERIAL_8250_,16550A_VARIANTS DW CONSOLE)

# network infrastructure
LX_ENABLE += NET NETDEVICES ETHERNET

# network driver
LX_ENABLE += NET_VENDOR_STMICRO STMMAC_ETH STMMAC_PLATFORM DWMAC_SUN8I

# ethernet PHY
LX_ENABLE += OF_MDIO MDIO_DEVICE PHYLIB
LX_ENABLE += MOTORCOMM_PHY                # needed for Pine-A64-LTS-V2

# network protocols
LX_ENABLE += INET IP_PNP IP_PNP_DHCP

# framebuffer driver
LX_ENABLE += DRM DRM_SUN4I DRM_SUN8I_MIXER DRM_SUN8I_DW_HDMI

# LCD display support (MIPI/DSI + panel + backlight)
LX_ENABLE += DRM_MIPI_DSI DRM_SUN6I_DSI GENERIC_PHY_MIPI_DPHY PHY_SUN6I_MIPI_DPHY
LX_ENABLE += DRM_PANEL_SITRONIX_ST7703
LX_ENABLE += PWM PWM_SUN4I BACKLIGHT_PWM BACKLIGHT_CLASS_DEVICE

# determined by bisecting kernel configuration options (needed by fb driver)
LX_ENABLE += CMA DMA_CMA
LX_ENABLE += MFD_AXP20X_RSB REGULATOR REGULATOR_AXP20X
LX_ENABLE += PROC_FS SYSFS

# to automatically set up screen mode at boot time
LX_ENABLE += FB FRAMEBUFFER_CONSOLE

# show Tux
LX_ENABLE += LOGO

# touchscreen
LX_ENABLE += INPUT_TOUCHSCREEN TOUCHSCREEN_GOODIX I2C I2C_MV64XXX FW_LOADER

# slim down kernel by removing superfluous drivers
LX_DISABLE += $(addprefix NET_VENDOR_, ALACRITECH AMAZON AQUANTIA ARC BROADCOM \
                                       CADENCE CAVIUM CORTINA EZCHIP GOOGLE \
                                       HISILICON HUAWEI I825XX INTEL MARVELL \
                                       MICREL MICROCHIP MICROSEMI NATSEMI \
                                       NETRONOME NI 8390 PENSANDO QUALCOMM \
                                       RENESAS ROCKER SAMSUNG SEEQ SOLARFLARE \
                                       SMSC SOCIONEXT)
LX_DISABLE += WLAN

