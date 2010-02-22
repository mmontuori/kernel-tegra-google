obj-y += irq.o
obj-y += io.o
obj-y += pinmux.o

obj-${CONFIG_MACH_HARMONY} += board-harmony.o
obj-${CONFIG_MACH_HARMONY} += board-harmony-pinmux.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += clock.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_clocks.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += timer.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += gpio.o
obj-$(CONFIG_SMP)			+= platsmp.o localtimer.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
