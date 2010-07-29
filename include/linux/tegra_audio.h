/* include/linux/tegra_audio.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TEGRA_AUDIO_H
#define _TEGRA_AUDIO_H

#include <asm/ioctl.h>

#define TEGRA_AOUT_MAGIC 'o'

#define TEGRA_AUDIO_OUT_PAUSE  _IO(TEGRA_AOUT_MAGIC, 0)
#define TEGRA_AUDIO_OUT_RESUME _IO(TEGRA_AOUT_MAGIC, 1)

#endif/*_TEGRA_AUDIO_H*/
