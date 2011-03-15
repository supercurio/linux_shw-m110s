/*
 * wm8994.h  --  WM8994 Soc Audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _WM8994_H
#define _WM8994_H

#include <sound/soc.h>

extern struct snd_soc_codec_device soc_codec_dev_wm8994;
// We don't use array	- DW Shim.
//extern struct snd_soc_dai wm8994_dai[];

/* Sources for AIF1/2 SYSCLK - use with set_dai_sysclk() */
#define WM8994_SYSCLK_MCLK1 1
#define WM8994_SYSCLK_MCLK2 2
#define WM8994_SYSCLK_FLL1  3
#define WM8994_SYSCLK_FLL2  4

#define WM8994_FLL1 1
#define WM8994_FLL2 2

//-----------------------------------------------------------
// Added belows codes by Samsung Electronics.

#include "wm8994_def.h"

extern struct snd_soc_dai wm8994_dai;

#define WM8994_SYSCLK_MCLK      1
#define WM8994_SYSCLK_FLL       2

#define AUDIO_COMMON_DEBUG      0

// if you want to see log, set this definition to NULL or KERN_WARNING
#define SND_KERN_DEBUG          KERN_DEBUG

//------------------------------------------------
// Definitions of DRC Feature for VoIP
//------------------------------------------------
//#define FEATURE_VOIP_DRC // DRC Feature disabled
#define FEATURE_VOIP_HPFILTER // High Pass Filter enabled

//------------------------------------------------
// Definitions of enum type
//------------------------------------------------
//enum playback_path {OFF, RCV, SPK, HP, BT, SPK_HP, EXTRA_DOCK_SPEAKER=9};
enum playback_path {OFF, RCV, SPK, HP, BT, SPK_HP, RING_SPK, RING_HP, RING_DUAL, EXTRA_DOCK_SPEAKER, TV_OUT};
//enum mic_path      {MAIN, EAR, BT_REC, MIC_OFF};
enum mic_path      {MAIN, EAR, MIC_OFF};
enum power_state   {CODEC_OFF, CODEC_ON};
enum recognition   {REC_OFF, REC_ON};

#define DEACTIVE				0x00
#define PLAYBACK_ACTIVE			0x01
#define CAPTURE_ACTIVE			0x02
#define CALL_ACTIVE				0x04
#define FMRADIO_ACTIVE			0x08
#define VOICE_CALL_ACTIVE		0x10
#define VOIP_CALL_ACTIVE		0x20

#define PCM_STREAM_DEACTIVE		0x00
#define PCM_STREAM_PLAYBACK		0x01
#define PCM_STREAM_CAPTURE		0x02

#define CMD_FMR_INPUT_DEACTIVE       0 // Codec Input PGA off for reducing white noise.
#define CMD_FMR_INPUT_ACTIVE         1 // Codec Input PGA on
#define CMD_FMR_FLAG_CLEAR           2 // Radio flag clear for shutdown - to reduce pop up noise.
#define CMD_FMR_END                  3 // Codec off in FM radio mode - to reduce pop up noise.
#define CMD_RECOGNITION_DEACTIVE     4 // Distingush recognition gain. To use default MIC gain.
#define CMD_RECOGNITION_ACTIVE       5 // Distingush recognition gain. To use MIC gain for recognition.
#define CMD_CALL_FLAG_CLEAR          6 // Call flag clear for shutdown - to reduce pop up noise.
#define CMD_CALL_END                 7 // Codec off in call mode - to reduce pop up noise.
#define CMD_CODEC_STANDBY            8 // Playback flag clear for shutdown. It's possible for codec to go sleep mode when sound is not played after path setting.
#define CMD_CODEC_EMERGENCY_RECOVERY 9 // Emergency recovery for Error like -EIO, -ESTRPIPE, and etc.

typedef void (*select_route)(struct snd_soc_codec *);
typedef void (*select_mic_route)(struct snd_soc_codec *);

struct wm8994_setup_data {
	int i2c_bus;
	unsigned short i2c_address;
};

struct wm8994_priv {
	//u16 reg_cache[WM8994_REGISTER_COUNT];
	struct snd_soc_codec codec;
	int master;
	int sysclk_source;
	unsigned int mclk_rate;
	unsigned int sysclk_rate;
	unsigned int fs;
	unsigned int bclk;
	unsigned int hw_version;		// For wolfson H/W version. 1 = Rev B, 3 = Rev D
	unsigned int codec_state;
	unsigned int stream_state;
	enum playback_path cur_path;
	enum mic_path rec_path;
	enum power_state power_state;
	enum recognition recognition_active;		// for control gain to voice recognition.
	select_route *universal_playback_path;
	select_route *universal_voicecall_path;
	select_mic_route *universal_mic_path;
	select_route *universal_voipcall_path;
	int testmode_config_flag;	// for testmode.
};

#if AUDIO_COMMON_DEBUG
#define DEBUG_LOG(format,...)\
	printk (SND_KERN_DEBUG "["SUBJECT" (%s,%d)] " format "\n", __func__, __LINE__, ## __VA_ARGS__);
#else
#define DEBUG_LOG(format,...)
#endif

#define DEBUG_LOG_ERR(format,...)\
	printk ("["SUBJECT" (%s,%d)] " format "\n", __func__, __LINE__, ## __VA_ARGS__);

// Definitions of function prototype.
inline unsigned int wm8994_read(struct snd_soc_codec *codec,unsigned int reg);
int wm8994_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value);
int wm8994_write_mask(struct snd_soc_codec *codec, unsigned int reg, unsigned int mask, unsigned int value);
void wm8994_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *codec_dai);
int audio_init(void);
int audio_power(int en, int check);
void audio_ctrl_mic_bias_gpio(struct snd_soc_codec *codec, char callid);
void audio_ctrl_sleep_gpio(int enable);
void wm8994_block_control_playback(int on, struct snd_soc_codec *codec);
void wm8994_block_control_record(int on, struct snd_soc_codec *codec);
void wm8994_set_off(struct snd_soc_codec *codec);
void wm8994_disable_playback_path(struct snd_soc_codec *codec, enum playback_path path);
void wm8994_disable_rec_path(struct snd_soc_codec *codec, enum mic_path rec_path);
void wm8994_record_main_mic(struct snd_soc_codec *codec);
void wm8994_record_headset_mic(struct snd_soc_codec *codec);
void wm8994_record_bluetooth_mic(struct snd_soc_codec *codec);
void wm8994_set_playback_receiver(struct snd_soc_codec *codec);
void wm8994_set_playback_headset(struct snd_soc_codec *codec);
void wm8994_set_playback_speaker(struct snd_soc_codec *codec);
void wm8994_set_playback_speaker_headset(struct snd_soc_codec *codec);
void wm8994_set_playback_bluetooth(struct snd_soc_codec *codec);
void wm8994_set_playback_extra_dock_speaker(struct snd_soc_codec *codec);
void wm8994_set_voicecall_receiver(struct snd_soc_codec *codec);
void wm8994_set_voicecall_headset(struct snd_soc_codec *codec);
void wm8994_set_voicecall_headphone(struct snd_soc_codec *codec);
void wm8994_set_voicecall_speaker(struct snd_soc_codec *codec);
void wm8994_set_voicecall_bluetooth(struct snd_soc_codec *codec);
void close_output_path_all(struct snd_soc_codec *codec);
void wm8994_set_voipcall_receiver(struct snd_soc_codec *codec);
void wm8994_set_voipcall_headset(struct snd_soc_codec *codec);
void wm8994_set_voipcall_headphone(struct snd_soc_codec *codec);
void wm8994_set_voipcall_speaker(struct snd_soc_codec *codec);
void wm8994_set_voipcall_bluetooth(struct snd_soc_codec *codec);
#endif

