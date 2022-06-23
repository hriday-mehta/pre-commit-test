/**
 * Copyright (C) 2021 EERS Global Technologies Inc. - All Rights Reserved.
 * This software is proprietary, confidential, and may only be used in accordance
 * with applicable licensing terms, or as directed by EERS in writing. In any
 * case, any redistributions of this software must retain this notice.

 * Unless required by applicable law, provided otherwise in the licensing
 * terms, or agreed to in writing, this software is distributed on an "AS IS"
 * basis, without warranties of any kind, either express or implied and in no
 * event shall the copyright holder be liable for any direct, indirect,
 * incidental, special, or consequential damages arising in any way out of
 * the use of this software.
 */

#include <Arduino.h>
#include <Audio.h>
#include <math.h>
#include <stdint.h>

#include "src/CComplex/ccomplex.h"

#include "eeprom_data.h"
#include "eeprom_internal_data.h"
#include "io.h"
#include "pink_noise.h"

#define DEBUG_ENABLED
#include "debug.h"

#include "audio.h"
#include "src/kissfft/kiss_fftr.h"


// From the audio framework
#define SAMPLE_RATE ((int)AUDIO_SAMPLE_RATE_EXACT)

#define DELAY_PLAYBACK_TO_MIC_SAMPLES 660
#define PRETEST_DURATION_SEC 1
#define SINE_TONE_PLAYBACK_DURATION 5
#define TEST1_DURATION_SEC 10
#define TEST2A_DURATION_SEC 10
#define TEST2B_DURATION_SEC 10
#define TEST3_DURATION_SEC 5
#define HWSERIAL_DELAY_MS 500

#define NB_BLOCKS_IN_FFTSIZE (FFTSIZE / AUDIO_BLOCK_SAMPLES)

#define KISS_FFT_OUT_SIZE ((FFTSIZE / 2) + 1)

//Sine wave for debug
static AudioSynthWaveformSine AudioSynthWaveformSine_1;

//Analysis modules
static AudioAnalyzeRMS AudioAnalyzeRMS_IEM_L;
static AudioAnalyzeRMS AudioAnalyzeRMS_IEM_R;
static AudioAnalyzeRMS AudioAnalyzeRMS_OEM_L;
static AudioAnalyzeRMS AudioAnalyzeRMS_OEM_R;

//Input buffers
static AudioRecordQueue AudioRecordQueue_OEM_L; //in1_L //TODO: validate these comments
static AudioRecordQueue AudioRecordQueue_IEM_L; //in1_R
static AudioRecordQueue AudioRecordQueue_OEM_R; //in2_L
static AudioRecordQueue AudioRecordQueue_IEM_R; //in2_R
static AudioRecordQueue AudioRecordQueue_SINE;

//Output buffers
static AudioPlayQueue AudioPlayQueue_SPK_L; //in1_L //TODO: validate these comments
static AudioPlayQueue AudioPlayQueue_SPK_R; //in1_R
static AudioPlayQueue AudioPlayQueue_CAL_L; //in2_L
static AudioPlayQueue AudioPlayQueue_CAL_R; //in2_R

static AudioInputI2SQuad AudioInputI2SQuad_1;   //Both input I2S streams (HW pin 8 and 6 on teensy 4.0)
static AudioOutputI2SQuad AudioOutputI2SQuad_1; //Both output I2S streams (HW pin 7 and 32 on teensy 4.0)

//Connecting the I2S inputs to the appropriate buffers
static AudioConnection patchCord1(AudioInputI2SQuad_1, 0, AudioRecordQueue_OEM_R,
                                  0); //ch 1-2 are R earpiece //TODO: validate these comments
static AudioConnection patchCord2(AudioInputI2SQuad_1, 1, AudioRecordQueue_IEM_R, 0);
static AudioConnection patchCord3(AudioInputI2SQuad_1, 2, AudioRecordQueue_OEM_L, 0); //ch 3-4 are L earpiece
static AudioConnection patchCord4(AudioInputI2SQuad_1, 3, AudioRecordQueue_IEM_L, 0);

//Connecting the I2S outputs to the appropriate buffers
static AudioConnection patchCord5(AudioPlayQueue_SPK_L, 0, AudioOutputI2SQuad_1,
                                  0); //ch 1-2 are earpiece SPKs //TODO: validate these comments
static AudioConnection patchCord6(AudioPlayQueue_SPK_R, 0, AudioOutputI2SQuad_1, 1);
static AudioConnection patchCord7(AudioPlayQueue_CAL_L, 0, AudioOutputI2SQuad_1, 2); //ch 3-4 are Cal SPKs
static AudioConnection patchCord8(AudioPlayQueue_CAL_R, 0, AudioOutputI2SQuad_1, 3);

//Connecting the I2S inputs to RMS modules
static AudioConnection patchCord10(AudioInputI2SQuad_1, 0, AudioAnalyzeRMS_OEM_R, 0);
static AudioConnection patchCord11(AudioInputI2SQuad_1, 1, AudioAnalyzeRMS_IEM_R, 0);
static AudioConnection patchCord12(AudioInputI2SQuad_1, 2, AudioAnalyzeRMS_OEM_L, 0);
static AudioConnection patchCord13(AudioInputI2SQuad_1, 3, AudioAnalyzeRMS_IEM_L, 0);

// Connect debug sine wave to appropirate buffers
static AudioConnection patchCord14(AudioSynthWaveformSine_1, 0, AudioRecordQueue_SINE, 0);

static AudioControlSGTL5000 AudioControlSGTL5000_1; //shield 1 is earpiece's loudspeakers
static AudioControlSGTL5000 AudioControlSGTL5000_2; //shield 2 is Calib loudspeakers, y-splitter

static int next_fft_block_number;
static bool fft_data_available;

//Align buffers to multiples of 4 bytes in case they are accessed as uint32, as pink noise is doing

// Regular sample buffers to interact with RecordQueues and PlayQueues
static int16_t bOEM_L[FFTSIZE] __attribute__((aligned(4)));
static int16_t bIEM_L[FFTSIZE] __attribute__((aligned(4)));
static int16_t bOEM_R[FFTSIZE] __attribute__((aligned(4)));
static int16_t bIEM_R[FFTSIZE] __attribute__((aligned(4)));
static int16_t bNoise[FFTSIZE]
    __attribute__((aligned(4))); //technically we only need AUDIO_BLOCK_SAMPLES, but using FFTSIZE for coherence
static int16_t bNoise_delayed[FFTSIZE] __attribute__((aligned(4)));
static int16_t bSine[AUDIO_BLOCK_SAMPLES];


// buffers for kiss fft input (scalar)
static kiss_fft_scalar fftIn[FFTSIZE];

// buffers for kiss fft output (complex)
static kiss_fft_cpx fftOEM_L[KISS_FFT_OUT_SIZE];
static kiss_fft_cpx fftIEM_L[KISS_FFT_OUT_SIZE];
static kiss_fft_cpx fftOEM_R[KISS_FFT_OUT_SIZE];
static kiss_fft_cpx fftIEM_R[KISS_FFT_OUT_SIZE];
static kiss_fft_cpx fftNoise_delayed[KISS_FFT_OUT_SIZE];

//Noise played
static float NoiseSquaredCumul[FFTSIZE / 2];

// Measured values
static float MIEMLSquaredCumul[FFTSIZE / 2];
static float MIEMRSquaredCumul[FFTSIZE / 2];
static float MOEMLSquaredCumul[FFTSIZE / 2];
static float MOEMRSquaredCumul[FFTSIZE / 2];

static bool audio_headset_connected = false;
static bool audio_headset_eeprom_alive = false;


typedef enum
{
    TEST_TYPE_0,
    TEST_TYPE_1,
    TEST_TYPE_2A,
    TEST_TYPE_2B,
    TEST_TYPE_3,
    TEST_TYPE_SINE_DEBUG,
} test_type_t;

static inline float amplitude2dB(float amplitude_value)
{
    return 20.0f * log10f(amplitude_value);
}

static inline float dB2amplitude(float dB_value)
{
    return powf(10.0f, dB_value / 20.0f);
}

static inline float energy2dB(float energy_value)
{
    return 10.0f * log10f(energy_value);
}

static inline float dB2energy(float dB_value)
{
    return powf(10.0f, dB_value / 10.0f);
}

static void beginAudioRecordQueues()
{
    AudioRecordQueue_OEM_L.begin();
    AudioRecordQueue_IEM_L.begin();
    AudioRecordQueue_OEM_R.begin();
    AudioRecordQueue_IEM_R.begin();
    AudioRecordQueue_SINE.begin();
}

static void endAudioRecordQueues()
{
    AudioRecordQueue_OEM_L.end();
    AudioRecordQueue_IEM_L.end();
    AudioRecordQueue_OEM_R.end();
    AudioRecordQueue_IEM_R.end();
    AudioRecordQueue_SINE.end();
}

static inline void enableAudioChain()
{
    beginAudioRecordQueues();
    audio_enable_interrupts();
}

static inline void disableAudioChain()
{
    endAudioRecordQueues();
    audio_disable_interrupts();
}

static void ReadRecordQueue(AudioRecordQueue &rq, int16_t dest_buf[AUDIO_BLOCK_SAMPLES])
{
    const int16_t *read_buf = rq.readBuffer();
    PanicFalse(read_buf != NULL);
    memcpy(dest_buf, read_buf, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
    rq.freeBuffer();
}

static void WritePlayQueue(AudioPlayQueue &pq, const int16_t source_buf[AUDIO_BLOCK_SAMPLES])
{
    int16_t *write_buf = pq.getBuffer();
    PanicFalse(write_buf != NULL);
    memcpy(write_buf, source_buf, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
    pq.playBuffer();
}

static void ManageQueueBuffers(test_type_t test_type, uint8_t channel)
{
    ReadRecordQueue(AudioRecordQueue_OEM_L, &bOEM_L[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
    ReadRecordQueue(AudioRecordQueue_IEM_L, &bIEM_L[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
    ReadRecordQueue(AudioRecordQueue_OEM_R, &bOEM_R[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
    ReadRecordQueue(AudioRecordQueue_IEM_R, &bIEM_R[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
    ReadRecordQueue(AudioRecordQueue_SINE, bSine);

#if 0
    // This code can be used to calculate the delay between noise samples generated and when they are available in the mic inputs
    static int aa = 0;
    if(aa < 10)
    {
        if(aa == 0)
            console_write("a = [ ");
        for(int i=0; i<AUDIO_BLOCK_SAMPLES; i++)
        {
            console_write("%d ", bIEM_L[next_fft_block_number*AUDIO_BLOCK_SAMPLES+i]);
        }
        if(aa == 9)
            console_write("];\n");
        aa++;
    }
#endif

    pink_noise_get(&bNoise[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
    pink_noise_get_delayed(&bNoise_delayed[next_fft_block_number * AUDIO_BLOCK_SAMPLES], DELAY_PLAYBACK_TO_MIC_SAMPLES);

    // Output noise to appropriate shield depending on test
    // We play back the non-delayed noise, and analyze the delayed noise to get sync between REF and MIC FFTs
    if((test_type == TEST_TYPE_0) || (test_type == TEST_TYPE_1) || (test_type == TEST_TYPE_2B))
    {
        WritePlayQueue(AudioPlayQueue_SPK_L, &bNoise[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
        WritePlayQueue(AudioPlayQueue_SPK_R, &bNoise[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
    }
    else if((test_type == TEST_TYPE_2A) || (test_type == TEST_TYPE_3))
    {
        WritePlayQueue(AudioPlayQueue_CAL_L, &bNoise[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
        WritePlayQueue(AudioPlayQueue_CAL_R, &bNoise[next_fft_block_number * AUDIO_BLOCK_SAMPLES]);
    }
    else if(test_type == TEST_TYPE_SINE_DEBUG)
    {
        switch(channel)
        {
            case 0:
                WritePlayQueue(AudioPlayQueue_SPK_L, bSine);
                break;
            case 1:
                WritePlayQueue(AudioPlayQueue_SPK_R, bSine);
                break;
            case 2:
                WritePlayQueue(AudioPlayQueue_CAL_L, bSine);
                break;
            case 3:
                WritePlayQueue(AudioPlayQueue_CAL_R, bSine);
                break;
            default:
                break;
        }
    }
    else
    {
        Panic();
    }

    next_fft_block_number = (next_fft_block_number + 1) % NB_BLOCKS_IN_FFTSIZE;
    if(next_fft_block_number == 0) //only after wrap-around
    {
        fft_data_available = true;
    }
}

// To map q1.15 to [-1, 1)
const float Q_SCALING_FACTOR = 1.0f / 32768.0f;

extern const int16_t AudioWindowHamming1024[];
static void apply_window_to_fft_buffer(kiss_fft_scalar buf[FFTSIZE], const int16_t window[FFTSIZE])
{
    PanicFalse(buf != NULL);
    PanicFalse(window != NULL);

    for(int i = 0; i < FFTSIZE; i++)
    {
        buf[i] *= (window[i] * Q_SCALING_FACTOR);
    }
}

// kiss_fft_scalar is a float in our configuration
static void copy_to_kiss_fft_buffer(kiss_fft_scalar dest_buf[FFTSIZE], const int16_t src_buf[FFTSIZE])
{
    PanicFalse(dest_buf != NULL);
    PanicFalse(src_buf != NULL);

    for(int i = 0; i < FFTSIZE; i++)
    {
        dest_buf[i] = (src_buf[i] * Q_SCALING_FACTOR);
    }
}

static void ComputeFFT(kiss_fft_cpx fft_buf_dest[KISS_FFT_OUT_SIZE], const int16_t buf_src[FFTSIZE])
{
    PanicFalse(fft_buf_dest != NULL);
    PanicFalse(buf_src != NULL);

    kiss_fftr_cfg fft_cfg = kiss_fftr_alloc(FFTSIZE, 0, 0, 0);

    copy_to_kiss_fft_buffer(fftIn, buf_src);
    apply_window_to_fft_buffer(fftIn, AudioWindowHamming1024);

    kiss_fftr(fft_cfg, fftIn, fft_buf_dest);
    kiss_fftr_free(fft_cfg);
}

// Warning: processing is done in place and destroys the samples in the buffers
static void ComputeFFTs(void)
{
    PanicFalse(fft_data_available);

    ComputeFFT(fftOEM_L, bOEM_L);
    ComputeFFT(fftIEM_L, bIEM_L);
    ComputeFFT(fftOEM_R, bOEM_R);
    ComputeFFT(fftIEM_R, bIEM_R);
    ComputeFFT(fftNoise_delayed, bNoise_delayed);
}

static inline float GetTFPoint(const float SyySquaredCumul[FFTSIZE / 2], const float SxxSquaredCumul[FFTSIZE / 2],
                               const int bin)
{
    PanicFalse(bin < (FFTSIZE / 2));

    return energy2dB(SyySquaredCumul[bin] / SxxSquaredCumul[bin]);
}

static void ComputeAccumulateFFT(float squared_cumul[FFTSIZE / 2], const kiss_fft_cpx fft_buf_src[KISS_FFT_OUT_SIZE])
{
    PanicFalse(squared_cumul != NULL);
    PanicFalse(fft_buf_src != NULL);

    for(int i = 0; i < (FFTSIZE / 2); i++)
    {
        Complex c(fft_buf_src[i].r, fft_buf_src[i].i);
        Complex ccj = c * c.conjugate();
        squared_cumul[i] += ccj.real();
    }
}

static void ComputeAccumulateFFTs(void)
{
    ComputeAccumulateFFT(NoiseSquaredCumul, fftNoise_delayed);
    ComputeAccumulateFFT(MOEMLSquaredCumul, fftOEM_L);
    ComputeAccumulateFFT(MIEMLSquaredCumul, fftIEM_L);
    ComputeAccumulateFFT(MOEMRSquaredCumul, fftOEM_R);
    ComputeAccumulateFFT(MIEMRSquaredCumul, fftIEM_R);
}

static void ResetAccumulateBuffer(float buf[FFTSIZE / 2])
{
    memset(buf, 0, (FFTSIZE / 2) * sizeof(float));
}

static void ResetAccumulateBuffers(void)
{
    ResetAccumulateBuffer(NoiseSquaredCumul);
    ResetAccumulateBuffer(MOEMLSquaredCumul);
    ResetAccumulateBuffer(MIEMLSquaredCumul);
    ResetAccumulateBuffer(MOEMRSquaredCumul);
    ResetAccumulateBuffer(MIEMRSquaredCumul);
}

static void ResetChain(void)
{
    next_fft_block_number = 0;
    fft_data_available = false;

    pink_noise_clear();

    audio_reset_record_queues();

    ResetAccumulateBuffers();
}

void audio_reset_record_queues(void)
{
    AudioRecordQueue_OEM_L.clear();
    AudioRecordQueue_IEM_L.clear();
    AudioRecordQueue_OEM_R.clear();
    AudioRecordQueue_IEM_R.clear();
}

static bool RecordQueueNotEmpty(AudioRecordQueue &rq)
{
    int rq_blocks_available = rq.available();

    // If buffers fill up too much, that's abnormal
    PanicFalse(rq_blocks_available <= 2);

    return (rq_blocks_available > 1);
}

static bool RecordQueuesNotEmpty(void)
{
    return RecordQueueNotEmpty(AudioRecordQueue_OEM_L) && RecordQueueNotEmpty(AudioRecordQueue_IEM_L) &&
           RecordQueueNotEmpty(AudioRecordQueue_OEM_R) && RecordQueueNotEmpty(AudioRecordQueue_IEM_R);
}

static void RunChain(test_type_t test_type, int duration_sec, bool enable_processing, uint8_t channel)
{
    int duration_nb_fft = duration_sec * SAMPLE_RATE / FFTSIZE;

    while(duration_nb_fft > 0)
    {
        if(RecordQueuesNotEmpty())
        {
            ManageQueueBuffers(test_type, channel);

            if(fft_data_available)
            {
                if(enable_processing)
                {
                    ComputeFFTs();
                    ComputeAccumulateFFTs();
                }

                fft_data_available = false;
                duration_nb_fft--;
            }
        }
    }
}

// This test calculates the frequency responses between external speaker connected through earpiece speaker lines and
// corresponding OEM
bool audio_run_test0(stray_test_result_t *STOEML, stray_test_result_t *STOEMR)
{
    PanicFalse(STOEML != NULL);
    PanicFalse(STOEMR != NULL);

    *STOEML = STRAY_RESULT_SUCCESS;
    *STOEMR = STRAY_RESULT_SUCCESS;

    AudioControlSGTL5000_1.volume(0.7);
    AudioControlSGTL5000_2.volume(0.7);
    io_set_status_led_color(LED_COLOR_WHITE);

    ResetChain();
    enableAudioChain();

    // Prime the audio chain so output samples have been captures by mics
    RunChain(TEST_TYPE_0, PRETEST_DURATION_SEC, false, 0);

    DEBUG("Running test 0 for %d seconds\n", TEST1_DURATION_SEC);
    RunChain(TEST_TYPE_0, TEST1_DURATION_SEC, true, 0);
    DEBUG("Done - Analyzing Results\n");

    disableAudioChain();

    io_set_status_led_color(LED_COLOR_BLUE);

    return true;
}

// This test calculates the frequency responses between each earpiece speaker and corresponding IEM
bool audio_run_test1(stray_test_result_t *LTIEML, stray_test_result_t *LTIEMR)
{
    PanicFalse(LTIEML != NULL);
    PanicFalse(LTIEMR != NULL);

    if(!audio_headset_connected)
    {
        *LTIEML = TEST_RESULT_NO_HEADSET;
        *LTIEMR = TEST_RESULT_NO_HEADSET;
        return false;
    }
    else if(!audio_headset_eeprom_alive)
    {
        *LTIEML = TEST_RESULT_NO_EEPROM;
        *LTIEMR = TEST_RESULT_NO_EEPROM;
        return false;
    }
    else
    {
        *LTIEML = STRAY_RESULT_SUCCESS;
        *LTIEMR = STRAY_RESULT_SUCCESS;
    }

    AudioControlSGTL5000_1.volume(0.7);
    AudioControlSGTL5000_2.volume(0.7);
    io_set_status_led_color(LED_COLOR_WHITE);

    ResetChain();
    enableAudioChain();

    // Prime the audio chain so output samples have been captures by mics
    RunChain(TEST_TYPE_1, PRETEST_DURATION_SEC, false, 0);

    DEBUG("Running test 1 for %d seconds\n", TEST1_DURATION_SEC);
    RunChain(TEST_TYPE_1, TEST1_DURATION_SEC, true, 0);
    DEBUG("Done - Analyzing Results\n");

    disableAudioChain();

    io_set_status_led_color(LED_COLOR_BLUE);

    return true;
}

// This test calculates the frequency responses between the calibrator speakers and the 4 earpiece microphones
bool audio_run_test2a(stray_test_result_t *STOEML, stray_test_result_t *STOEMR, stray_test_result_t *STIEML,
                      stray_test_result_t *STIEMR)
{
    PanicFalse(STOEML != NULL);
    PanicFalse(STOEMR != NULL);
    PanicFalse(STIEML != NULL);
    PanicFalse(STIEMR != NULL);

    if(!audio_headset_connected)
    {
        *STOEML = TEST_RESULT_NO_HEADSET;
        *STOEMR = TEST_RESULT_NO_HEADSET;
        *STIEML = TEST_RESULT_NO_HEADSET;
        *STIEMR = TEST_RESULT_NO_HEADSET;
        return false;
    }
    else if(!audio_headset_eeprom_alive)
    {
        *STOEML = TEST_RESULT_NO_EEPROM;
        *STOEMR = TEST_RESULT_NO_EEPROM;
        *STIEML = TEST_RESULT_NO_EEPROM;
        *STIEMR = TEST_RESULT_NO_EEPROM;
        return false;
    }
    else
    {
        *STOEML = STRAY_RESULT_SUCCESS;
        *STOEMR = STRAY_RESULT_SUCCESS;
        *STIEML = STRAY_RESULT_SUCCESS;
        *STIEMR = STRAY_RESULT_SUCCESS;
    }

    AudioControlSGTL5000_1.volume(0.7);
    AudioControlSGTL5000_2.volume(0.7);
    io_set_status_led_color(LED_COLOR_WHITE);

    ResetChain();
    enableAudioChain();

    // Prime the audio chain so output samples have been captures by mics
    RunChain(TEST_TYPE_2A, PRETEST_DURATION_SEC, false, 0);

    DEBUG("Running test 2A for %d seconds\n", TEST2A_DURATION_SEC);
    RunChain(TEST_TYPE_2A, TEST2A_DURATION_SEC, true, 0);
    DEBUG("Done - Analyzing Results\n");

    disableAudioChain();

    io_set_status_led_color(LED_COLOR_BLUE);

    return true;
}

// This test calculates the frequency responses between each earpiece speaker and corresponding IEM
bool audio_run_test2b(stray_test_result_t *LTIEML, stray_test_result_t *LTIEMR)
{
    PanicFalse(LTIEML != NULL);
    PanicFalse(LTIEMR != NULL);

    if(!audio_headset_connected)
    {
        *LTIEML = TEST_RESULT_NO_HEADSET;
        *LTIEMR = TEST_RESULT_NO_HEADSET;
        return false;
    }
    else if(!audio_headset_eeprom_alive)
    {
        *LTIEML = TEST_RESULT_NO_EEPROM;
        *LTIEMR = TEST_RESULT_NO_EEPROM;
        return false;
    }
    else
    {
        *LTIEML = STRAY_RESULT_SUCCESS;
        *LTIEMR = STRAY_RESULT_SUCCESS;
    }

    AudioControlSGTL5000_1.volume(0.7);
    AudioControlSGTL5000_2.volume(0.7);
    io_set_status_led_color(LED_COLOR_WHITE);

    ResetChain();
    enableAudioChain();

    // Prime the audio chain so output samples have been captures by mics
    RunChain(TEST_TYPE_2B, PRETEST_DURATION_SEC, false, 0);

    DEBUG("Running test 2B for %d seconds\n", TEST2B_DURATION_SEC);
    RunChain(TEST_TYPE_2B, TEST2B_DURATION_SEC, true, 0);
    DEBUG("Done - Analyzing Results\n");

    disableAudioChain();

    io_set_status_led_color(LED_COLOR_BLUE);

    return true;
}

// This test calculates the transfer function between OEM and IEM in order to detect leaks.
bool audio_run_test3(stray_test_result_t *LTL, stray_test_result_t *LTR)
{
    PanicFalse(LTL != NULL);
    PanicFalse(LTR != NULL);

    if(!audio_headset_connected)
    {
        *LTL = TEST_RESULT_NO_HEADSET;
        *LTR = TEST_RESULT_NO_HEADSET;
        return false;
    }
    else if(!audio_headset_eeprom_alive)
    {
        *LTL = TEST_RESULT_NO_EEPROM;
        *LTR = TEST_RESULT_NO_EEPROM;
        return false;
    }
    else
    {
        *LTL = STRAY_RESULT_SUCCESS;
        *LTR = STRAY_RESULT_SUCCESS;
    }

    AudioControlSGTL5000_1.volume(0.55); // Reset to output 80dB at unity gain on Presonus P5XT speakers
    AudioControlSGTL5000_2.volume(0.55); // Reset to output 80dB at unity gain on Presonus P5XT speakers
    io_set_status_led_color(LED_COLOR_WHITE);

    ResetChain();
    enableAudioChain();

    // Prime the audio chain so output samples have been captures by mics
    RunChain(TEST_TYPE_3, PRETEST_DURATION_SEC, false, 0);

    DEBUG("Running test 3 for %d seconds\n", TEST3_DURATION_SEC);
    RunChain(TEST_TYPE_3, TEST3_DURATION_SEC, true, 0);
    DEBUG("Done - Analyzing Results\n");

    disableAudioChain();
    io_set_status_led_color(LED_COLOR_BLUE);

    return true;
}

bool audio_play_sine(uint32_t channel)
{
    PanicFalse(channel < 4);

    DEBUG("Playing sine wave on channel %lu for %d seconds\n", channel, SINE_TONE_PLAYBACK_DURATION);
    ResetChain();
    enableAudioChain();
    RunChain(TEST_TYPE_SINE_DEBUG, SINE_TONE_PLAYBACK_DURATION, false, channel);
    disableAudioChain();
    DEBUG("Done - Playing sine tone\n");

    return true;
}

bool audio_get_headset_tf(float data[FFTSIZE / 2], unsigned curve_id)
{
    PanicFalse(data != NULL);
    PanicFalse(curve_id < 8);

    for(int i = 0; i < (FFTSIZE / 2); i++)
    {
        float point;
        if(curve_id == 0) //OEML-Noise
        {
            point = GetTFPoint(MOEMLSquaredCumul, NoiseSquaredCumul, i);
        }
        else if(curve_id == 1) //OEMR-Noise
        {
            point = GetTFPoint(MOEMRSquaredCumul, NoiseSquaredCumul, i);
        }
        else if(curve_id == 2) //IEML-Noise
        {
            point = GetTFPoint(MIEMLSquaredCumul, NoiseSquaredCumul, i);
        }
        else if(curve_id == 3) //IEMR-Noise
        {
            point = GetTFPoint(MIEMRSquaredCumul, NoiseSquaredCumul, i);
        }
        else
        {
            return false;
        }
        data[i] = point;
    }

    return true;
}

bool audio_get_current_mic_rms(float *measured_rms_oem_l, float *measured_rms_oem_r, float *measured_rms_iem_l,
                               float *measured_rms_iem_r)
{
    PanicFalse(measured_rms_oem_l != NULL);
    PanicFalse(measured_rms_oem_r != NULL);
    PanicFalse(measured_rms_iem_l != NULL);
    PanicFalse(measured_rms_iem_r != NULL);

    if(!AudioAnalyzeRMS_OEM_L.available())
        return false;
    if(!AudioAnalyzeRMS_OEM_R.available())
        return false;
    if(!AudioAnalyzeRMS_IEM_L.available())
        return false;
    if(!AudioAnalyzeRMS_IEM_R.available())
        return false;

    *measured_rms_oem_l = amplitude2dB(AudioAnalyzeRMS_OEM_L.read());
    *measured_rms_oem_r = amplitude2dB(AudioAnalyzeRMS_OEM_R.read());
    *measured_rms_iem_l = amplitude2dB(AudioAnalyzeRMS_IEM_L.read());
    *measured_rms_iem_r = amplitude2dB(AudioAnalyzeRMS_IEM_R.read());

    return true;
}

void audio_enable_interrupts(void)
{
    AudioInterrupts();
}

void audio_disable_interrupts(void)
{
    AudioNoInterrupts();
}

bool audio_is_headset_connected(void)
{
    return audio_headset_connected;
}

void audio_set_headset_connected(bool headset_connected)
{
    audio_headset_connected = headset_connected;
}

bool audio_is_headset_eeprom_alive(void)
{
    return audio_headset_eeprom_alive;
}

void audio_set_headset_eeprom_alive(bool headset_eeprom_alive)
{
    audio_headset_eeprom_alive = headset_eeprom_alive;
}

void audio_initialise(void)
{
    PanicFalse(AUDIO_BLOCK_SAMPLES == 128);
    PanicFalse(SAMPLE_RATE == 44100);
    PanicFalse(FFTSIZE == 1024);

    //Audio connections require memory to work.  For more detailed information, see the MemoryAndCpuUsage example
    AudioMemory(120);

    //First audio shield
    AudioControlSGTL5000_1.setAddress(LOW);
    AudioControlSGTL5000_1.enable();
    AudioControlSGTL5000_1.inputSelect(AUDIO_INPUT_LINEIN);
    AudioControlSGTL5000_1.volume(0.7);

    //Second audio shield
    AudioControlSGTL5000_2.setAddress(HIGH);
    AudioControlSGTL5000_2.enable();
    AudioControlSGTL5000_2.inputSelect(AUDIO_INPUT_LINEIN);
    AudioControlSGTL5000_2.volume(0.7);

    pink_noise_amplitude(1.0f);
    AudioSynthWaveformSine_1.amplitude(1.0);
    AudioSynthWaveformSine_1.frequency(4000);
}
