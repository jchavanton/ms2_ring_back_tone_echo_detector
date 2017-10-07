//
// Copyright © 2016 Julien Chavanton
//

#ifdef HAVE_CONFIG_H
#include "mediastreamer-config.h"
#endif

#include "mediastreamer2/msticker.h"

#include <math.h>

#define SIG_DUMP 0
#ifdef ANDROID
#define SIG_DUMP_PREFIX "/sdcard"
#else
#define SIG_DUMP 0
#endif

#define PI 3.14159265358979323846
#define MAX_CORRELATION_SAMPLES 10000
#define SOUND_CARD_MIN_LATENCY 64
#define REF_TONE 0
#define READ_TONE 1
#define TONE_440HZ 0
#define TONE_480HZ 1

typedef struct correlation_variable {
	int val[MAX_CORRELATION_SAMPLES];
	float sum;
	float std;
	float avg;
} correlation_variable_t;

typedef struct correlation {
	correlation_variable_t var[2];
	int count;
	float sum_cross_deviation;
	float avg_sum_cross_deviation;
} correlation_t;

static float get_corr_coeff(correlation_t *c) {
	int count = c->count;
	int i=0;
	float corr_coef;
	if (count == 0)
		return 0.0f;
	c->var[REF_TONE].avg = c->var[REF_TONE].sum / count;
	c->var[READ_TONE].avg = c->var[READ_TONE].sum / count;
	c->sum_cross_deviation = 0;
	c->var[REF_TONE].std = 0;
	c->var[READ_TONE].std = 0;
	for (i=0;i<count;i++) {
		c->var[REF_TONE].std += pow(c->var[REF_TONE].val[i] - c->var[REF_TONE].avg, 2);
		c->var[READ_TONE].std += pow(c->var[READ_TONE].val[i] - c->var[READ_TONE].avg, 2);
		c->sum_cross_deviation += (c->var[REF_TONE].val[i]-c->var[REF_TONE].avg) * (c->var[READ_TONE].val[i]-c->var[READ_TONE].avg);
	}
	c->var[REF_TONE].std = sqrt(c->var[REF_TONE].std/count);
	c->var[READ_TONE].std = sqrt(c->var[READ_TONE].std/count);
	ms_message("sum[%.3f|%.3f]avg[%.3f|%.3f]std[%.3f|%.3f] [%.2f/%d]",
               c->var[REF_TONE].sum, c->var[READ_TONE].sum, c->var[REF_TONE].avg, c->var[READ_TONE].avg, c->var[REF_TONE].std, c->var[READ_TONE].std, c->sum_cross_deviation, count);
	c->avg_sum_cross_deviation = c->sum_cross_deviation / count;
	if (c->var[REF_TONE].std == 0 || c->var[READ_TONE].std == 0)
		return 0.0f;
	corr_coef = c->avg_sum_cross_deviation / (c->var[REF_TONE].std * c->var[READ_TONE].std);
	return corr_coef;
}

typedef struct goertzel_state {
	float Q2;
	float Q1;
	float sine;
	float cosine;
	float coeff;
	float sampling_rate;
	int block_size;
	int block_processed_samples;
	float target_hz;
	float last_block_real_part;
	float last_block_imag_part;
	int block_processed;
} goertzel_state_t;

/* Call this routine before every "block" (size=N) of samples. */
static void goertzel_reset(goertzel_state_t* g) {
	g->Q2 = 0;
	g->Q1 = 0;
	g->block_processed_samples = 0;
}

/* Call this once, to precompute the constants. */
static void goertzel_init(goertzel_state_t* g, float target_hz, int block_size, float sampling_rate) {
	int k;
	float float_n;
	float omega;
	g->sampling_rate = sampling_rate;
	g->block_size = block_size;
	g->target_hz = target_hz;
	float_n = (float) block_size;
	k = (int) (0.5 + ((float_n * g->target_hz) / g->sampling_rate));
	omega = (2.0 * PI * k) / float_n;
	g->sine = sin(omega);
	g->cosine = cos(omega);
	g->coeff = 2.0 * g->cosine;
	g->block_processed=0;
	goertzel_reset(g);
}

/* Call this routine after every block to get the complex result. */
static void goertzel_get_complex(goertzel_state_t* g) {
	g->last_block_real_part = (g->Q1 - g->Q2 * g->cosine);
	g->last_block_imag_part = (g->Q2 * g->sine);
}

static void update_corr_reference(goertzel_state_t* g, correlation_t* c, int delay_offset) {
	int block_processed = 0;
	int sum=0;
	int i=0;
	for (i = 0 ; i < g->block_processed; i++) {
		int ms, ms_delayed;
		int reference = 1000;
		block_processed++;
		ms = block_processed*g->block_size/(int)(g->sampling_rate/1000);
		ms_delayed = ms - delay_offset;

		if (ms_delayed%5000 > 2000) {
			reference = 0;
		}
		sum += reference;
		c[TONE_440HZ].var[REF_TONE].val[i] = reference;
		c[TONE_480HZ].var[REF_TONE].val[i] = reference;
	}
	c[TONE_440HZ].var[REF_TONE].sum = sum;
	c[TONE_480HZ].var[REF_TONE].sum = sum;
}

static int goertzel_block_check(goertzel_state_t* g, correlation_t* c) {
	if (g->block_processed_samples == g->block_size) {
		float magnitude_squared, magnitude, db;

		goertzel_get_complex(g);
		goertzel_reset(g);
		g->block_processed++;
		magnitude_squared = g->last_block_real_part*g->last_block_real_part + g->last_block_imag_part*g->last_block_imag_part;
		magnitude = sqrt(magnitude_squared);

		db = 10*log10f(magnitude/32768);
		ms_debug("goertzel_block_check[%d] [%fHz] mag[%0.0f] corr:dB[%0.2f]", g->block_processed, g->target_hz, magnitude, db);
		if (g->block_processed <= MAX_CORRELATION_SAMPLES) {
			c->var[READ_TONE].val[c->count] = (int)(db*100);
			c->var[READ_TONE].sum += (int)(db*100);
			c->count++;
			return 1;
		}
	} else {
		g->block_processed_samples++;
	}
	return 0;
}

/* Call this routine for every sample. */
static void goertzel_process_sample(goertzel_state_t* g, uint16_t sample) {
	float Q0;
	Q0 = g->coeff * g->Q1 - g->Q2 + (float) sample;
	g->Q2 = g->Q1;
	g->Q1 = Q0;
}

typedef struct detector_state {
	float target_fz[2]; // the two target Fz will that we will search in the signal
	correlation_t c[2];  // the correlation data between each frequency and the reference signal
	goertzel_state_t g[2]; // the goertzel analysis for each frequency, currently 440Hz and 480Hz
	MSBufferizer *buf;
	int rate;
	int framesize;
	int frame_ms;
#if SIG_DUMP == 1
	FILE *recfile;
#endif
} detector_state_t;

static void detector_init(MSFilter *f) {
	int goertzel_block_size = 256; // 32 ms //  8000/1000 8*40 = 320
	detector_state_t *s=ms_new0(detector_state_t,1);
	s->buf=ms_bufferizer_new();
	s->rate=8000;
	s->target_fz[0] = 480.0f; // US 480Hz
	s->target_fz[1] = 440.0f; // US 440Hz
	memset(&s->c[TONE_440HZ],0,sizeof(correlation_t));
	memset(&s->c[TONE_480HZ],0,sizeof(correlation_t));
	goertzel_init(&s->g[0], s->target_fz[0], goertzel_block_size, s->rate);
	goertzel_init(&s->g[1], s->target_fz[1], goertzel_block_size, s->rate);
	s->frame_ms=20;
	s->framesize=2*(s->frame_ms*s->rate)/1000;
	f->data=s;
#if SIG_DUMP == 1
{
	char *fname = ms_strdup_printf("%s/ringbacktone-detector-%p.raw", SIG_DUMP_PREFIX, f);
	s->recfile = fopen(fname, "w");
	if(s->recfile) {
		ms_message("tone detector recording[%s][%p]", fname, s->recfile);
	} else {
		ms_message("tone detector error opening file recording[%s][%p]", fname, s->recfile);
	}
	ms_free(fname);
}
#endif
}

static void detector_uninit(MSFilter *f){
	detector_state_t *s=(detector_state_t *)f->data;
#if SIG_DUMP == 1
	if(s->recfile)
		fclose(s->recfile);
	ms_message("tone detector recording file closed");
#endif
	ms_bufferizer_destroy (s->buf);
	ms_free(f->data);
}

static int detector_set_rate(MSFilter *f, void *arg){
	detector_state_t *s=(detector_state_t *)f->data;
	s->rate = *((int*) arg);
	return 0;
}

static int get_result(MSFilter *f, void *arg){
	detector_state_t *s=(detector_state_t *)f->data;
	float best_corr_coef=-1.0f;
	float corr_coef_1=0.0f;
	float corr_coef_2=0.0f;
	int delay_adj = SOUND_CARD_MIN_LATENCY; // we start with 64ms, this is considered the lowest delay echo delay (between speaker and mic)
	int delay_inc = s->g->block_size / (s->g->sampling_rate/1000); // delay increment in ms is equivalent to the goertzel block size
	rbt_rbt_echo_detection_data_t *data = (rbt_rbt_echo_detection_data_t *) arg;
	
	int echo_delay_ms;
	for(delay_adj=SOUND_CARD_MIN_LATENCY; delay_adj< 500; delay_adj += delay_inc) {
		update_corr_reference(s->g, s->c, delay_adj);
		corr_coef_1 = get_corr_coeff(&s->c[TONE_440HZ]);
		corr_coef_2 = get_corr_coeff(&s->c[TONE_480HZ]);
		ms_message("get_corr_coef %dms[+%dms] best[%.3f] [%.3f|%.3f]", delay_adj, delay_inc, best_corr_coef, corr_coef_1, corr_coef_2);
		if ( best_corr_coef < MIN(corr_coef_1, corr_coef_2)) {
			best_corr_coef = MIN(corr_coef_1,corr_coef_2);
			echo_delay_ms =delay_adj;
		} 
	}

	ms_message("get_corr_coef result[%.3f] echo_delay[%dms]", best_corr_coef, delay_adj);
	data->duration_ms = s->g->block_size / (s->g->sampling_rate/1000) * s->g->block_processed;
	data->echo_delay_ms = echo_delay_ms;
	data->corr_coef = best_corr_coef; // We return the worst one to minize false positif echo detection
	return 0;
}

static void detector_process(MSFilter *f){
	detector_state_t *s=(detector_state_t *)f->data;
	mblk_t *m;
	uint8_t *buf=alloca(s->framesize);

	while ((m=ms_queue_get(f->inputs[0]))!=NULL){
		ms_queue_put(f->outputs[0],m);
		ms_bufferizer_put(s->buf,dupmsg(m));
	}
	while(ms_bufferizer_read(s->buf,buf,s->framesize)!=0){
		int i=0;
		int16_t* samples = (int16_t*) buf;
	#if SIG_DUMP == 1
		if(s->recfile)
			fwrite(buf,s->framesize,1,s->recfile);
	#endif
		for (i=0;i<s->framesize/2;i++) {
			goertzel_process_sample(&s->g[0], *samples);
			goertzel_process_sample(&s->g[1], *samples);
			goertzel_block_check(&s->g[0], &s->c[TONE_440HZ]);
			goertzel_block_check(&s->g[1], &s->c[TONE_480HZ]);
			samples++;
		}
	}
}

static MSFilterMethod detector_methods[]={
	{	MS_FILTER_SET_SAMPLE_RATE,	detector_set_rate	},
	{	RBT_MS_FILTER_GET_RESULT,	get_result	},
	{	0,	NULL}
};

#ifndef _MSC_VER

MSFilterDesc ms_rbt_tone_detector_desc={
	.id=RBT_TONE_DETECTOR_ID,
	.name="RBT MSToneDetector",
	.text="RBT Custom tone detection filter.",
	.category=MS_FILTER_OTHER,
	.ninputs=1,
	.noutputs=1,
	.init=detector_init,
	.process=detector_process,
	.uninit=detector_uninit,
	.methods=detector_methods
};

#else

MSFilterDesc ms_rbt_tone_detector_desc={
	RBT_TONE_DETECTOR_ID,
	"RBT MSToneDetector",
	"RBT Custom tone detection filter.",
	MS_FILTER_OTHER,
	NULL,
	1,
	1,
	detector_init,
	NULL,
	detector_process,
	NULL,
	detector_uninit,
	detector_methods,
};

#endif

MS_FILTER_DESC_EXPORT(ms_rbt_tone_detector_desc)
