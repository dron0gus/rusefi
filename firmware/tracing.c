#include "global.h"
#include "os_access.h"

#include "ch.h"
#include "shell.h"

#include "tracing.h"

#define CPU_TICKS_PER_MS		(168000)

BaseSequentialStream *trace_stream = (BaseSequentialStream *)&EFI_SERIAL_SHELL_SD;

static bool tracing_run = false;

static systime_t last_task_switch = 0;
static systime_t last_irq_enter = 0;
static uint32_t last_task_switch_tick = 0;
static uint32_t last_task_enter_tick = 0;

static sysinterval_t max_task_time = 0;
static sysinterval_t max_irq_time = 0;
static uint32_t max_irq_start_tick = 0;

static uint32_t max_task_tick = 0;
static uint32_t max_irq_tick = 0;
static uint32_t max_tast_start_tick = 0;

static const char *long_irq_name;
static thread_t *long_thread;
static thread_t *last_thread;

/* trace points */
struct trace_point_t trace_points[TRACE_POINTS_N];
int trace_point_counter = 0;

void rusefi_trace_hook(void *p)
{
	bool max = false;
	sysinterval_t diff;
	int64_t diff_tick;
	ch_trace_event_t *t = (ch_trace_event_t *)p;

	switch (t->type) {
		case CH_TRACE_TYPE_SWITCH:
			if (last_task_switch) {
				diff_tick = (int64_t)t->rtstamp - (int64_t)last_task_switch_tick;
				if (diff_tick < 0)
					diff_tick += 0x100000000;
				diff = chTimeDiffX(last_task_switch, t->time);
				/* use ticks */
				if (1) {
					if (diff_tick > max_task_tick) {
						max = true;
					}
				} else {
					if (diff > max_task_time)
						max = true;
				}
				if (max) {
					max_task_tick = diff_tick;
					max_task_time = diff;
					/* save pointer to longest thread */
					long_thread = last_thread;
					max_tast_start_tick = last_task_switch_tick;
				}
			}
			last_task_switch = t->time;
			last_task_switch_tick = t->rtstamp;
			last_thread = t->u.sw.ntp;

			if (tracing_run)
				trace_points[trace_point_counter].counter[0]++;
		break;

		case CH_TRACE_TYPE_ISR_ENTER:
			last_irq_enter = t->time;
			last_task_enter_tick = t->rtstamp;

			if (tracing_run)
				trace_points[trace_point_counter].counter[1]++;
		break;

		case CH_TRACE_TYPE_ISR_LEAVE:
				diff_tick = (int64_t)t->rtstamp - (int64_t)last_task_enter_tick;
				if (diff_tick < 0)
					diff_tick += 0x100000000;
				diff = chTimeDiffX(last_irq_enter, t->time);
			if (1) {
				if (diff_tick > max_irq_tick)
					max = true;
			} else {
				if (diff > max_irq_time)
					max = true;
			}
			if (max) {
				max_irq_tick = diff_tick;
				max_irq_time = diff;
				/* save IRQ name */
				long_irq_name = t->u.isr.name;
				max_irq_start_tick = last_task_enter_tick;
			}
		break;

		default:
		break;
	}
}

void rusefi_tracing_stat_clean(void)
{
	max_irq_time = 0; max_irq_tick = 0; max_irq_start_tick = 0;
	max_task_time = 0; max_task_tick = 0; last_task_switch_tick = 0;
}

void trace_point_reset(int i)
{
	memset(&trace_points[i], 0, sizeof(trace_points[0]));
}

void rusefi_tracing_show(BaseSequentialStream *chp)
{
	chprintf(chp, "RusEFI tracing stats:\r\n");

	chprintf(chp, "[%010u] longest irq: %s, %u OS ticks, %u CPU ticks (%4u mS)\r\n",
		max_irq_start_tick, long_irq_name, max_irq_time, max_irq_tick, max_irq_tick / CPU_TICKS_PER_MS);
	chprintf(chp, "[%010u] longest context: %s, %u OS ticks, %u CPU ticks (%4u mS)\r\n",
		last_task_switch_tick, long_thread->name, max_task_time, max_task_tick, max_task_tick / CPU_TICKS_PER_MS);

	if (trace_point_counter) {
		int i;
		uint32_t diff;

		for (i = 0; i < trace_point_counter; i++) {
			if (i != 0)
				diff = trace_points[i].ts - trace_points[i - 1].ts;
			else
				diff = 0;

			chprintf(chp, "[%010u]%02d %s:%d +%09u (%4u mS): sw %d, irq %d\r\n",
				trace_points[i].ts,
				i,
				trace_points[i].func,
				trace_points[i].line,
				diff,
				diff / CPU_TICKS_PER_MS,
				trace_points[i].counter[0],
				trace_points[i].counter[1]);
		}
	}
	chprintf(chp, "RusEFI tracing end\r\n\r\n");
}

void rusefi_trace_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void)argc; (void)argv;

	rusefi_tracing_show(chp);
	rusefi_tracing_stat_clean();
}

extern void setNeedToWriteConfiguration(void);

void rusefi_trace_trigger(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void)argc; (void)argv;

	chprintf(chp, "Calling flash write\r\n");

	setNeedToWriteConfiguration();
}

void rusefi_trace_start(void)
{
	/* reset */
	trace_point_reset(0);
	trace_point_counter = 0;

	rusefi_tracing_stat_clean();
	tracing_run = true;
}

void rusefi_trace_stop(void)
{
	tracing_run = false;
	rusefi_tracing_show(trace_stream);
	//rusefi_tracing_stat_clean();
}