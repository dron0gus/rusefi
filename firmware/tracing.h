#ifndef _TRACING_
#define _TRACING_

#ifdef __cplusplus
extern "C"
{
#endif

struct trace_point_t {
	const char *func;
	int 		line;
	uint32_t	ts;
	uint32_t	counter[2];
};

#define TRACE_POINTS_N	32

extern struct trace_point_t trace_points[TRACE_POINTS_N];
extern int trace_point_counter;

void rusefi_trace_start(void);
void rusefi_trace_stop(void);
void trace_point_reset(int i);

#define TRACEPOINT()											\
	do {														\
		if (trace_point_counter < TRACE_POINTS_N) {				\
			int i = trace_point_counter;						\
			trace_points[i].func = __func__;					\
			trace_points[i].line = __LINE__;					\
			trace_points[i].ts = chSysGetRealtimeCounterX();	\
			trace_point_counter++;								\
			trace_point_reset(trace_point_counter);				\
		}														\
	} while (0)

#define TRACESTART()											\
	trace_point_counter = 0;									\
	rusefi_trace_start();										\
	TRACEPOINT()

#define TRACESTOP()												\
	TRACEPOINT();												\
	rusefi_trace_stop()

#ifdef __cplusplus
}
#endif

#endif
