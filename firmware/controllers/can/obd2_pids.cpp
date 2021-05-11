/*
 * @file obd2_pids.cpp
 *
 * ISO 15765-4
 * http://en.wikipedia.org/wiki/OBD-II_PIDs
 *
 * @date May 14, 2019
 * @author Andrey Gusakov, (c) 2019
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "global.h"

#if EFI_CAN_SUPPORT

#include "unaligned.h"
#include "common.h"

#include "engine.h"
#include "obd2.h"
#include "can_hw.h"
#include "vehicle_speed.h"
#include "map.h"
#include "maf.h"
#include "tps.h"
#include "sensor.h"
#include "engine_math.h"
#include "fuel_math.h"

#define DEBUG

/*==========================================================================*/
/* Extern definitions.														*/
/*==========================================================================*/

EXTERN_ENGINE
;

/*==========================================================================*/
/* Local definitions.														*/
/*==========================================================================*/

/*==========================================================================*/
/* Local types.												*/
/*==========================================================================*/

struct obd_pid_handler_desc {
	uint8_t 		pid;
	uint8_t 		reply_size;
#ifdef DEBUG
	const char 		*desc;
#endif
	int				(*handler)(struct obd_pid_handler_desc* desc, uint8_t *data);
};


#ifdef DEBUG
	#define PID(p, s, d, h)	{.pid = p, .reply_size = s, .desc = d, .handler = obd_get_##h}
#else
	#define PID(p, s, d, h)	{.pid = p, .reply_size = s, .handler = obd_get_##h}
#endif

#define PID_S(p, s, h)	PID(p, s, #h, h)

/*==========================================================================*/
/* Forvard declarations														*/
/*==========================================================================*/

static int obd_get_supported_pids(struct obd_pid_handler_desc* desc, uint8_t *data);

/*==========================================================================*/
/* Local functions.															*/
/*==========================================================================*/

static void odb_clamp_and_store_u8(uint8_t *data, float value)
{
	/* TODO: do we really need this rounding? */
	int iValue = (int)efiRound(value, 1.0f);

	put_be8(data, CLAMP(iValue, 0, 255));
}

static void odb_clamp_and_store_u16(uint8_t *data, float value)
{
	/* TODO: do we really need this rounding? */
	int iValue = (int)efiRound(value, 1.0f);

	put_be16(data, CLAMP(iValue, 0, 65535));
}

static int obd_get_monitor_status(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	/* TODO: add real status */
	put_be32(data, 0);

	return desc->reply_size;
}

static int obd_get_fuel_system_status(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	/* TODO: report real value */
	/* 2 = "Closed loop, using oxygen sensor feedback to determine fuel mix" */
	/* System #1 */
	data[0] = 2;
	/* System #2 */
	data[1] = 0;

	return desc->reply_size;
}

static int obd_get_engine_load(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	//odb_clamp_and_store_u8(data, getEngineLoadT(PASS_ENGINE_PARAMETER_SIGNATURE) * 2.55f);

	return desc->reply_size;
}

static int obd_get_coolant_temp(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	//odb_clamp_and_store_u8(data, engine->sensors.clt + 40.0f);

	return desc->reply_size;
}

static int obd_get_map(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	odb_clamp_and_store_u8(data, getMap());

	return desc->reply_size;
}

static int obd_get_rpm(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	odb_clamp_and_store_u16(data, GET_RPM() * 4.0f);

	return desc->reply_size;
}

static int obd_get_speed(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	odb_clamp_and_store_u8(data, getVehicleSpeed());

	return desc->reply_size;
}

static int obd_get_timing_advance(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	float timing = engine->engineState.timingAdvance;

	timing = (timing > 360.0f) ? (timing - 720.0f) : timing;
	/* angle before TDC.	(A/2)-64 */
	odb_clamp_and_store_u8(data, (timing + 64.0f) * 2.0f);

	return desc->reply_size;
}

static int obd_get_iat(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	odb_clamp_and_store_u8(data, Sensor::get(SensorType::Iat).value_or(0) + 40.0f);

	return desc->reply_size;
}

static int obd_get_maf(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	/* convert from kg/h to g/sec */
	//odb_clamp_and_store_u16(data, getRealMaf(PASS_ENGINE_PARAMETER_SIGNATURE) * 1000.0 / 60.0 / 60.0 * 100.0f);

	return desc->reply_size;
}

static int obd_get_throttle(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	//odb_clamp_and_store_u8(data, getTPS(PASS_ENGINE_PARAMETER_SIGNATURE) * 2.55f);

	return desc->reply_size;
}

static int obd_get_fuel_rate(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	odb_clamp_and_store_u16(data, engine->engineState.fuelConsumption.getConsumptionGramPerSecond() * 20.0f);

	return desc->reply_size;
}

/*==========================================================================*/
/* Local variables															*/
/*==========================================================================*/

static const struct obd_pid_handler_desc obd_pids[] = {

	/*	ID		Data bytes returned		Description		Min value..Max value Units */
	/*	0x00	4	PIDs supported [01 - 20] */
	PID_S(0x00,	4,	supported_pids),
	/*	0x01	4	Monitor status since DTCs cleared. (Includes malfunction indicator lamp (MIL) status and number of DTCs.) */
	PID_S(0x01,	4,	monitor_status),
//	0x02	2	Freeze DTC
	/*	0x03	2	Fuel system status */
	PID_S(0x03,	2,	fuel_system_status),
	/*	0x04	1	Calculated engine load 	(0..100% to 0..255) */
	PID_S(0x04,	1,	engine_load),
	/*	0x05	1	Engine coolant temperature 	-40	215	°C */
	PID_S(0x05,	1,	coolant_temp),
//	0x06	1	Short term fuel trim--Bank 1 	-100 (Reduce Fuel: Too Rich) 	99.2 (Add Fuel: Too Lean) 	%
//	0x07	1	Long term fuel trim--Bank 1
//	0x08	1	Short term fuel trim--Bank 2
//	0x09	1	Long term fuel trim--Bank 2
//	0x0A 	1	Fuel pressure (gauge pressure) 	0	765	kPa
	/*	0x0B 	1	Intake manifold absolute pressure 	0	255	kPa */
	PID_S(0x0B,	1,	map),
	/*	0x0C 	2	Engine RPM 	0..16383.75	rpm */
	PID_S(0x0C,	2,	rpm),
	/*	0x0D 	1	Vehicle speed 	0..255	km/h */
	PID_S(0x0D,	1,	speed),
	/*	0x0E 	1	Timing advance 	-64..63.5 before TDC */
	PID_S(0x0E,	1,	timing_advance),
	/*	0x0F 	1	Intake air temperature 	-40..215	C */
	PID_S(0x0F,	1,	iat),
	/*	0x10	2	MAF air flow rate 	0..655.35	grams/sec */
	PID_S(0x10,	2,	maf),
	/*	0x11	1	Throttle position 	0..100	% */
	PID_S(0x11,	1,	throttle),
//	0x12	1	Commanded secondary air status
//	0x13	1	Oxygen sensors present (in 2 banks)
//	0x14	2	"Oxygen Sensor 1 A: Voltage B: Short term fuel trim"
//	0x15	2	"Oxygen Sensor 2 A: Voltage B: Short term fuel trim "
//	0x16	2	"Oxygen Sensor 3 A: Voltage B: Short term fuel trim "
//	0x17	2	"Oxygen Sensor 4 A: Voltage B: Short term fuel trim "
//	0x18	2	"Oxygen Sensor 5 A: Voltage B: Short term fuel trim "
//	0x19	2	"Oxygen Sensor 6 A: Voltage B: Short term fuel trim "
//	0x1A 	2	"Oxygen Sensor 7 A: Voltage B: Short term fuel trim "
//	0x1B 	2	"Oxygen Sensor 8 A: Voltage B: Short term fuel trim "
//	0x1C 	1	OBD standards this vehicle conforms to
//	0x1D 	1	Oxygen sensors present (in 4 banks)
//	0x1E 	1	Auxiliary input status
//	0x1F 	2	Run time since engine start 	0	65535	seconds
	/*	0x20	4	PIDs supported [21 - 40] */
	PID_S(0x20,	4,	supported_pids),
//	0x21	2	Distance traveled with malfunction indicator lamp (MIL) on 	0	65535	km
//	0x22	2	Fuel Rail Pressure (relative to manifold vacuum) 	0	5177.265	kPa
//	0x23	2	Fuel Rail Gauge Pressure (diesel, or gasoline direct injection) 	0	655350	kPa
//	0x24	4	"Oxygen Sensor 1 AB: Fuel-Air Equivalence Ratio  CD: Voltage"
//	0x25	4	"Oxygen Sensor 2 AB: Fuel-Air Equivalence Ratio CD: Voltage "
//	0x26	4	"Oxygen Sensor 3 AB: Fuel-Air Equivalence Ratio CD: Voltage "
//	0x27	4	"Oxygen Sensor 4 AB: Fuel-Air Equivalence Ratio CD: Voltage "
//	0x28	4	"Oxygen Sensor 5 AB: Fuel-Air Equivalence Ratio CD: Voltage "
//	0x29	4	"Oxygen Sensor 6 AB: Fuel-Air Equivalence Ratio CD: Voltage "
//	0x2A 	4	"Oxygen Sensor 7 AB: Fuel-Air Equivalence Ratio CD: Voltage "
//	0x2B 	4	"Oxygen Sensor 8 AB: Fuel-Air Equivalence Ratio CD: Voltage "
//	0x2C 	1	Commanded EGR 	0	100	%
//	0x2D 	1	EGR Error 	-100	99.2	%
//	0x2E 	1	Commanded evaporative purge 	0	100	%
//	0x2F 	1	Fuel Tank Level Input 	0	100	%
//	0x30	1	Warm-ups since codes cleared 	0	255	count
//	0x31	2	Distance traveled since codes cleared 	0	65535	km
//	0x32	2	Evap. System Vapor Pressure 	-8192	8191.75	Pa
//	0x33	1	Absolute Barometric Pressure 	0	255	kPa
//	0x34	4	"Oxygen Sensor 1 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x35	4	"Oxygen Sensor 2 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x36	4	"Oxygen Sensor 3 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x37	4	"Oxygen Sensor 4 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x38	4	"Oxygen Sensor 5 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x39	4	"Oxygen Sensor 6 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x3A 	4	"Oxygen Sensor 7 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x3B 	4	"Oxygen Sensor 8 AB: Fuel-Air Equivalence Ratio CD: Current "
//	0x3C 	2	Catalyst Temperature: Bank 1, Sensor 1 	-40	6513.5	°C
//	0x3D 	2	Catalyst Temperature: Bank 2, Sensor 1
//	0x3E 	2	Catalyst Temperature: Bank 1, Sensor 2
//	0x3F 	2	Catalyst Temperature: Bank 2, Sensor 2
	/*	0x40	4	PIDs supported [41 - 60] */
	PID_S(0x40,	4,	supported_pids),
//	0x41	4	Monitor status this drive cycle
//	0x42	2	Control module voltage 	0	65.535	V
//	0x43	2	Absolute load value 	0	25700	%
//	0x44	2	Fuel-Air commanded equivalence ratio 	0	< 2 	ratio
//	0x45	1	Relative throttle position 	0	100	%
//	0x46	1	Ambient air temperature 	-40	215	°C
//	0x47	1	Absolute throttle position B 	0	100	%
//	0x48	1	Absolute throttle position C
//	0x49	1	Accelerator pedal position D
//	0x4A 	1	Accelerator pedal position E
//	0x4B 	1	Accelerator pedal position F
//	0x4C 	1	Commanded throttle actuator
//	0x4D 	2	Time run with MIL on 	0	65535	minutes
//	0x4E 	2	Time since trouble codes cleared
//	0x4F 	4	Maximum value for Fuel-Air equivalence ratio, oxygen sensor voltage, oxygen sensor current, and intake manifold absolute pressure 	0, 0, 0, 0 	255, 255, 255, 2550 	ratio, V, mA, kPa
//	0x50	4	Maximum value for air flow rate from mass air flow sensor 	0	2550	g/s
//	0x51	1	Fuel Type
//	0x52	1	Ethanol fuel % 	0	100	%
//	0x53	2	Absolute Evap system Vapor Pressure 	0	327.675	kPa
//	0x54	2	Evap system vapor pressure 	-32767	32768	Pa
//	0x55	2	Short term secondary oxygen sensor trim, A: bank 1, B: bank 3 	-100	99.2	%
//	0x56	2	Long term secondary oxygen sensor trim, A: bank 1, B: bank 3
//	0x57	2	Short term secondary oxygen sensor trim, A: bank 2, B: bank 4
//	0x58	2	Long term secondary oxygen sensor trim, A: bank 2, B: bank 4
//	0x59	2	Fuel rail absolute pressure 	0	655350	kPa
//	0x5A 	1	Relative accelerator pedal position 	0	100	%
//	0x5B 	1	Hybrid battery pack remaining life 	0	100	%
//	0x5C 	1	Engine oil temperature 	-40	210	°C
//	0x5D 	2	Fuel injection timing 	-210	301.992	°
	/*	0x5E 	2	Engine fuel rate 	0	3212.75	L/h */
	PID_S(0x5e,	2,	fuel_rate),
//	0x5F 	1	Emission requirements to which vehicle is designed
	/*	0x60	4	PIDs supported [61 - 80] */
	PID_S(0x60,	4,	supported_pids),
//	0x61	1	Drivers demand engine - percent torque 	-125	130	%
//	0x62	1	Actual engine - percent torque 	-125	130	%
//	0x63	2	Engine reference torque 	0	65535	Nm
//	0x64	5	Engine percent torque data 	-125	130	%
//	0x65	2	Auxiliary input / output supported
//	0x66	5	Mass air flow sensor
//	0x67	3	Engine coolant temperature
//	0x68	7	Intake air temperature sensor
//	0x69	7	Commanded EGR and EGR Error
//	0x6A 	5	Commanded Diesel intake air flow control and relative intake air flow position
//	0x6B 	5	Exhaust gas recirculation temperature
//	0x6C 	5	Commanded throttle actuator control and relative throttle position
//	0x6D 	6	Fuel pressure control system
//	0x6E 	5	Injection pressure control system
//	0x6F 	3	Turbocharger compressor inlet pressure
//	0x70	9	Boost pressure control
//	0x71	5	Variable Geometry turbo (VGT) control
//	0x72	5	Wastegate control
//	0x73	5	Exhaust pressure
//	0x74	5	Turbocharger RPM
//	0x75	7	Turbocharger temperature
//	0x76	7	Turbocharger temperature
//	0x77	5	Charge air cooler temperature (CACT)
//	0x78	9	Exhaust Gas temperature (EGT) Bank 1
//	0x79	9	Exhaust Gas temperature (EGT) Bank 2
//	0x7A 	7	Diesel particulate filter (DPF)
//	0x7B 	7	Diesel particulate filter (DPF)
//	0x7C 	9	Diesel Particulate filter (DPF) temperature
//	0x7D 	1	NOx NTE (Not-To-Exceed) control area status
//	0x7E 	1	PM NTE (Not-To-Exceed) control area status
//	0x7F 	13	Engine run time
	/*	0x80	4	PIDs supported [81 - A0] */
	PID_S(0x80,	4,	supported_pids),
//	0x81	21	Engine run time for Auxiliary Emissions Control Device(AECD)
//	0x82	21	Engine run time for Auxiliary Emissions Control Device(AECD)
//	0x83	5	NOx sensor
//	0x84	1	Manifold surface temperature
//	0x85	10	NOx reagent system
//	0x86	5	Particulate matter (PM) sensor
//	0x87	5	Intake manifold absolute pressure
//	0x88	13	SCR Induce System
//	0x89	41	Run Time for AECD #11-#15
//	0x8A 	41	Run Time for AECD #16-#20
//	0x8B 	7	Diesel Aftertreatment
//	0x8C 	16	O2 Sensor (Wide Range)
//	0x8D 	1	Throttle Position G 	0	100	%
//	0x8E 	1	Engine Friction - Percent Torque 	-125	130	%
//	0x8F 	5	PM Sensor Bank 1 & 2
//	0x90	3	WWH-OBD Vehicle OBD System Information 			hours
//	0x91	5	WWH-OBD Vehicle OBD System Information 			hours
//	0x92	2	Fuel System Control
//	0x93	3	WWH-OBD Vehicle OBD Counters support 			hours
//	0x94	12	NOx Warning And Inducement System
//	0x98	9	Exhaust Gas Temperature Sensor
//	0x99	9	Exhaust Gas Temperature Sensor
//	0x9A 	6	Hybrid/EV Vehicle System Data, Battery, Voltage
//	0x9B 	4	Diesel Exhaust Fluid Sensor Data
//	0x9C 	17	O2 Sensor Data
//	0x9D 	4	Engine Fuel Rate 			g/s
//	0x9E 	2	Engine Exhaust Flow Rate 			kg/h
//	0x9F 	9	Fuel System Percentage Use
	/*	0xA0 	4	PIDs supported [A1 - C0] */
	PID_S(0xA0,	4,	supported_pids),
//	0xA1 	9	NOx Sensor Corrected Data 			ppm
//	0xA2 	2	Cylinder Fuel Rate 			mg/stroke
//	0xA3 	9	Evap System Vapor Pressure 			Pa
//	0xA4 	4	Transmission Actual Gear
//	0xA5 	4	Diesel Exhaust Fluid Dosing
//	0xA6 	4	Odometer 			hm
	/*	0xC0 	4	PIDs supported [C1 - E0] */
	PID_S(0xC0,	4,	supported_pids),
//	0xC3 	? 	? 	? 	? 	?
//	0xC4 	? 	? 	? 	? 	?
};

/*==========================================================================*/
/* Local functions.															*/
/*==========================================================================*/

static int obd_get_supported_pids(struct obd_pid_handler_desc* desc, uint8_t *data)
{
	unsigned int i;
	int offset;

	/* pids offset is equal to request pid */
	offset = desc->pid;

	memset(data, 0, 4);

	for (i = 0; i < ARRAY_SIZE(obd_pids); i++) {
		if ((obd_pids[i].pid >= offset) && (obd_pids[i].pid < offset + 0x20))
			bit_set(data, obd_pids[i].pid - offset);

		i++;
	}

	return desc->reply_size;
}

/*==========================================================================*/
/* Exported variables.														*/
/*==========================================================================*/

/*==========================================================================*/
/* Exported functions.														*/
/*==========================================================================*/

#endif /* EFI_CAN_SUPPORT */
