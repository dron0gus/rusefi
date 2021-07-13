/*
 * @file test_real_nb2_cranking.cpp
 *
 * @date July 13, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */


#include "engine_test_helper.h"
#include "logicdata_csv_reader.h"

TEST(crankingNB2, nb2RealCrankingFromFile) {
	CsvReader reader(1, /* vvtCount */ 1);
	int indeces[1] = {0};

	reader.open("tests/trigger/resources/nb2_rev-d-4.csv", indeces);
	WITH_ENGINE_TEST_HELPER (ET_HELLEN_NB2);

	while (eth.getTimeNowUs() < 3'028'987) {
		reader.processLine(&eth);
		ASSERT_EQ(0, GET_RPM()) << "At line " << reader.lineIndex() << " time " << eth.getTimeNowUs();
	}
	ASSERT_EQ(243, GET_RPM()) << "At line " << reader.lineIndex() << " time " << eth.getTimeNowUs();


}