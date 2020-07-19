#!/bin/bash

echo "This script reads rusefi_config.txt and produces firmware persistent configuration headers"
echo "The storage section of rusefi.ini is updated as well"

rm -f gen_config.log
rm -f gen_config_board.log

bash gen_config_default.sh
[ $? -eq 0 ] || { echo "ERROR generating default"; exit 1; }

echo "This would automatically copy latest file to 'dev' TS projects to ${TS_PATH}"
[ -d $TS_PATH/dev/projectCfg/ ] || mkdir -p $TS_PATH/dev/projectCfg/
cp -v tunerstudio/generated/rusefi.ini $TS_PATH/dev/projectCfg/mainController.ini
[ -d $TS_PATH/mre_f4/projectCfg/ ] || mkdir -p $TS_PATH/mre_f4/projectCfg/
cp -v tunerstudio/generated/rusefi_mre_f4.ini $TS_PATH/mre_f4/projectCfg/mainController.ini

#
# see also build-firmware where we compile all versions of firmware
#
# While adding a new board do not forget to manually git add/commit .h and .ini into
# firmware\tunerstudio\generated and firmware\controllers\generated folders
# maybe one day we will automate but not yet
#
for BOARD in "f429-discovery f429-discovery" "hellen/hellen128 hellen128" "hellen/hellen121vag hellen121vag" "hellen/hellen121nissan hellen121nissan" "hellen/hellen72 hellen72" "hellen/hellen64_miataNA6_94 hellenNA6" "microrusefi mre_f7" "microrusefi mre_f4" "frankenso frankenso_na6" "prometheus prometheus_469" "prometheus prometheus_405" "proteus proteus_f7" "proteus proteus_f4"; do
 BOARD_NAME="${BOARD% *}"
 BOARD_SHORT_NAME="${BOARD#* }"
 echo "Generating for ${BOARD_NAME} (${BOARD_SHORT_NAME})"
 bash gen_config_board.sh $BOARD_NAME $BOARD_SHORT_NAME >> /dev/null
 [ $? -eq 0 ] || { echo "ERROR generating board $BOARD_NAME $BOARD_SHORT_NAME"; exit 1; }
done

[ -d $TS_PATH/subaru_eg33_f7/projectCfg/ ] || mkdir -p $TS_PATH/subaru_eg33_f7/projectCfg/
cp -v tunerstudio/generated/rusefi_subaru_eg33_f7.ini $TS_PATH/subaru_eg33_f7/projectCfg/mainController.ini

cd config/boards/kinetis/config
bash gen_config.sh >> /dev/null
[ $? -eq 0 ] || { echo "ERROR generating board kinetis kin"; exit 1; }


cd ../../../..
cd config/boards/hellen/cypress/config
bash gen_config.sh
[ $? -eq 0 ] || { echo "ERROR generating board hellen_cypress hellen_cypress"; exit 1; }
cd ../../../../..

bash config/boards/subaru_eg33/config/gen_config.sh
[ $? -eq 0 ] || { echo "ERROR generating board subaru_eg33 subaru_eg33_f7"; exit 1; }

exit 0
