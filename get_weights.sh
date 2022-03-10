#!/bin/bash

FILE_NAME=weights

wget "https://onedrive.live.com/download?cid=37A95CCBD0552808&resid=37A95CCBD0552808%21118180&authkey=AFtwLLONRiFa03g" -O ${FILE_NAME}.zip
unzip -o ${FILE_NAME}.zip -d src/01_common/hma_yolact/hma_yolact_pkg/io/
rm -rf ${FILE_NAME}.zip