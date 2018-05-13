#!/bin/bash
cd scripts
openocd -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg
