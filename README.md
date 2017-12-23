Vecmocon Technologies Pvt Ltd
@author: Altamash Abdul Rahim
Initial Release: 23-December-2017
 
This Repo contains two Keil projects for SD based IAP.
->The binary project is for creating a binary file for IAP
* Vector table offset is changed in system_stm32f4xx.c according to the APPLICATION_ADDRESS
* Also in the options for target change Read/Only Memory areas : IROM1 start to APPLICATION_ADDRESS.
* Build the project and run the .bat file to create binary file.

-> The storage project contain the IAP Code 
* Choose the APPLICATION_ADDRESS with respect to flash sector and IAP cpde size.
 TODO: Work to be done on the IAP UI development