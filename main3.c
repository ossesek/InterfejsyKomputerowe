#include<stdio.h>
#include<wiringPi.h>
#include<wiringPiSPI.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

 int chan = 0;
  int speed = 1000000;
  int t_fine;
  
  enum {
  BMP280_REGISTER_DIG_T1 = 0x88,
  BMP280_REGISTER_DIG_T2 = 0x8A,
  BMP280_REGISTER_DIG_T3 = 0x8C,
  BMP280_REGISTER_DIG_P1 = 0x8E,
  BMP280_REGISTER_DIG_P2 = 0x90,
  BMP280_REGISTER_DIG_P3 = 0x92,
  BMP280_REGISTER_DIG_P4 = 0x94,
  BMP280_REGISTER_DIG_P5 = 0x96,
  BMP280_REGISTER_DIG_P6 = 0x98,
  BMP280_REGISTER_DIG_P7 = 0x9A,
  BMP280_REGISTER_DIG_P8 = 0x9C,
  BMP280_REGISTER_DIG_P9 = 0x9E,
  BMP280_REGISTER_CHIPID = 0xD0,
  BMP280_REGISTER_VERSION = 0xD1,
  BMP280_REGISTER_SOFTRESET = 0xE0,
  BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
  BMP280_REGISTER_STATUS = 0xF3,
  BMP280_REGISTER_CONTROL = 0xF4,
  BMP280_REGISTER_CONFIG = 0xF5,
  BMP280_REGISTER_PRESSUREDATA = 0xF7,
  BMP280_REGISTER_TEMPDATA = 0xFA,
};


typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

bmp280_calib_data calib_data;
  
  
  int writeRegister(int adress,int data) {
	 unsigned char buff[2];
	buff[0]=(adress& ~0x80);
	buff[1]=data;
	wiringPiSPIDataRW(chan, buff, 2);
	return 1;
}


int readRegister(int adress) {
	 unsigned char buff[2];
	buff[0]=(adress | 0x80);
	wiringPiSPIDataRW(chan, buff, 2);
	return buff[1];
}




int init() {
short unsigned wartosc=0;


	//reset
	wartosc=0xb6;
writeRegister(BMP280_REGISTER_SOFTRESET,wartosc);
delay(1000);


	return 0;
}

int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T){
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bmp280_compensate_P_int64(int32_t adc_P){
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
	var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
	if(var1 == 0){
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
	return (uint32_t)p;
}

float readPreassure(){
	int32_t rawPreassure = (readRegister(0xF7) << 12) | (readRegister(0xF8) << 4) | (readRegister(0xF9)); //12 empty bits, 8 bits msb, 8 bits lsb, 4 bits, xlsb
	//printf("raw pressure %ld\r\n", (long int)rawPreassure);
	return bmp280_compensate_P_int64(rawPreassure)/256.0/100.0;
}

float readTemperature(){
	int32_t rawTemperature = (readRegister(0xFA) << 12) | (readRegister(0xFB) << 4) | (readRegister(0xFC)); //12 empty bits, 8 bits msb, 8 bits lsb, 4 bits, xlsb
	//printf("raw temp %ld\r\n", (long int)rawTemperature);
	return bmp280_compensate_T_int32(rawTemperature)/100.0;
}

void loadCalibration(){
	calib_data.dig_P1 = (readRegister(0x8F) << 8 )|(readRegister(0x8E));
	calib_data.dig_P2 = (readRegister(0x91) << 8 )|(readRegister(0x90));
	calib_data.dig_P3 = (readRegister(0x93) << 8 )|(readRegister(0x92));
	calib_data.dig_P4 = (readRegister(0x95) << 8 )|(readRegister(0x94));
	calib_data.dig_P5 = (readRegister(0x97) << 8 )|(readRegister(0x96));
	calib_data.dig_P6 = (readRegister(0x99) << 8 )|(readRegister(0x98));
	calib_data.dig_P7 = (readRegister(0x9B) << 8 )|(readRegister(0x9A));
	calib_data.dig_P8 = (readRegister(0x9D) << 8 )|(readRegister(0x9C));
	calib_data.dig_P9 = (readRegister(0x9F) << 8 )|(readRegister(0x9E));
	calib_data.dig_T1 = (readRegister(0x89) << 8 )|(readRegister(0x88));
	calib_data.dig_T2 = (readRegister(0x8B) << 8 )|(readRegister(0x8A));
	calib_data.dig_T3 = (readRegister(0x8D) << 8 )|(readRegister(0x8C));
}

int main()
{
   
  

    if (wiringPiSPISetup(chan, speed) == -1)
    {
        printf("Could not initialise SPI\n");
        return 0;
    }
    
	init();
	delay(600);

    
     
		
		int id=readRegister(0x90);

	
		
printf("Czujnik BMP280 - ChipID: %x\r\n",id);
//konfiguracja oversamplingu, trybu, opóźnienia odczytów
//int config = readRegister(BMP280_REGISTER_CONFIG);

//writeRegister(BMP280_REGISTER_CONFIG, (config | 0x1F)); //ustawienie 000 na poczatku config, co daje t_sb = 0.5 ms bez zmiany reszty rejestru
writeRegister(BMP280_REGISTER_CONTROL, 0x57); //010 101 11 => 2x temp oversampling, 16x preassure oversampling, mode normal
int config = readRegister(BMP280_REGISTER_CONTROL);
printf("config %x\n", config);
//załadowanie danych kalibracji
loadCalibration();

//odczyt
delay(3000);
float temp = readTemperature();
float press = readPreassure();
printf("Temperatura: %f C,  Cisnienie: %f hPa\r\n", temp, press);

delay(3000);

temp = readTemperature();
press = readPreassure();
printf("Temperatura: %f C,  Cisnienie: %f hPa\r\n", temp, press);


	
	return 0;
}
