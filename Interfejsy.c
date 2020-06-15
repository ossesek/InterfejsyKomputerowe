#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

int chan = 0;
int speed = 1000000;
int32_t t_fine;

enum
{
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


typedef struct
{
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

int writeRegister(int adress, int data) //zapisujac danych przez SPI do urzadzenia
{
	unsigned char buff[2];  // 
	buff[0] = (adress & ~0x80); // 0x80 rejestr do zapisu albo odczytu
	buff[1] = data; 
	wiringPiSPIDataRW(chan, buff, 2); // kanal, buffor((dane) 2 - ilosc danych
	return 1;
}

int readRegister(int adress)
{
	unsigned char buff[2];
	buff[0] = (adress | 0x80);
	wiringPiSPIDataRW(chan, buff, 2);
	return buff[1];
}

int init()
{
	short unsigned wartosc = 0;

	//reset urzadzenia
	wartosc = 0xb6;
	writeRegister(BMP280_REGISTER_SOFTRESET, wartosc);
	delay(1000);

	return 0;
}

int32_t bmp280_compensate_T_int32(int32_t adc_T) //ta funkcja zwraca dane z czujnika ale nie przetworzone do temp
{
	int32_t var1, var2, T; 
	var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bmp280_compensate_P_int64(int32_t adc_P) //ta funkcja zwraca dane z czujnika ale nie przetworzone do cisnienia
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
	var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
	if (var1 == 0) {
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
	return (uint32_t)p;
}

float readPreassure()
{
	int32_t rawPreassure = (readRegister(0xF7) << 12) | (readRegister(0xF8) << 4) | (readRegister(0xF9)>>4); // 20 bitowa wartosc, pierwsze 12 bitow jest puste, 4 kolejne to sa gorna wartosc a kolejne 4 to dolna wartosc 0:3 najmlodsze bity 4:7 najstarsze
	return bmp280_compensate_P_int64(rawPreassure) / 256.0 / 100.0; // | suma logiczna
}

float readTemperature()
{
	int32_t rawTemperature = (readRegister(0xFA) << 12) | (readRegister(0xFB) << 4) | (readRegister(0xFC)>>4);
	return bmp280_compensate_T_int32(rawTemperature) / 100.0;
}

void CalibrationData() 
{
	calib_data.dig_P1 = (readRegister(0x8F) << 8) | (readRegister(0x8E)); // 16 bitowe warotsci 
	calib_data.dig_P2 = (readRegister(0x91) << 8) | (readRegister(0x90));
	calib_data.dig_P3 = (readRegister(0x93) << 8) | (readRegister(0x92));
	calib_data.dig_P4 = (readRegister(0x95) << 8) | (readRegister(0x94));
	calib_data.dig_P5 = (readRegister(0x97) << 8) | (readRegister(0x96));
	calib_data.dig_P6 = (readRegister(0x99) << 8) | (readRegister(0x98));
	calib_data.dig_P7 = (readRegister(0x9B) << 8) | (readRegister(0x9A));
	calib_data.dig_P8 = (readRegister(0x9D) << 8) | (readRegister(0x9C));
	calib_data.dig_P9 = (readRegister(0x9F) << 8) | (readRegister(0x9E));
	calib_data.dig_T1 = (readRegister(0x89) << 8) | (readRegister(0x88));
	calib_data.dig_T2 = (readRegister(0x8B) << 8) | (readRegister(0x8A));
	calib_data.dig_T3 = (readRegister(0x8D) << 8) | (readRegister(0x8C));
}


int main()
{
	printf("MENU\n");
    printf("1. Odczyt temperatury z modulu MPU6050\n");
    printf("2. Odczytanie wartosci pomiarow z akcelerometru i zyroskopu\n");
    printf("3. Odczytanie akutalnej daty\n");
    printf("4. Ustawianie nowej daty\n");
	printf("5. Odczytanie wartosci EEPROM\n");
	printf("6. Zapis wartosci EEPROM\n");
	printf("7. BMP280 odczytanie ID modulu, temperatury i cisnienia\n");
	printf("9. Informacje o autorach\n");
	printf("0. Informacje o opcjach \n\n\n");

int option = 0;
while (option != 8)
{
	printf("Podaj numer: ");
	scanf("%i", &option);
	printf("\n");
	switch (option)
	{
		//odczytanie temperatury z modułu MPU6050
	case 1:
	{
		int fd;
		int adressMPU6050 = 0x69;

		int zad2[2];

		if (wiringPiSetup() == -1)
			exit(1);

		if ((fd = wiringPiI2CSetup(adressMPU6050)) == -1) {
			printf("error initialize I2C");
			exit(1);
		}
		printf("Praca studentów: 232750, 242066\n");
		
		//Uruchamia pomiary
		int regPWR = 0x6B; //<<adres rejestru PWR_MGMT_1
		wiringPiI2CWriteReg8(fd, regPWR, 0);
			
		int adres_Temp_H = 0x41; //TEMP_OUT[15:8]
		int adres_Temp_L = 0x42; //TEMP_OUT[7:0]

		zad2[0] = wiringPiI2CReadReg8(fd, adres_Temp_H); 
		zad2[1] = wiringPiI2CReadReg8(fd, adres_Temp_L);
		int16_t Temp_konc=	(zad2[0] << 8) + zad2[1];
		float temp=((float)Temp_konc/340)+36.53; // wzor na temp

		printf("Temperautra TEMP_H: %d\r\n", zad2[0]); // Wartośc komorki 0x41
		printf("Temperatura TEMP_L: %d\r\n", zad2[1]); // Wartośc komorki 0x42 
		printf("Temperatura koncowa: %f \n",temp);
	}
	break;
	//odczytanie wartości pomiarów z akcelerometru i żyroskopu
	case 2:
	{
		int fd;
		int adressMPU6050 = 0x69;

		int zad3[12];

		if (wiringPiSetup() == -1)
			exit(1);

		if ((fd = wiringPiI2CSetup(adressMPU6050)) == -1) {
			printf("error initialize I2C");
			exit(1);
		}

		//Uruchamia pomiary
		int regPWR = 0x6B; //<<adres rejestru PWR_MGMT_1
		wiringPiI2CWriteReg8(fd, regPWR, 0);

		int A_ACCEL_XOUT_H = 0x3B;
		int A_ACCEL_XOUT_L = 0x3C;
		int A_ACCEL_YOUT_H = 0x3D; 
		int A_ACCEL_YOUT_L = 0x3E; 
		int A_ACCEL_ZOUT_H = 0x3F; 
		int A_ACCEL_ZOUT_L = 0x40;
		int A_GYRO_XOUT_H = 0x43; 
		int A_GYRO_XOUT_L = 0x44;
		int A_GYRO_YOUT_H = 0x45; 
		int A_GYRO_YOUT_L = 0x46; 
		int A_GYRO_ZOUT_H = 0x47;
		int A_GYRO_ZOUT_L = 0x48;

		zad3[0] = wiringPiI2CReadReg8(fd, A_ACCEL_XOUT_H); 
		zad3[1] = wiringPiI2CReadReg8(fd, A_ACCEL_XOUT_L);
		zad3[2] = wiringPiI2CReadReg8(fd, A_ACCEL_YOUT_H); 
		zad3[3] = wiringPiI2CReadReg8(fd, A_ACCEL_YOUT_L); 
		zad3[4] = wiringPiI2CReadReg8(fd, A_ACCEL_ZOUT_H); 
		zad3[5] = wiringPiI2CReadReg8(fd, A_ACCEL_ZOUT_L); 
		zad3[6] = wiringPiI2CReadReg8(fd, A_GYRO_XOUT_H); 
		zad3[7] = wiringPiI2CReadReg8(fd, A_GYRO_XOUT_L); 
		zad3[8] = wiringPiI2CReadReg8(fd, A_GYRO_YOUT_H); 
		zad3[9] = wiringPiI2CReadReg8(fd, A_GYRO_YOUT_L); 
		zad3[10] = wiringPiI2CReadReg8(fd, A_GYRO_ZOUT_H); 
		zad3[11] = wiringPiI2CReadReg8(fd, A_GYRO_ZOUT_L); 
		
		for (int i = 0; i < 12; i++) {
			printf("Dane z żyroskopu i akcelometru : %d\n", zad3[i]);
		}
		
		int16_t acc_x = zad3[0] << 8 | zad3[1] ;
		int16_t acc_y = zad3[2] << 8 | zad3[3] ;
		int16_t acc_z = zad3[4] << 8 | zad3[5];
		printf("Akcelometr x: %d\t", acc_x);
		printf("Akcelometr y: %d\t", acc_y);
		printf("Akcelometr z: %d\n", acc_z);
		int16_t gyro_x = zad3[6] << 8 | zad3[7];
		int16_t gyro_y = zad3[8] << 8 | zad3[9];
		int16_t gyro_z = zad3[10]<< 8 | zad3[11]; 
		printf("Żyroskop x : %d\t", gyro_x); 
		printf("Żyroskop y: %d\t", gyro_y);
		printf("Żyroskop z: %d\n", gyro_z);

		float poziom2= atan2(gyro_x, gyro_z)*(180/3.1415);
		printf("Poziom: %f \r\n", poziom2);
	}
	break;
	//odczytanie aktualnej daty z modułu RTC
	case 3:
	{
		int fd; //numer otwartego pyliku ' uchwyt do pliku ' 
		int adressDS = 0x68;
		int dane[6];

		if(wiringPiSetup() == -1)
		exit(1);

		if((fd=wiringPiI2CSetup(adressDS)) == -1){
		printf("error initialize I2C");
		exit(1);
		}
		//Uruchamia pomiary
		int second=0x00;
		dane[0] = wiringPiI2CReadReg8(fd,second);

		int minutes=0x01;
		dane[1] = wiringPiI2CReadReg8(fd, minutes);

		int hours=0x02;
		dane[2] = wiringPiI2CReadReg8(fd, hours);

		int day=0x04;
		dane[3] = wiringPiI2CReadReg8(fd, day);

		int month=0x05;
		dane[4] = wiringPiI2CReadReg8(fd, month);

		int year=0x06;
		dane[5] = wiringPiI2CReadReg8(fd, year);
		
		// & iloczyn
		dane[0]=(dane[0]&0x0F)+((dane[0]&0x70)>>4)*10; // 1cz. maska bitowa ktora przepuszcza tylko bity od 0 do 15. 
		dane[1]=(dane[1]&0x0F)+((dane[1]&0x70)>>4)*10;
		dane[2]=(dane[2]&0x0F)+((dane[2]&0x10)>>4)*10;
		dane[3]=(dane[3]&0x0F)+((dane[3]&0x30)>>4)*10;
		dane[4]=(dane[4]&0x0F)+((dane[4]&0x30)>>4)*10;
		dane[5]=(dane[5]&0x0F)+((dane[5]&0xF0)>>4)*10;
		printf("Praca studentow 232750,242066  \r\n");
		printf("Odczytana data %d-%d-%d %d:%d:%d \r\n",dane[5]+2000,dane[4],dane[3],dane[2],dane[1],dane[0]);


	}
	break;
	//wprowadzenie nowej daty dla modułu RTC
	case 4:
	{
		int fd;
		int adressDS1307 = 0x68;

		//przypisanie swojej daty
		int dane[6];
		dane[0]=0;
		dane[1]=0x37;
		dane[2]=0x05;
		dane[3]=0x14;
		dane[4]=0x02;
		dane[5]=0x39;
		
		if (wiringPiSetup() == -1)
			exit(1);

		if ((fd = wiringPiI2CSetup(adressDS1307)) == -1) {
			printf("error initialize I2C");
			exit(1);
		}
		
		
		int second=0x00;
		int minutes=0x01;
		int hours=0x02;
		int day=0x04;
		int month=0x05;
		int year=0x06;
		
		//wpisanie danych do rejestrow
		wiringPiI2CWriteReg8 ( fd,  second, dane[0]) ;
		wiringPiI2CWriteReg8 ( fd,  minutes, dane[1]) ;
		wiringPiI2CWriteReg8 ( fd,  hours, dane[2]) ;
		wiringPiI2CWriteReg8 ( fd,  day, dane[3]) ;
		wiringPiI2CWriteReg8 ( fd,  month, dane[4]) ;
		wiringPiI2CWriteReg8 ( fd,  year, dane[5]) ;

		dane[0]=(dane[0]&0x0F)+((dane[0]&0x70)>>4)*10;
		dane[1]=(dane[1]&0x0F)+((dane[1]&0x70)>>4)*10;
		dane[2]=(dane[2]&0x0F)+((dane[2]&0x10)>>4)*10;
		dane[3]=(dane[3]&0x0F)+((dane[3]&0x30)>>4)*10;
		dane[4]=(dane[4]&0x0F)+((dane[4]&0x30)>>4)*10;
		dane[5]=(dane[5]&0x0F)+((dane[5]&0xF0)>>4)*10;
		printf("Praca studentow 232750,242066  \r\n");
		printf("Przypisana data %d-%d-%d %d:%d:%d \r\n",dane[5],dane[4],dane[3],dane[2],dane[1],dane[0]);

	}
	break;
	//odczytanie 16 - bitowej wartości z pamięci EEPROM
	case 5:
	{
	int fd;
	int adresE;
	
	int adressAT24C32=0x50; //deklaracja zmiennej int (adres modułu)
	
    if (wiringPiSetup() == -1) exit(1); //inicjalizacja biblioteki wiringPi
    if((fd=wiringPiI2CSetup(adressAT24C32)) == -1){
		printf("error initialize I2C");
		exit(1);
	} //przypisanie do zmiennej fd numeru (file deskryptora) oraz sprawdzenie czy inicjalizacja przebiegla pomsyslnie
	
	 
	printf("Adres rejestru do odczytu liczby: "); 
	scanf("%d", &adresE);
	
	uint8_t i2cReadEeeprom(int fd, uint16_t adress) 
	{ 
		wiringPiI2CWriteReg8(fd, (adress > 8), (adress & 0xff)); 
		delay(20); 
		return wiringPiI2CRead(fd); 
	}
	
	int firstPart=(i2cReadEeeprom(fd, adresE)<<8); //odczyt 8 bitow i przesuniecie ich na miejsca 8-15    
	int secondPart=i2cReadEeeprom(fd, adresE+1); //odczyt 8 bitow 
	int final = firstPart + secondPart; //16-bitowa liczba 
	printf("Liczba: %d\n", final); //wyswietlenie 


	}
	break;
	//zapisanie 16 - bitowej wartości do pamięci EEPROM
	case 6:
	{
		int fd;
		int adresE; 
		int x; 
		printf("Adres rejestru do zapisu liczby: "); 
		scanf("%d", &adresE); 
		printf("Podaj liczbe: "); 
		scanf("%d", &x); 
		
		int adressAT24C32=0x50; //deklaracja zmiennej int (adres modułu)
		
		if (wiringPiSetup() == -1) exit(1); //inicjalizacja biblioteki wiringPi
		if((fd=wiringPiI2CSetup(adressAT24C32)) == -1){
			printf("error initialize I2C");
			exit(1);
		} //przypisanie do zmiennej fd numeru (file deskryptora) oraz sprawdzenie czy inicjalizacja przebiegla pomsyslnie
		
		int i2cWriteEEPROM(int fd, uint16_t adress, uint8_t x) 
		{ 
			wiringPiI2CWriteReg16(fd, (adress >> 8), (x << 8) | (adress & 0xff)); 
			delay(20); 
		}
		
		uint8_t firstPart = (x & 0xff00) >> 8; //wyzerowanie bitow 0-7 i przesuniecie bitow od 8-15 na miejsca 0-7    
		uint8_t secondPart = x & 0x00ff; //wyzerowanie bitow 8-15 
		i2cWriteEEPROM(fd, adresE, firstPart); //zapis bitow 8-15 z liczby do pamieci EEPROM    
		i2cWriteEEPROM(fd, adresE +1, secondPart); //zapis bitow 0-7 z liczby do nastepnej komorki pamieci EEPROM

	}
	break;
	//BMP280 – odczytanie ID modułu, temperatury i ciśnienia atmosferycznego

	case 7:
	{
		if (wiringPiSPISetup(chan, speed) == -1)
		{
			printf("Could not initialise SPI\n");
			return 0;
		}

		init();
		delay(600);

		int id = readRegister(0x90);

		printf("Czujnik BMP280 - ChipID: %x\r\n", id);

		writeRegister(BMP280_REGISTER_CONTROL, 0x57); // ustawienie szybkosci odswiezania dla tempx2 i cisnienia x16
		int config = readRegister(BMP280_REGISTER_CONTROL);
		//załadowanie danych kalibracji
		CalibrationData();

		//odczyt
		float temp = readTemperature();
		float press = readPreassure();
		delay(2000);

		temp = readTemperature();
		press = readPreassure();
		printf("Temperatura: %f C,  Cisnienie: %f hPa\r\n", temp, press);

	}
	break;
	//informacja o autorach
	case 9:
	{
		printf("Praca studentow: Magda Zarczynska - 242066,\n Roksana Smolka - 232750\n");
	}
	break;

	case 0:
	{
		printf("1. Odczyt temperatury z modulu MPU6050\n");
		printf("2. Odczytanie wartosci pomiarow z akcelerometru i zyroskopu\n");
		printf("3. Odczytanie akutalnej daty\n");
		printf("4. Ustawianie nowej daty\n");
		printf("5. Odczytanie wartosci EEPROM\n");
		printf("6. Zapis wartosci EEPROM\n");
		printf("7. BMP280 odczytanie ID modulu, temperatury i cisnienia\n");
		printf("9. Informacje o autorach\n");
		printf("0. Informacje o opcjach \n\n\n");
	}
	break;
	default: printf("Wprowadzono niepoprawny numer\n");

	}

}
return 0;
}