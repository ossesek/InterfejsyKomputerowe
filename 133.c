#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int fd;
int adress = 0x50; //adres pamieci EEPROM

/* Funkcja zapisuje pojedynczy bajt do komorki pamieci eeprom
 adress - adres komorki pod jaki zapisać
 value -  wartosc do zapisania
 */
int i2cWriteEEPROM(uint16_t adress, uint8_t value) {
	wiringPiI2CWriteReg16(fd, (adress >> 8), (value << 8) | (adress & 0xff));
	delay(20);
}

/*Funkcja odczytuje jeden bajt z podanego adresu komorki pamieci
 */
uint8_t i2cReadEeeprom(uint16_t adress) {
	wiringPiI2CWriteReg8(fd, (adress >> 8), (adress & 0xff));
	delay(20);
	return wiringPiI2CRead(fd);
}

int main(void) {

	int dane;
	printf("EEPROM \r\n");

	if (wiringPiSetup() == -1)
		exit(1);

	if ((fd = wiringPiI2CSetup(adress)) == -1) {
		printf("error initialize I2C");
		exit(1);
	}
	printf("I2C start EEPROM\r\n");

//Zapisuje do komorki 10 wartosc 44
	i2cWriteEEPROM(10, 44);

//Odczytanie z komorki 10 wartosc
	dane = i2cReadEeeprom(10);
	printf("%d\n ", dane);
	return 0;
}
