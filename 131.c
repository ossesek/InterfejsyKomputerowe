#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


int main (void)
{

int fd;
int adressDS=0x68;
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

dane[0]=(dane[0]&0x0F)+((dane[0]&0x70)>>4)*10;
dane[1]=(dane[1]&0x0F)+((dane[1]&0x70)>>4)*10;
dane[2]=(dane[2]&0x0F)+((dane[2]&0x10)>>4)*10;
dane[3]=(dane[3]&0x0F)+((dane[3]&0x30)>>4)*10;
dane[4]=(dane[4]&0x0F)+((dane[4]&0x30)>>4)*10;
dane[5]=(dane[5]&0x0F)+((dane[5]&0xF0)>>4)*10;
printf("Praca studentow 232750,242066  \r\n");
printf("Odczytana data %d-%d-%d %d:%d:%d \r\n",dane[5]+2000,dane[4],dane[3],dane[2],dane[1],dane[0]);

return 0;

}
