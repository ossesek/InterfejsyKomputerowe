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
dane[0]=0;
dane[1]=0x37;
dane[2]=0x05;
dane[3]=0x14;
dane[4]=0x02;
dane[5]=0x39;

if(wiringPiSetup() == -1)
exit(1);

if((fd=wiringPiI2CSetup(adressDS)) == -1){
printf("error initialize I2C");
exit(1);
}

int second=0x00;
int minutes=0x01;
int hours=0x02;
int day=0x04;
int month=0x05;
int year=0x06;

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

return 0;

}
