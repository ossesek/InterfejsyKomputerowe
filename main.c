#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>
#include <math.h> //biblioteka potrzebna do zadania 4
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


int main (void)
{
int fd;
//wszystkie adresy zczytane zostaly z pliku mapa rejestrow
int adressMPU6050=0x68; // adres modulu MPU6050

//tablice potrzebne do zrealizowania zadan
int dane[2];
int zad2[2];
int zad3[12];

if(wiringPiSetup() == -1)
exit(1);

if((fd=wiringPiI2CSetup(adressMPU6050)) == -1)
{
printf("error initialize I2C");
exit(1);
}
printf("I2C modul MPU6050\r\n");
printf("Praca studentów: 232750, 242066 - 11.05.2020");

//Uruchamia pomiary
int regPWR=0x6B; // adres rejestru PWR_MGMT_1 
wiringPiI2CWriteReg8(fd, regPWR, 0);

// Zadanie 1 
//Odczytanie rejestru WHO_AM_I
int regWho=0x75;// adres rejestru Who Am I

dane[0] = wiringPiI2CReadReg8(fd,regWho); // przypisanie wartosci rejestru do 
printf("I am: %d\r\n",dane[0]); // odczytanie wartości rejestru WHO AM I

// Zadanie 2
int adres_Temp_H = 0x41; // adres TEMP_OUT[15:8]
int adres_Temp_L = 0x42; // adres TEMP_OUT[7:0]

zad2[0] = wiringPiI2CReadReg16(fd, adres_Temp_H); // zmieniono warrtość odczytana z 8 na 16 bitowa(blednie, mozliwe,ze dlatego nasza temperatura wyszla zbyt wysoka?) 
zad2[1] = wiringPiI2CReadReg16(fd, adres_Temp_L); //powinnysmy uzyc funkcji 'wiringPiI2CReadReg8'
int16_t Temp_konc=zad2[0]<<8|zad2[1];//przesuniecie bitowe
float temp=Temp_konc/340+36.53;//obliczanie temp koncowej

printf("Temperautra TEMP_H: %d\r\n", zad2[0]); // Wartośc komorki 0x41
printf("Temperatura TEMP_L: %d\r\n", zad2[1]); // Wartośc komorki 0x42 
printf("Temperatura koncowa: %f \n",temp); //wartosc temp koncowej po przeksztalceniach

// Zdanie 3
// adresy rejestrow Akcelerometru i zyroskopu, odpowiednio X,Y,Z (H oraz L)
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

//przypsanie tych wartosci do tablicy zad3
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
} // wypisanie tych wartosci w petli

//obliczanie polozenia X,Y,Z Akcelometru i zyroskopu
int16_t acc_x = zad3[0] << 8 | zad3[1] ;
int16_t acc_y = zad3[2] << 8 | zad3[3] ;
int16_t acc_z = zad3[4] << 8 | zad3[5];
// wypisanie juz obliczonych wartosc X,Y,Z
printf("Akcelometr x: %d\t", acc_x);
printf("Akcelometr y: %d\t", acc_y);
printf("Akcelometr z: %d\n", acc_z);
//to samo z zyroskopem
int16_t gyro_x = zad3[6] << 8 | zad3[7];
int16_t gyro_y = zad3[8] << 8 | zad3[9];
int16_t gyro_z = zad3[10]<< 8 | zad3[11]; 
printf("Żyroskop x : %d\t", gyro_x); 
printf("Żyroskop y: %d\t", gyro_y);
printf("Żyroskop z: %d\n", gyro_z);

//Zadanie 4 // poziomica
float poziom = atan2(acc_x, acc_z)*(180/3.1415); // aby zadzialala funkcja atan2, przy kompilacji trzeba dodac parametr '-lm'
printf("Poziom: %f \r\n", poziom); //wypisanie polozenia poziomicy
return 0;
}
