#include "mbed.h"
#include "Motor.h"
#include "QEI.h"
#include "PID.h"

#define i2c_address 0x60

Serial pc(USBTX, USBRX);
Serial bluetooth(PB_6, PA_10);
I2C CMPS(PB_4, PA_8);
//SRF05 ultra(PA_9, PC_7);
float jarak;

DigitalOut trigPin1(PA_9); // sensor tengah 
DigitalIn echoPin1(PC_7);
DigitalIn ir(PB_9);
int eksistensi_koin;
int ada = 0;
int tidak_ada = 1;
int i;
int kecepatan_state;

QEI DepanKananQEI(PB_8, PC_9, NC, 986); //roda depan kanan PB_1, PB_15, NC, 624
QEI BelakangKananQEI(PB_1, PB_15, NC, 986); //roda belakang kiri PB_14, PB_13, NC, 624
QEI DepanKiriQEI(PC_12, PC_10, NC, 986); //roda depan kiri 
QEI BelakangKiriQEI(PB_7, PA_15, NC, 986); //roda belakang kiri

//Tuning Di alas
//PID DepanKananPID(0.40, 0.020, 0.00001, 0.02);  //0000001 Kc, Ti, Td MOTOR baru nomor 1
//PID BelakangKananPID(0.37, 0.018, 0.0000000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 2
//PID BelakangKiriPID(0.496, 0.0565, 0.0000000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 3
//PID DepanKiriPID(0.85, 0.08, 0.000000000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 4

//Tuning Di Lantai
//PID DepanKananPID(0.5, 0.020, 0.00001, 0.02);  //0000001 Kc, Ti, Td MOTOR baru nomor 1
//PID BelakangKananPID(0.5, 0.022, 0.00000000001, 0.02);
//PID BelakangKiriPID(0.496, 0.0565, 0.0000000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 3
//PID DepanKiriPID(0.85, 0.08, 0.000000000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 4

PID DepanKananPID(0.8, 0.1, 0.00000001, 0.02);  //0000001 Kc, Ti, Td MOTOR baru nomor 1
PID BelakangKananPID(0.78, 0.1, 0.0000001, 0.02);
PID BelakangKiriPID(0.78, 0.1, 0.000000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 3
PID DepanKiriPID(0.87, 0.1, 0.000001, 0.02);  //Kc, Ti, Td MOTOR baru nomor 4



Motor DepanKanan(PC_6, PA_11, PB_12);
Motor DepanKiri(PA_0, PA_4, PB_0);
Motor BelakangKanan(PC_8, PC_5, PA_12);
Motor BelakangKiri(PA_1, PC_1, PC_0);

PwmOut Servo(PA_7);
PwmOut Servo2(PB_10);

Timer t_jalan;
Timer t2;
Timer t;
Timer t_capit;

char nilai, huruf;
int Jalan;
int Jalan_Maju =1;
int Berhenti = 0;
int Jalan_Mundur = 2;
int Jalan_Kanan = 3;
int Jalan_Kiri = 4;
int ke_setpoint = 5;
int mode_inverse = 6;
int Rotasi_Kanan = 7;
int Rotasi_Kiri = 8;
int Belok_Kanan = 9;
int Belok_Kiri = 10;
int Muter_Kiri = 11;
int Muter_Kanan =12;
int Capit_Buka = 13;
int Capit_Muter = 14;
int Cari_Koin1 = 15;
int Cari_Koin2 = 16;
int Mundur_Belok_Kanan = 17;
int Mundur_Belok_Kiri = 18;
int Ganti_Kecepatan = 19;

int DepanKananPulses      = 0; 
int DepanKananPrevPulses  = 0; 
float DepanKananVelocity  = 0.0; 
    
int BelakangKananPulses     = 0; 
int BelakangKananPrevPulses = 0; 
float BelakangKananVelocity = 0.0;

int BelakangKiriPulses     = 0;
int BelakangKiriPrevPulses = 0; 
float BelakangKiriVelocity = 0.0;
    
int DepanKiriPulses     = 0;
int DepanKiriPrevPulses = 0;
float DepanKiriVelocity = 0.0;

int Depan_Kanan_Count;
int Depan_Kiri_Count;
int Belakang_Kanan_Count;
int Belakang_Kiri_Count;

int d_pulse_DepanKanan;
int d_pulse_DepanKiri;
int d_pulse_BelakangKanan;
int d_pulse_BelakangKiri;

int koordinat_x_DepanKiri;
int koordinat_x_DepanKanan;
int koordinat_x_BelakangKiri;
int koordinat_x_BelakangKanan;
int koordinat_y_DepanKiri;
int koordinat_y_DepanKanan;
int koordinat_y_BelakangKiri;
int koordinat_y_BelakangKanan;
float koordinat_x;
float koordinat_y;
int Resolusi = 986;
float pi = 3.141;
float diameter = 5.8f;
float rtd = pi/180.0f;
int setpoint_x;
int setpoint_y;
float radius = 0.029;
float orientasi;
int orientasi_2;
int target_orientasi;
float waktu;
int BEARING_Register;
char bits[2];
int _byteHigh;
int _byteLow;
float bearing;

float target_depan_kiri;
float target_depan_kanan;
float target_belakang_kiri;
float target_belakang_kanan;
float arah_depan_kiri;
float arah_depan_kanan;
float arah_belakang_kiri;
float arah_belakang_kanan;

float temp_x_DepanKiri;
float temp_x_DepanKanan;
float temp_x_BelakangKanan;
float temp_x_BelakangKiri;
float temp_y_DepanKiri;
float temp_y_DepanKanan;
float temp_y_BelakangKanan;
float temp_y_BelakangKiri;
float temp_x;
float temp_y;
float terhadap_x;
float terhadap_y;
int koordinat_x_bulat;
int koordinat_y_bulat;
float target_x_basis;
float target_y_basis;
float sudut_awal;
int capit_state;
int capit2_state;

void fungsi_bluetooth(void)
{
    if(bluetooth.readable())
    {
        setpoint_x = bluetooth.getc();
        setpoint_y = bluetooth.getc();
        int target_sudut = bluetooth.getc();
        nilai = bluetooth.getc();
        if (nilai == 'S')
        {
            huruf = 'S';
            Jalan = Berhenti;  
        }
        else if (nilai == 'F')
        {
            huruf = 'F';
            Jalan = Jalan_Maju;
        }
        else if(nilai =='B')
        {
            huruf = 'B';
            Jalan = Jalan_Mundur;
        }
        else if(nilai =='L')
        {
            huruf = 'L';
            Jalan = Jalan_Kiri;
        }
        else if(nilai =='K')
        {
            huruf = 'K';
            Jalan = Belok_Kiri;
        }
        else if(nilai =='A')
        {
            huruf = 'A';
            Jalan = Belok_Kanan;
        }
        else if(nilai =='R')
        {
            huruf = 'R';
            Jalan = Jalan_Kanan;
        }
        else if(nilai =='Q')
        {
            huruf = 'Q';
            Jalan = Muter_Kiri;
        }
        else if(nilai =='E')
        {
            huruf = 'E';
            Jalan = Muter_Kanan;
        }
        else if(nilai =='M')
        {
            huruf = 'M';
            Jalan = Rotasi_Kanan;
        }
        else if(nilai =='W')
        {
            huruf = 'W';
            Jalan = Rotasi_Kiri;
        }
        else if(nilai == 'U')
        {
            huruf = 'U';
            Jalan = Mundur_Belok_Kiri;
        }
        else if(nilai == 'V')
        {
            huruf = 'V';
            Jalan = Mundur_Belok_Kanan;
        }
        else if(nilai =='T')
        {
            huruf = 'T';
            Jalan = Capit_Buka;
        }
        else if(nilai =='O')
        {
            huruf = 'O';
            Jalan = Capit_Muter;
        }
        else if(nilai =='J')
        {
            huruf = 'J';
            Jalan = Cari_Koin1;
            //Jalan = ke_setpoint;
        }
        else if(nilai =='H')
        {
            huruf = 'H';
            Jalan = Cari_Koin2;
            //Jalan = ke_setpoint;
        }
        
        else if(nilai =='Y')
        {
            huruf = 'G';
            Jalan = ke_setpoint;
        }
        else if(nilai == 'G')
        {
            huruf = 'G';
            Jalan = mode_inverse;
        }
        else if(nilai == 'X')
        {
            huruf = 'X';
            Jalan = Ganti_Kecepatan;
        }
        else if(nilai == 'S')
        {
            Jalan = Berhenti;
        }
        else if(nilai == 'p')
        {
            NVIC_SystemReset();
        }         
    }
}

void kosongkan()
{
    BelakangKananQEI.reset();
    DepanKananQEI.reset();
    DepanKiriQEI.reset();
    BelakangKiriQEI.reset();
                
    DepanKananPulses      = 0;
    DepanKananPrevPulses  = 0; 
    DepanKananVelocity  = 0.0; 
                
    BelakangKananPulses     = 0; 
    BelakangKananPrevPulses = 0; 
    BelakangKananVelocity = 0.0;
    
    BelakangKiriPulses     = 0;
    BelakangKiriPrevPulses = 0; 
    BelakangKiriVelocity = 0.0;
                
    DepanKiriPulses     = 0;
    DepanKiriPrevPulses = 0;
    DepanKiriVelocity = 0.0; 
}

void proses_kecepatan()
{
    DepanKananPulses = DepanKananQEI.getPulses();
    d_pulse_DepanKanan = DepanKananPulses - DepanKananPrevPulses;
    DepanKananVelocity = (DepanKananPulses - DepanKananPrevPulses) / 0.02;
    DepanKananPrevPulses = DepanKananPulses;             
    DepanKananPID.setProcessValue(fabs(DepanKananVelocity));
    
    DepanKiriPulses = DepanKiriQEI.getPulses();
    d_pulse_DepanKiri = DepanKiriPulses - DepanKiriPrevPulses;
    DepanKiriVelocity = (DepanKiriPulses - DepanKiriPrevPulses) / 0.02;
    DepanKiriPrevPulses = DepanKiriPulses;             
    DepanKiriPID.setProcessValue(fabs(DepanKiriVelocity));
    
    BelakangKananPulses = BelakangKananQEI.getPulses();
    d_pulse_BelakangKanan = BelakangKananPulses - BelakangKananPrevPulses;
    BelakangKananVelocity = (BelakangKananPulses - BelakangKananPrevPulses) / 0.02;
    BelakangKananPrevPulses = BelakangKananPulses;             
    BelakangKananPID.setProcessValue(fabs(BelakangKananVelocity));
    
    BelakangKiriPulses = BelakangKiriQEI.getPulses();
    d_pulse_BelakangKiri = BelakangKiriPulses - BelakangKiriPrevPulses;
    BelakangKiriVelocity = (BelakangKiriPulses - BelakangKiriPrevPulses) / 0.02;
    BelakangKiriPrevPulses = BelakangKiriPulses;             
    BelakangKiriPID.setProcessValue(fabs(BelakangKiriVelocity));
}

/*float Get_Yaw()
{
    CMPS.unlock();
    CMPS.start();
    // to indicate an i2c read, shift the 7 bit address up 1 bit and set bit 0 to a 1
    CMPS.write(i2c_address << 1); 
    int writeResult = CMPS.write(BEARING_Register);
    if(writeResult != 1)
    {
        pc.printf("%d\n", writeResult);
        return bearing;
    }
    else
    {
        CMPS.stop();
        CMPS.read(i2c_address <<1, bits, 2);
        _byteHigh = bits[0];
        _byteLow = bits[1];
        bearing = ((_byteHigh<<=8) + _byteLow) / 10;
        return bearing;
    }
}
*/

void Get_Count()
{
    Depan_Kanan_Count = Depan_Kanan_Count + d_pulse_DepanKanan;
    Depan_Kiri_Count = Depan_Kiri_Count + d_pulse_DepanKiri;
    Belakang_Kanan_Count = Belakang_Kanan_Count + d_pulse_BelakangKanan;
    Belakang_Kiri_Count = Belakang_Kiri_Count + d_pulse_BelakangKiri;
    
    temp_y_DepanKiri = ((double)d_pulse_DepanKiri/(double)Resolusi)*pi*diameter*cos(45.0f*rtd);
    temp_y_DepanKanan = ((double)d_pulse_DepanKanan/(double)Resolusi)*pi*diameter*cos(135.0f*rtd);
    temp_y_BelakangKanan = ((double)d_pulse_BelakangKanan/(double)Resolusi)*pi*diameter*cos(225.0f*rtd);
    temp_y_BelakangKiri = ((double)d_pulse_BelakangKiri/(double)Resolusi)*pi*diameter*cos(315.0f*rtd);
    
    temp_x_DepanKiri = ((double)d_pulse_DepanKiri/(double)Resolusi)*pi*diameter*sin(45.0f*rtd);
    temp_x_DepanKanan = ((double)d_pulse_DepanKanan/(double)Resolusi)*pi*diameter*sin(135.0f*rtd);
    temp_x_BelakangKanan = ((double)d_pulse_BelakangKanan/(double)Resolusi)*pi*diameter*sin(225.0f*rtd);
    temp_x_BelakangKiri = ((double)d_pulse_BelakangKiri/(double)Resolusi)*pi*diameter*sin(315.0f*rtd);
    
    temp_x = (temp_x_DepanKiri + temp_x_DepanKanan + temp_x_BelakangKiri + temp_x_BelakangKanan)/2;
    temp_y = (temp_y_DepanKiri + temp_y_DepanKanan + temp_y_BelakangKiri + temp_y_BelakangKanan)/2;
    
    terhadap_x = -1*(double)temp_y*sin(orientasi*rtd) + (double)temp_x*cos(orientasi*rtd);
    terhadap_y = (double)temp_y*cos(orientasi*rtd) + (double)temp_x*sin(orientasi*rtd);
}

void Cari_Koordinat()
{
    //orientasi = -1.0f * ((double)Get_Yaw()- sudut_awal);
    //orientasi_2 = orientasi;
    orientasi = 0;
    
    koordinat_y = koordinat_y + terhadap_y;
    koordinat_x = koordinat_x + terhadap_x;
    koordinat_x_bulat = koordinat_x;
    koordinat_y_bulat = koordinat_y;
}

float cari_abs(float a)
{
    if(a<0)
    {
        if(fabs(a)<20)
        {
            return 0.0f;
        }
        else if(fabs(a)>=20)
        {
            return -1.0f;
        }
    }
    else if(a>0)
    {
        if(fabs(a)<20)
        {
            return 0.0f;
        }
        else if(fabs(a)>=20)
        {
            return 1.0f;
        }
    }
    else 
    {
        return 0.0f;
    }
}

void cari_inverse(float setpoint_sudut, float target_x, float target_y)
{
    //mengganti basis vektor berdasarkan sudut(rotasi axis)
    setpoint_sudut = setpoint_sudut*rtd;
    target_x = target_x - koordinat_x;
    target_y = target_y - koordinat_y;
    target_x_basis = target_x*cos(setpoint_sudut*-1.0f) - target_y*sin(setpoint_sudut*-1.0f);
    target_y_basis = target_x*sin(setpoint_sudut*-1.0f) + target_y*cos(setpoint_sudut*-1.0f);
    target_x = target_x_basis;
    target_y = target_y_basis;
    //meengganti satuan target
    float target_vx;
    float target_vy;
    target_x = (double)target_x/100.0f;
    target_y = (double)target_y/100.0f;
    //mencari target kecepatan translasi dari robot
    //maks kecepatan translasi per aksis 0.6 m/s
    if(fabs(target_x)>fabs(target_y))
    {
        if((target_x<0)&&(target_y<0))
        {
            target_vx = -0.6;
            target_vy = ((double)target_y/(double)target_x)*target_vx;
        }
        else if((target_x<0)&&(target_y>0))
        {
            target_vx = -0.6;
            target_vy = ((double)target_y/(double)target_x)*target_vx;
        }
        else if((target_x>0)&&(target_y>0))
        {
            target_vx = 0.6;
            target_vy = ((double)target_y/(double)target_x)*target_vx;
        }
        else if((target_x>0)&&(target_y<0))
        {
            target_vx = 0.6;
            target_vy = ((double)target_y/(double)target_x)*target_vx;
        }
        else if((target_y==0)&&(target_x>0))
        {
            target_vx = 0.6;
            target_vy = 0;
        }
        else if((target_y==0)&&(target_x<0))
        {
            target_vx = -0.6;
            target_vy = 0;
        }
    }
    else if(fabs(target_x)<fabs(target_y))
    {
        if((target_x<0)&&(target_y<0))
        {
            target_vy = -0.6;
            target_vx = ((double)target_x/(double)target_y)*target_vy;
        }
        else if((target_x<0)&&(target_y>0))
        {
            target_vy = 0.6;
            target_vx = ((double)target_x/(double)target_y)*target_vy;
        }
        else if((target_x>0)&&(target_y>0))
        {
            target_vy = 0.6;
            target_vx = ((double)target_x/(double)target_y)*target_vy;
        }
        else if((target_x>0)&&(target_y<0))
        {
            target_vy = -0.6;
            target_vx = ((double)target_x/(double)target_y)*target_vy;
        }
        else if((target_x==0)&&(target_y>0))
        {
            target_vy = 0.6;
            target_vx = 0;
        }
        else if((target_x==0)&&(target_y<0))
        {
            target_vy = -0.6;
            target_vx = 0;
        }
    }
    else if(fabs(target_x)==fabs(target_y))
    {
        if((target_x<0)&&(target_y<0))
        {
            target_vy = -0.6;
            target_vx = target_vy;
        }
        else if((target_x<0)&&(target_y>0))
        {
            target_vy = 0.6;
            target_vx = -1*target_vy;
        }
        else if((target_x>0)&&(target_y<0))
        {
            target_vy = -0.6;
            target_vx = -1*target_vy;
        }
        else if((target_x>0)&&(target_y>0))
        {
            target_vy = 0.6;
            target_vx = target_vy;
        }
    }
    //mencari kecepatan sudut tiap roda berdasarkan target kecepatan translasi
    target_depan_kiri = (1/radius)*(sin(45.0f*rtd)*target_vx + cos(45.0f*rtd)*target_vy);
    target_depan_kanan = (1/radius)*(sin(135.0f*rtd)*target_vx + cos(135.0f*rtd)*target_vy);
    target_belakang_kanan = (1/radius)*(sin(225.0f*rtd)*target_vx + cos(225.0f*rtd)*target_vy);
    target_belakang_kiri = (1/radius)*(sin(315.0f*rtd)*target_vx + cos(315.0f*rtd)*target_vy);
    target_depan_kiri = (target_depan_kiri*624.0f)/(2.0f*3.14);
    target_depan_kanan = (target_depan_kanan*624.0f)/(2.0f*3.14);
    target_belakang_kiri = (target_belakang_kiri*624.0f)/(2.0f*3.14);
    target_belakang_kanan = (target_belakang_kanan*624.0f)/(2.0f*3.14);
    //mengambil arah gerak motor
    arah_depan_kiri = cari_abs(target_depan_kiri);
    arah_depan_kanan = cari_abs(target_depan_kanan);
    arah_belakang_kiri = cari_abs(target_belakang_kiri);
    arah_belakang_kanan = cari_abs(target_belakang_kanan);
}

void set_inverse()
{
    //memasukkan nilai target kecepatan sudut untuk setpoint PID
    if(arah_depan_kanan!=0)
    {
        if(arah_depan_kiri==0)
        {
            DepanKananPID.setSetPoint(fabs(target_depan_kanan));
            BelakangKiriPID.setSetPoint(fabs(target_belakang_kiri));
            BelakangKananPID.setSetPoint(0);
            DepanKiriPID.setSetPoint(0);
        }
        else if(arah_depan_kiri!=0)
        {
            DepanKananPID.setSetPoint(fabs(target_depan_kanan));
            BelakangKiriPID.setSetPoint(fabs(target_belakang_kiri));
            BelakangKananPID.setSetPoint(fabs(target_belakang_kanan));
            DepanKiriPID.setSetPoint(fabs(target_depan_kiri));
        }
    }
    else if(arah_depan_kiri!=0)
    {
        if(arah_depan_kanan==0)
        {
            DepanKananPID.setSetPoint(0);
            BelakangKiriPID.setSetPoint(0);
            BelakangKananPID.setSetPoint(fabs(target_belakang_kanan));
            DepanKiriPID.setSetPoint(fabs(target_depan_kiri));
        }
        else if(arah_depan_kanan!=0)
        {
            DepanKananPID.setSetPoint(fabs(target_depan_kanan));
            BelakangKiriPID.setSetPoint(fabs(target_belakang_kiri));
            BelakangKananPID.setSetPoint(fabs(target_belakang_kanan));
            DepanKiriPID.setSetPoint(fabs(target_depan_kiri));
        }
    }
}

void jalan_maju()
{
    t_jalan.reset();
    t_jalan.start();
    proses_kecepatan();
    Get_Count();
    Cari_Koordinat();
    pc.printf("Jarak : %d\n", jarak);
    //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
    BelakangKanan.speed(BelakangKananPID.compute()*-1);
    BelakangKiri.speed(BelakangKiriPID.compute());
    DepanKanan.speed(DepanKananPID.compute()*-1);
    DepanKiri.speed(DepanKiriPID.compute());
    while(t_jalan.read_ms()<=20)
    {
    }
    t_jalan.reset();
}

void jalan_mundur()
{
    t_jalan.reset();
    t_jalan.start();
    proses_kecepatan();
    Get_Count();
    Cari_Koordinat();
    pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
    pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
    BelakangKanan.speed(BelakangKananPID.compute());
    BelakangKiri.speed(BelakangKiriPID.compute()*-1);
    DepanKanan.speed(DepanKananPID.compute());
    DepanKiri.speed(DepanKiriPID.compute()*-1);
    while(t_jalan.read_ms()<=20)
    {
    }
    t_jalan.reset();
}

void jalan_kanan()
{
    t_jalan.reset();
    t_jalan.start();
    proses_kecepatan();
    Get_Count();
    Cari_Koordinat();
    pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
    pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
    BelakangKanan.speed(BelakangKananPID.compute()*-1);
    BelakangKiri.speed(BelakangKiriPID.compute()*-1);
    DepanKanan.speed(DepanKananPID.compute());
    DepanKiri.speed(DepanKiriPID.compute());
    while(t_jalan.read_ms()<=20)
    {
    }
    t_jalan.reset();
}

void jalan_kiri()
{
    t_jalan.reset();
    t_jalan.start();
    proses_kecepatan();
    Get_Count();
    Cari_Koordinat();
    pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
    pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
    BelakangKanan.speed(BelakangKananPID.compute());
    BelakangKiri.speed(BelakangKiriPID.compute());
    DepanKanan.speed(DepanKananPID.compute()*-1);
    DepanKiri.speed(DepanKiriPID.compute()*-1);
    while(t_jalan.read_ms()<=20)
    {
    }
    t_jalan.reset();
}

void jalan_rotasi_kanan()
{
    t_jalan.reset();
    t_jalan.start();
    proses_kecepatan();
    Get_Count();
    Cari_Koordinat();
    //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
    //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
    pc.printf("Orientasi : %d\n", orientasi_2);
    BelakangKanan.speed(BelakangKananPID.compute());
    BelakangKiri.speed(BelakangKiriPID.compute());
    DepanKanan.speed(DepanKananPID.compute());
    DepanKiri.speed(DepanKiriPID.compute());
    while(t_jalan.read_ms()<=20)
    {
    }
    t_jalan.reset();
}

void jalan_rotasi_kiri()
{
    t_jalan.reset();
    t_jalan.start();
    proses_kecepatan();
    Get_Count();
    Cari_Koordinat();
    //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
    //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
    pc.printf("Orientasi : %d\n", orientasi_2);
    BelakangKanan.speed(BelakangKananPID.compute()*-1);
    BelakangKiri.speed(BelakangKiriPID.compute()*-1);
    DepanKanan.speed(DepanKananPID.compute()*-1);
    DepanKiri.speed(DepanKiriPID.compute()*-1);
    while(t_jalan.read_ms()<=20)
    {
    }
    t_jalan.reset();
}

void berhenti()
{
    t_jalan.reset();
    t_jalan.start();
    //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
    //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
    pc.printf("Orientasi : %d\n", orientasi_2);
    DepanKanan.brake(0);
    BelakangKanan.brake(0);
    BelakangKiri.brake(0);
    DepanKiri.brake(0);
    while(t_jalan.read_ms()<=20)
    {
    }
    t_jalan.reset();
}

void ke_setpoint_x()
{
    if(koordinat_x_bulat<setpoint_x)
    {
        kosongkan();
        while(koordinat_x_bulat<setpoint_x)
        {
            jalan_kanan();
        }
        DepanKanan.brake();
        BelakangKanan.brake();
        BelakangKiri.brake();
        DepanKiri.brake();
    }
    else if(koordinat_x_bulat>setpoint_x)
    {
        kosongkan();
        while(koordinat_x_bulat>setpoint_x)
        {
            jalan_kiri();
        }
        DepanKanan.brake();
        BelakangKanan.brake();
        BelakangKiri.brake();
        DepanKiri.brake();
    }
    else if(koordinat_x_bulat==setpoint_x)
    {
        berhenti();
    }
}

void ke_setpoint_y()
{
    if(koordinat_y_bulat<setpoint_y)
    {
        kosongkan();
        while(koordinat_y_bulat<setpoint_y)
        {
            jalan_maju();
        }
        DepanKanan.brake();
        BelakangKanan.brake();
        BelakangKiri.brake();
        DepanKiri.brake();
    }
    else if(koordinat_y_bulat>setpoint_y)
    {
        kosongkan();
        while(koordinat_y>setpoint_y)
        {
            jalan_mundur();
        }
        DepanKanan.brake();
        BelakangKanan.brake();
        BelakangKiri.brake();
        DepanKiri.brake();
    }
    else if(koordinat_y_bulat==setpoint_y)
    {
        berhenti();
    }
}

void mode_cepat()
{
    target_depan_kanan = 1800.0f;
    target_depan_kiri = 1800.0f;
    target_belakang_kanan = 1800.0f;
    target_belakang_kiri = 1800.0f;
}

void mode_lambat()
{
    target_depan_kanan = 1000.0f;
    target_depan_kiri = 1000.0f;
    target_belakang_kanan = 1000.0f;
    target_belakang_kiri = 1000.0f;
}


void ke_target_orientasi()
{
    if(orientasi_2>target_orientasi)
    {
        kosongkan();
        while(orientasi_2<target_orientasi)
        {
            jalan_rotasi_kiri();
        }
        DepanKanan.brake();
        BelakangKanan.brake();
        BelakangKiri.brake();
        DepanKiri.brake();
    }
    else if(orientasi_2<target_orientasi)
    {
        kosongkan();
        while(orientasi_2>target_orientasi)
        {
            jalan_rotasi_kanan();
        }
        DepanKanan.brake();
        BelakangKanan.brake();
        BelakangKiri.brake();
        DepanKiri.brake();
    }
    else if(orientasi_2==target_orientasi)
    {
        berhenti();
    }
}

void ke_target_orientasi2()
{
    while(orientasi_2<target_orientasi)
    {
        jalan_rotasi_kiri();
        if(orientasi_2 == target_orientasi)
        {
            berhenti();
        }
    }
    while(orientasi_2>target_orientasi)
    {
        jalan_rotasi_kanan();
        if(orientasi_2 == target_orientasi)
        {
            berhenti();
        }
    }
}

void jalan_inverse()
{
    if((arah_depan_kiri==0)&&(arah_belakang_kanan==0)==1)
    {
        BelakangKiri.speed(BelakangKiriPID.compute()*arah_belakang_kiri);
        DepanKanan.speed(DepanKananPID.compute()*arah_depan_kanan);
    }
    else if((arah_depan_kanan==0)&&(arah_belakang_kiri==0)==1)
    {
        BelakangKanan.speed(BelakangKiriPID.compute()*arah_belakang_kanan);
        DepanKiri.speed(DepanKananPID.compute()*arah_depan_kiri);
    }
    else if(arah_depan_kiri!=0)
    {
        if(arah_depan_kanan!=0)
        {
            BelakangKiri.speed(BelakangKiriPID.compute()*arah_belakang_kiri);
            DepanKanan.speed(DepanKananPID.compute()*arah_depan_kanan);
            BelakangKanan.speed(BelakangKiriPID.compute()*arah_belakang_kanan);
            DepanKiri.speed(DepanKananPID.compute()*arah_depan_kiri);
        }
    }
}

void cari_koin1()
{
    target_orientasi = 180;
    ke_target_orientasi();
    while(jarak>=23)
    {
        //jarak  = ultra.read();
        jalan_maju();
    }
    //while(jarak<=20)
    //{
    //   jalan_maju();
    //}
    //while(eksistensi_koin!=0)
    //{
    //    jalan_kanan();
    //}
}

float sensor(){
   float jarak2; 
   int durasi;
   trigPin1=1;
   t.reset();
    wait_us(10);
   trigPin1=0;
   while (echoPin1 == 0){    
    }t.start();
   while (echoPin1 == 1)
   {
    }
    t.stop();
   durasi = t.read_us();
   jarak2 = (double)(durasi) / 29.0f /2.0f ; // 29 kecepatan suara di udara (2.9 detik untuk 1 kilometer), dibagi 2 karena bolak-balik
   return jarak2;     
}

void cari_koin2()
{
    while(orientasi != 180)
    {
        ke_target_orientasi();
    }
    while(jarak>=23)
    {
        //jarak = ultra.read();
        jalan_maju();
    }
    //while(eksistensi_koin!=0)
    //{
    //    jalan_kanan();
    //}
}

int main()
{   
    pc.baud(38400);
    bluetooth.baud(9600);
    bluetooth.attach(&fungsi_bluetooth, Serial::RxIrq);
    
    BEARING_Register = 0x02;    
    DepanKanan.period(0.01f);    
    BelakangKanan.period(0.01f);
    BelakangKiri.period(0.01f);  //Set motor PWM periods to 20KHz.
    DepanKiri.period(0.01f);

    DepanKananPID.setInputLimits(0, 3000);//Input  units: counts per second.
    DepanKananPID.setOutputLimits(0.0, 0.9);//Output units: PwmOut duty cycle as %.
    DepanKananPID.setMode(AUTO_MODE);
    
    BelakangKananPID.setInputLimits(0, 3000);
    BelakangKananPID.setOutputLimits(0.0, 0.9);
    BelakangKananPID.setMode(AUTO_MODE);
    
    BelakangKiriPID.setInputLimits(0, 3000);
    BelakangKiriPID.setOutputLimits(0.0, 0.9);
    BelakangKiriPID.setMode(AUTO_MODE);
    
    DepanKiriPID.setInputLimits(0, 3000);
    DepanKiriPID.setOutputLimits(0.0, 0.9);
    DepanKiriPID.setMode(AUTO_MODE);
    
    DepanKananPID.setSetPoint(1400);
    BelakangKananPID.setSetPoint(1400);
    BelakangKiriPID.setSetPoint(1400);
    DepanKiriPID.setSetPoint(1400);
    //sudut_awal = Get_Yaw(); 
    while(1)
    {
        
        if(Jalan == Jalan_Maju)
        {
            DepanKananPID.setSetPoint(target_depan_kanan);
            BelakangKananPID.setSetPoint(target_belakang_kanan);
            BelakangKiriPID.setSetPoint(target_belakang_kiri);
            DepanKiriPID.setSetPoint(target_depan_kiri);
            kosongkan();
            while(Jalan== Jalan_Maju)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute()*-1);
                BelakangKiri.speed(BelakangKiriPID.compute());
                DepanKanan.speed(DepanKananPID.compute()*-1);
                DepanKiri.speed(DepanKiriPID.compute());
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Jalan_Mundur)
        {
            DepanKananPID.setSetPoint(target_depan_kanan);
            BelakangKananPID.setSetPoint(target_belakang_kanan);
            BelakangKiriPID.setSetPoint(target_belakang_kiri);
            DepanKiriPID.setSetPoint(target_depan_kiri);
            kosongkan();
            while(Jalan== Jalan_Mundur)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute());
                BelakangKiri.speed(BelakangKiriPID.compute()*-1);
                DepanKanan.speed(DepanKananPID.compute());
                DepanKiri.speed(DepanKiriPID.compute()*-1);
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Jalan_Kiri)
        {
            DepanKananPID.setSetPoint(target_depan_kanan);
            BelakangKananPID.setSetPoint(target_belakang_kanan);
            BelakangKiriPID.setSetPoint(target_belakang_kiri);
            DepanKiriPID.setSetPoint(target_depan_kiri);
            kosongkan();
            while(Jalan== Jalan_Kiri)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute());
                BelakangKiri.speed(BelakangKiriPID.compute());
                DepanKanan.speed(DepanKananPID.compute()*-1);
                DepanKiri.speed(DepanKiriPID.compute()*-1);
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        else if(Jalan == Jalan_Kanan)
        {
            DepanKananPID.setSetPoint(target_depan_kanan);
            BelakangKananPID.setSetPoint(target_belakang_kanan);
            BelakangKiriPID.setSetPoint(target_belakang_kiri);
            DepanKiriPID.setSetPoint(target_depan_kiri);
            kosongkan();
            while(Jalan== Jalan_Kanan)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute()*-1);
                BelakangKiri.speed(BelakangKiriPID.compute()*-1);
                DepanKanan.speed(DepanKananPID.compute());
                DepanKiri.speed(DepanKiriPID.compute());
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake(1);
            BelakangKanan.brake(1);
            BelakangKiri.brake(1);
            DepanKiri.brake(1);
        }
        
        else if(Jalan == Rotasi_Kanan)
        {
            DepanKananPID.setSetPoint(target_depan_kanan-800.0f);
            BelakangKananPID.setSetPoint(target_belakang_kanan-800.0f);
            BelakangKiriPID.setSetPoint(target_belakang_kiri-800.0f);
            DepanKiriPID.setSetPoint(target_depan_kiri-800.0f);
            kosongkan();
            while(Jalan== Rotasi_Kanan)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute());
                BelakangKiri.speed(BelakangKiriPID.compute());
                DepanKanan.speed(DepanKananPID.compute());
                DepanKiri.speed(DepanKiriPID.compute());
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Rotasi_Kiri)
        {
            DepanKananPID.setSetPoint(target_depan_kanan-800.0f);
            BelakangKananPID.setSetPoint(target_belakang_kanan-800.0f);
            BelakangKiriPID.setSetPoint(target_belakang_kiri-800.0f);
            DepanKiriPID.setSetPoint(target_depan_kiri-800.0f);
            kosongkan();
            while(Jalan== Rotasi_Kiri)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute()*-1);
                BelakangKiri.speed(BelakangKiriPID.compute()*-1);
                DepanKanan.speed(DepanKananPID.compute()*-1);
                DepanKiri.speed(DepanKiriPID.compute()*-1);
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Belok_Kanan)
        {
            DepanKananPID.setSetPoint(target_depan_kanan-1000.0f);
            BelakangKananPID.setSetPoint(target_belakang_kanan-1000.0f);
            BelakangKiriPID.setSetPoint(target_belakang_kiri);
            DepanKiriPID.setSetPoint(target_depan_kiri);
            kosongkan();
            while(Jalan== Belok_Kanan)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute()*-1);
                BelakangKiri.speed(BelakangKiriPID.compute());
                DepanKanan.speed(DepanKananPID.compute()*-1);
                DepanKiri.speed(DepanKiriPID.compute());
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Mundur_Belok_Kanan)
        {
            DepanKananPID.setSetPoint(target_depan_kanan-1000.0f);
            BelakangKananPID.setSetPoint(target_belakang_kanan-1000.0f);
            BelakangKiriPID.setSetPoint(target_belakang_kiri);
            DepanKiriPID.setSetPoint(target_depan_kiri);
            kosongkan();
            while(Jalan== Mundur_Belok_Kanan)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute());
                BelakangKiri.speed(BelakangKiriPID.compute()*-1);
                DepanKanan.speed(DepanKananPID.compute());
                DepanKiri.speed(DepanKiriPID.compute()*-1);
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Mundur_Belok_Kiri)
        {
            DepanKananPID.setSetPoint(target_depan_kanan);
            BelakangKananPID.setSetPoint(target_belakang_kanan);
            BelakangKiriPID.setSetPoint(target_belakang_kiri-1000.0f);
            DepanKiriPID.setSetPoint(target_depan_kiri-1000.0f);
            kosongkan();
            while(Jalan== Mundur_Belok_Kiri)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute());
                BelakangKiri.speed(BelakangKiriPID.compute()*-1);
                DepanKanan.speed(DepanKananPID.compute());
                DepanKiri.speed(DepanKiriPID.compute()*-1);
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Belok_Kiri)
        {
            DepanKananPID.setSetPoint(target_depan_kanan);
            BelakangKananPID.setSetPoint(target_belakang_kanan);
            BelakangKiriPID.setSetPoint(target_belakang_kiri-1000);
            DepanKiriPID.setSetPoint(target_depan_kiri-1000);
            kosongkan();
            while(Jalan== Belok_Kiri)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute()*-1);
                BelakangKiri.speed(BelakangKiriPID.compute());
                DepanKanan.speed(DepanKananPID.compute()*-1);
                DepanKiri.speed(DepanKiriPID.compute());
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Muter_Kanan)
        {
            //DepanKananPID.setSetPoint(1400);
            BelakangKananPID.setSetPoint(1400);
            //BelakangKiriPID.setSetPoint(1400);
            DepanKiriPID.setSetPoint(1400);
            kosongkan();
            while(Jalan== Muter_Kanan)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                BelakangKanan.speed(BelakangKananPID.compute()*-1);
                //BelakangKiri.speed(BelakangKiriPID.compute()*-1);
                //DepanKanan.speed(DepanKananPID.compute());
                DepanKiri.speed(DepanKiriPID.compute());
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Muter_Kiri)
        {
            DepanKananPID.setSetPoint(1400);
            //BelakangKananPID.setSetPoint(1400);
            BelakangKiriPID.setSetPoint(1400);
            //DepanKiriPID.setSetPoint(1400);
            kosongkan();
            while(Jalan== Muter_Kiri)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %f\n", orientasi);
                //BelakangKanan.speed(BelakangKananPID.compute());
                BelakangKiri.speed(BelakangKiriPID.compute()*1);
                DepanKanan.speed(DepanKananPID.compute()*-1);
                //DepanKiri.speed(DepanKiriPID.compute()*-1);
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();
            }
            DepanKanan.brake();
            BelakangKanan.brake();
            BelakangKiri.brake();
            DepanKiri.brake();
        }
        
        else if(Jalan == Ganti_Kecepatan)
        {
            if(kecepatan_state==0)
            {
                mode_lambat();
                wait_ms(200);
                kecepatan_state =1;
            }
            else if(kecepatan_state==1)
            {
                mode_cepat();
                wait_ms(200);
                kecepatan_state = 0;
            }
        }
        
        else if(Jalan == ke_setpoint)
        {
            DepanKananPID.setSetPoint(1400);
            BelakangKananPID.setSetPoint(1400);
            BelakangKiriPID.setSetPoint(1400);
            DepanKiriPID.setSetPoint(1400);
            //while(Jalan == ke_setpoint)
            //{
                //ke_setpoint_x();
                //ke_setpoint_y();
                ke_target_orientasi();
            //}
        }
        else if(Jalan == mode_inverse)
        {
            kosongkan();
            cari_inverse(orientasi, (float)setpoint_x, (float)setpoint_y);
            set_inverse();
            while(Jalan == mode_inverse)
            {
                if((koordinat_x_bulat!=setpoint_x)&&(koordinat_y_bulat!=setpoint_y))
                {
                    t_jalan.reset();
                    t_jalan.start();
                    proses_kecepatan();
                    Get_Count();
                    Cari_Koordinat();
                    if(arah_depan_kanan!=0)
                    {
                        if(arah_depan_kiri==0)
                        {
                            BelakangKiri.speed(BelakangKiriPID.compute()*arah_belakang_kiri);
                            DepanKanan.speed(DepanKananPID.compute()*arah_depan_kanan);
                            BelakangKanan.brake(0);
                            DepanKiri.brake(0);
                        }
                        else if(arah_depan_kiri!=0)
                        {
                            BelakangKiri.speed(BelakangKiriPID.compute()*arah_belakang_kiri);
                            DepanKanan.speed(DepanKananPID.compute()*arah_depan_kanan);
                            BelakangKanan.speed(BelakangKananPID.compute()*arah_belakang_kanan);
                            DepanKiri.speed(DepanKiriPID.compute()*arah_depan_kiri);
                        }
                    }
                    else if(arah_depan_kiri!=0)
                    {
                        if(arah_depan_kanan==0)
                        {
                            BelakangKanan.speed(BelakangKananPID.compute()*arah_belakang_kanan);
                            DepanKiri.speed(DepanKiriPID.compute()*arah_depan_kiri);
                            BelakangKiri.brake(0);
                            DepanKanan.brake(0);
                        }
                        else if(arah_depan_kanan!=0)
                        {
                            BelakangKiri.speed(BelakangKiriPID.compute()*arah_belakang_kiri);
                            DepanKanan.speed(DepanKananPID.compute()*arah_depan_kanan);
                            BelakangKanan.speed(BelakangKananPID.compute()*arah_belakang_kanan);
                            DepanKiri.speed(DepanKiriPID.compute()*arah_depan_kiri);
                        }
                    }
                    pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                    pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                    pc.printf("Orientasi : %d\n", orientasi_2);
                    //pc.printf("X basis : %f\n", target_x_basis);
                    //pc.printf("Y basis : %f\n", target_y_basis);
                    //pc.printf("Depan Kiri : %f\n", target_depan_kiri);
                    //pc.printf("Depan Kanan : %f\n", target_depan_kanan);
                    //pc.printf("Belakang Kanan : %f\n", target_belakang_kanan);
                    //pc.printf("Belakang Kiri : %f\n", target_belakang_kiri);
                    while(t_jalan.read_ms()<=20)
                    {
                    }
                    t_jalan.reset();
                }
            }
        }
        else if(Jalan == Capit_Buka)
        {
            if(capit_state==0)
            {
                Servo.pulsewidth_us(1230);
                capit_state=1;
                wait_ms(200);
            }
            else if(capit_state==1)
            {
                Servo.pulsewidth_us(2400);
                capit_state=0;
                wait_ms(200);
            }
            /*else if(capit_state==2)
            {
                Servo.pulsewidth_us(1500);
                capit_state=0;
                wait_ms(200);
            }
            */
        }
        else if(Jalan == Capit_Muter)
        {
            if(capit2_state==0)
            {
                for(i=1800; i>910; i--)
                {
                    t_capit.reset();
                    t_capit.start();
                    Servo2.pulsewidth_us(i);
                    while(t_capit.read_ms()<1)
                    {
                    }
                    t_capit.stop();
                }
                capit2_state=1;
            }
            else if(capit2_state==1)
            {
                for(i=910; i<1800; i=i++)
                {
                    t_capit.reset();
                    t_capit.start();
                    Servo2.pulsewidth_us(i);
                    while(t_capit.read_us()<600)
                    {
                    }
                    t_capit.stop();
                }
                capit2_state=0;
            }
        }
        else if(Jalan == Cari_Koin1)
        {
            DepanKananPID.setSetPoint(1000);
            BelakangKananPID.setSetPoint(1000);
            BelakangKiriPID.setSetPoint(1000);
            DepanKiriPID.setSetPoint(1000);
            eksistensi_koin = ir.read();
            jarak = sensor();
            while(Jalan == Cari_Koin1)
            {
                //target_orientasi = 180;
                //ke_target_orientasi2();
                Servo.pulsewidth_us(2290);
                wait_ms(100);
                while(jarak>16.0f)
                {
                    //jalan_maju();
                    jarak = sensor();
                    jalan_maju();
                } 
                berhenti();
                wait_ms(300);    
                while(eksistensi_koin== tidak_ada)
                {
                    eksistensi_koin = ir.read();
                    jalan_kiri();
                }
                berhenti();
                wait_ms(300);
                Servo.pulsewidth_us(1190);
                wait_ms(100);
                t2.reset();
                t2.start();
                while(t2.read_ms()<1500)
                {
                    jalan_mundur();
                }
                t2.stop();
                //while(jarak<90)
                //{
                //    jalan_mundur();
                //    jarak = sensor();
                //}
                //Servo2.pulsewidth_us(2200);
                //target_orientasi = 0;
                //ke_target_orientasi2();   
            }
        }
        else if(Jalan == Cari_Koin2)
        {            
            while(Jalan == Cari_Koin2)
            {
                cari_koin2();
            }
        }
        else if(Jalan == Berhenti)
        {
            kosongkan();
            while(Jalan==Berhenti)
            {
                t_jalan.reset();
                t_jalan.start();
                proses_kecepatan();
                Get_Count();
                Cari_Koordinat();
                //pc.printf("Koordinat X : %d\n", koordinat_x_bulat);
                //pc.printf("Koordinat Y : %d\n", koordinat_y_bulat);
                pc.printf("Orientasi : %d\n", orientasi_2);
                DepanKanan.brake(0);
                BelakangKanan.brake(0);
                BelakangKiri.brake(0);
                DepanKiri.brake(0);
                while(t_jalan.read_ms()<=20)
                {
                }
                t_jalan.reset();    
            }
        }
    }                       
}