// imu_bruto.ino — Lab 3 de Robótica 2026, grupo 37
//
// Lê o ICM-20600 (Grove, endereço 0x69) diretamente por I2C e envia as
// contagens em bruto, a uma taxa fixa, pela porta série.
//
// Porque não se usa a biblioteca da Seeed (a do sketch da cadeira):
//   - getAccelerationX()/getGyroscopeX() devolvem inteiros em mg e em dps.
//     Com o giroscópio a ±2000 dps (o que initialize() deixa) a resolução é
//     1 dps, maior do que o desvio que queremos medir.
//   - getTemperature() lê o registo como sem sinal: abaixo de 25 °C dá lixo.
// Aqui só se usa Wire. A conversão para g, dps e °C faz-se no Python.
//
// Formato (texto, uma linha por amostra):
//   linhas começadas por '#' : cabeçalho (configuração)
//   t_us,ax,ay,az,temp,gx,gy,gz : tempo do Arduino em µs e contagens int16
//
// Conversão:  a [g]   = contagem / (32768 / ACC_FS_G)
//             w [dps] = contagem / (32768 / GYRO_FS_DPS)
//             T [°C]  = temp / 326.8 + 25

#include <Wire.h>

// ---------------------------------------------------------------- configuração
const long  BAUD        = 500000;  // 0 % de erro com o cristal de 16 MHz do Uno
const int   RATE_HZ     = 200;     // amostras por segundo
const int   ACC_FS_G    = 4;       // 2, 4, 8 ou 16
const int   GYRO_FS_DPS = 500;     // 250, 500, 1000 ou 2000
// Filtros internos: giroscópio 92 Hz, acelerómetro 99 Hz (abaixo de RATE_HZ/2)
const byte  DLPF_GYRO   = 2;       // registo CONFIG, DLPF_CFG
const byte  DLPF_ACC    = 2;       // registo ACCEL_CONFIG2, A_DLPF_CFG

// ---------------------------------------------------------------- registos
const byte REG_SMPLRT_DIV   = 0x19;
const byte REG_CONFIG       = 0x1A;
const byte REG_GYRO_CONFIG  = 0x1B;
const byte REG_ACCEL_CONFIG = 0x1C;
const byte REG_ACCEL_CONFIG2= 0x1D;
const byte REG_ACCEL_XOUT_H = 0x3B;  // 14 bytes seguidos: acc(6) temp(2) gyro(6)
const byte REG_PWR_MGMT_1   = 0x6B;
const byte REG_PWR_MGMT_2   = 0x6C;
const byte REG_WHO_AM_I     = 0x75;  // 0x11 no ICM-20600

byte addr = 0x69;
const unsigned long PERIODO_US = 1000000UL / RATE_HZ;
unsigned long proximo;

void escreve(byte reg, byte val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

byte le(byte reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (byte)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

byte codigoGama(int fs, int base) {   // 0..3 para base, 2·base, 4·base, 8·base
  byte c = 0;
  while (fs > base && c < 3) { fs /= 2; c++; }
  return c;
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);

  // Endereço: 0x69 (AD0 alto, como no sketch da cadeira); se não responder, 0x68.
  Wire.beginTransmission(0x69);
  if (Wire.endTransmission() != 0) addr = 0x68;

  escreve(REG_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  escreve(REG_PWR_MGMT_1, 0x01);  // acordado, relógio automático (PLL do giro)
  escreve(REG_PWR_MGMT_2, 0x00);  // todos os eixos ligados
  delay(50);
  escreve(REG_SMPLRT_DIV, 0x00);  // 1 kHz interno
  escreve(REG_CONFIG, DLPF_GYRO);
  escreve(REG_GYRO_CONFIG, codigoGama(GYRO_FS_DPS, 250) << 3);
  escreve(REG_ACCEL_CONFIG, codigoGama(ACC_FS_G, 2) << 3);
  escreve(REG_ACCEL_CONFIG2, DLPF_ACC);
  delay(100);                     // deixar os filtros assentar

  byte who = le(REG_WHO_AM_I);
  Serial.println(F("# imu_bruto v1 (Lab 3, grupo 37)"));
  Serial.print(F("# addr=0x")); Serial.print(addr, HEX);
  Serial.print(F(" whoami=0x")); Serial.println(who, HEX);
  Serial.print(F("# acc_fs_g=")); Serial.print(ACC_FS_G);
  Serial.print(F(" gyro_fs_dps=")); Serial.print(GYRO_FS_DPS);
  Serial.print(F(" rate_hz=")); Serial.print(RATE_HZ);
  Serial.print(F(" baud=")); Serial.println(BAUD);
  Serial.println(F("# t_us,ax,ay,az,temp,gx,gy,gz"));

  proximo = micros();
}

void loop() {
  // Espera pelo instante da próxima amostra (taxa fixa, sem deriva).
  while ((long)(micros() - proximo) < 0) { }
  unsigned long t = micros();
  proximo += PERIODO_US;
  if ((long)(t - proximo) > 0) {   // atrasou mais de um período: recomeça
    proximo = t + PERIODO_US;
  }

  Wire.beginTransmission(addr);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (byte)14);
  int16_t v[7];
  for (byte i = 0; i < 7; i++) {
    byte h = Wire.read();
    byte l = Wire.read();
    v[i] = (int16_t)((h << 8) | l);
  }

  Serial.print(t);
  for (byte i = 0; i < 7; i++) {
    Serial.print(',');
    Serial.print(v[i]);
  }
  Serial.println();
}
