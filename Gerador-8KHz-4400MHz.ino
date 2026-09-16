/*
  =====================================================================
  Gerador ADF4351 + Si5351 com ESP32 + TFT ST7735 + Encoder Rotativo
  =====================================================================
  Cobre de poucos kHz (Si5351) até 4,4 GHz (ADF4351), com troca
  automática de chip por histerese e chave de RF externa (relé/PIN
  diode) comandada por GPIO.

  Controla:
    - Frequência de saída (RFOUT) - única faixa editável, chip decidido
      automaticamente
    - Potência (ADF, em dBm) ou corrente de drive (Si5351, em mA)
    - Frequência de referência de cada módulo, separadamente (página 2)

  Bibliotecas necessárias (Library Manager):
    - Adafruit GFX Library
    - Adafruit ST7735 and ST7789 Library
    - Etherkit Si5351 (Si5351Arduino)

  Registradores do ADF4351 calculados conforme datasheet oficial
  (Analog Devices ADF4351, Rev. A) - mapa de bits conferido registro
  a registro (R0..R5).

  -------------------- LIGAÇÕES SUGERIDAS (ESP32 DevKit) --------------
  ADF4351      ESP32
  LE       ->  GPIO26
  CLK      ->  GPIO18 (SPI SCK - compartilhado com o TFT)
  DATA     ->  GPIO23 (SPI MOSI - compartilhado com o TFT)
  CE       ->  GPIO27 (mantido em HIGH pelo firmware)
  MUXOUT   ->  GPIO34 (lock detect digital, pino só-entrada)
  GND      ->  GND

  Si5351       ESP32
  SDA      ->  GPIO21
  SCL      ->  GPIO22
  VCC      ->  3V3
  GND      ->  GND

  Chave de RF (relé/PIN diode) ESP32
  CTRL     ->  GPIO5  (HIGH = seleciona ADF4351, LOW = seleciona Si5351)

  TFT ST7735   ESP32
  CS       ->  GPIO15
  DC       ->  GPIO2
  RST      ->  GPIO4
  SCK      ->  GPIO18
  MOSI(SDA)->  GPIO23
  VCC      ->  3V3
  GND      ->  GND
  LED      ->  3V3 (ou GPIO com PWM se quiser controlar o brilho)

  Encoder      ESP32
  CLK      ->  GPIO32
  DT       ->  GPIO33
  SW       ->  GPIO25
  +        ->  3V3
  GND      ->  GND
  =====================================================================
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Preferences.h>
#include <si5351.h>

struct PLLParams {
  uint32_t INT;
  uint32_t FRAC;
  uint8_t  outDivSel;   // 0=/1, 1=/2, 2=/4, 3=/8, 4=/16, 5=/32, 6=/64
  uint8_t  prescaler;   // 0 = 4/5 , 1 = 8/9
};

enum Modo { MODO_FREQ, MODO_POTENCIA, MODO_REF_ADF, MODO_REF_SI5351, MODO_COUNT };
enum ChipAtivo { CHIP_SI5351, CHIP_ADF };

// ------------------------- PINOS --------------------------------
#define ADF_LE       26
#define ADF_CE       27
#define ADF_MUXOUT   34   // lock detect

#define TFT_CS       15
#define TFT_DC        2
#define TFT_RST       4

#define ENC_CLK      32
#define ENC_DT       33
#define ENC_SW       25

#define RF_SW_PIN     5   // chave de RF: HIGH=ADF4351, LOW=Si5351

// ------------------------- OBJETOS --------------------------------
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Preferences prefs;
Si5351 si5351;

// ------------------------- ENCODER --------------------------------

volatile int32_t encoderDelta = 0;

#define R_START     0x0
#define R_CW_FINAL  0x1
#define R_CW_BEGIN  0x2
#define R_CW_NEXT   0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT  0x6
#define DIR_CW      0x10
#define DIR_CCW     0x20

static const uint8_t tabelaEncoder[7][4] = {
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

volatile uint8_t estadoEncoder = R_START;

void IRAM_ATTR encoderISR() {
  uint8_t pinstate = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);
  estadoEncoder = tabelaEncoder[estadoEncoder & 0x0F][pinstate];
  uint8_t direcao = estadoEncoder & 0x30;

  if (direcao == DIR_CW) {
    encoderDelta++;
  } else if (direcao == DIR_CCW) {
    encoderDelta--;
  }
}

// ------------------------- ESTADO DA UI --------------------------------
// Página 1 (FREQ, POTENCIA): frequência + potência/drive, chip decidido
//   automaticamente por faixa.
// Página 2 (REF_ADF, REF_SI5351): referência de cada módulo, separada.
Modo modoAtual = MODO_FREQ;

inline uint8_t paginaDoModo(Modo m) {
  return (m == MODO_FREQ || m == MODO_POTENCIA) ? 0 : 1;
}

double freqHz          = 1000000000.0;  // 1000 MHz - frequência inicial
double refFreqHz       = 25000000.0;    // 25 MHz - referência do ADF4351
double refFreqSi5351Hz = 25000000.0;    // 25 MHz - referência do Si5351
uint8_t potIndex         = 3;  // índice em tabelaPotencia[] (ADF, dBm)
uint8_t potIndexSi5351   = 3;  // índice em tabelaDriveSi5351[] (Si5351, mA)

const int32_t tabelaPotencia[4] = { -4, -1, 2, 5 }; // dBm (ADF4351)
const uint16_t tabelaDriveMA[4] = { 2, 4, 6, 8 };    // mA (Si5351, só pra exibição)
const si5351_drive tabelaDriveSi5351[4] = {
  SI5351_DRIVE_2MA, SI5351_DRIVE_4MA, SI5351_DRIVE_6MA, SI5351_DRIVE_8MA
};

// Passos de sintonia (ciclados com clique curto do botão)
const double passosFreq[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};
const uint8_t NUM_PASSOS_FREQ = sizeof(passosFreq) / sizeof(passosFreq[0]);
uint8_t indicePassoFreq = 5; // começa em 100 kHz

const double passosRef[] = {100, 1000, 10000, 100000, 1000000};
const uint8_t NUM_PASSOS_REF = sizeof(passosRef) / sizeof(passosRef[0]);
uint8_t indicePassoRef      = 2; // referência do ADF - começa em 10 kHz
uint8_t indicePassoRefSi    = 2; // referência do Si5351 - começa em 10 kHz

bool precisaRecalcular = true;
bool precisaRedesenharTudo = true;
bool ultimoLock = false;

// ------------------------- CHIP: ADF4351 OU ADF4350 --------------------------------
// Os registradores (R0..R5) são idênticos nos dois chips (pin/software compatível).
// Só mudam os limites de divisor de saída e o limiar do prescaler 8/9.
#define CHIP_ADF4351   0   // deixe 1 para ADF4351, mude para 0 se estiver usando o ADF4350

#if CHIP_ADF4351
  const uint8_t  MAX_DIV_SEL       = 6;          // ADF4351: até /64
  const double   FREQ_MIN_HZ       = 35000000.0; // 34,375 MHz na prática, 35 MHz de margem
  const double   PRESCALER_LIMIAR  = 3600000000.0; // acima disso usa prescaler 8/9
#else
  const uint8_t  MAX_DIV_SEL       = 4;          // ADF4350: até /16
  const double   FREQ_MIN_HZ       = 140000000.0; // 137,5 MHz é o limite do datasheet, mas medido na prática só fica estável acima de 140 MHz
  const double   PRESCALER_LIMIAR  = 3000000000.0; // acima disso usa prescaler 8/9
#endif

// ------------------------- SELEÇÃO DE CHIP (HISTERESE) --------------------------------
// Faixa de sobreposição entre os dois chips: ADF4351/4350 desce até FREQ_MIN_HZ,
// Si5351 sobe a ~150-160 MHz. Troca com histerese pra não "cascatear" a chave de
// RF perto do limiar - e sempre com uma margem de segurança acima do mínimo real
// do chip ADF configurado (evita cair numa faixa onde ele ainda não está estável).
ChipAtivo chipAtivo = CHIP_ADF;

// A descida (ADF->Si5351) usa o mínimo real medido (FREQ_MIN_HZ). A subida
// (Si5351->ADF) usa uma margem pequena acima disso, só pra criar uma banda
// de histerese estreita e evitar chaveamento repetido bem em cima do limiar.
const double LARGURA_HISTERESE_HZ = 5000000.0; // 5 MHz
const double LIMIAR_BAIXO_HZ = FREQ_MIN_HZ;                          // abaixo disso, força Si5351
const double LIMIAR_ALTO_HZ  = FREQ_MIN_HZ + LARGURA_HISTERESE_HZ;   // acima disso, força ADF

const double FREQ_MIN_ABS_HZ = 8000.0;        // 8 kHz - mínimo prático do Si5351
const double FREQ_MAX_ABS_HZ = 4400000000.0;  // 4,4 GHz - máximo do ADF4351

ChipAtivo selecionarChip(double freq, ChipAtivo atual) {
  if (atual == CHIP_ADF) {
    return (freq < LIMIAR_BAIXO_HZ) ? CHIP_SI5351 : CHIP_ADF;
  } else {
    return (freq > LIMIAR_ALTO_HZ) ? CHIP_ADF : CHIP_SI5351;
  }
}

void definirChaveRF(ChipAtivo c) {
  digitalWrite(RF_SW_PIN, c == CHIP_ADF ? HIGH : LOW);
}

// ------------------------- CÁLCULO DO PLL --------------------------------
const uint32_t MOD_VAL = 4000; // resolução de canal = refFreqHz / MOD_VAL

bool calcularPLL(double rfOutHz, double refHz, PLLParams &p) {
  if (rfOutHz < FREQ_MIN_HZ || rfOutHz > 4400000000.0) return false;
  if (refHz < 1000000.0 || refHz > 60000000.0) return false;

  double vco = rfOutHz;
  uint8_t divSel = 0;
  while (vco < 2200000000.0 && divSel < MAX_DIV_SEL) {
    vco *= 2.0;
    divSel++;
  }
  if (vco > 4400000000.0) return false;

  uint8_t presc = (vco > PRESCALER_LIMIAR) ? 1 : 0;

  double pfd = refHz; // R = 1, sem dobrador, sem divisor por 2
  double N = vco / pfd;
  uint32_t intVal = (uint32_t)N;
  double fracF = (N - intVal) * (double)MOD_VAL;
  uint32_t fracVal = (uint32_t)(fracF + 0.5);
  if (fracVal >= MOD_VAL) {
    fracVal = 0;
    intVal++;
  }

  uint32_t minInt = presc ? 75 : 23;
  if (intVal < minInt || intVal > 65535) return false;

  p.INT = intVal;
  p.FRAC = fracVal;
  p.outDivSel = divSel;
  p.prescaler = presc;
  return true;
}

// ------------------------- REGISTRADORES --------------------------------
uint32_t regs[6];

void montarRegistradores(const PLLParams &p, uint8_t codigoPotencia, bool saidaHabilitada) {
  // R0: control=000 -> INT[30:15] FRAC[14:3]
  regs[0] = ((p.INT & 0xFFFFUL) << 15) | ((p.FRAC & 0xFFFUL) << 3) | 0UL;

  // R1: control=001 -> Prescaler[27] Phase[26:15]=1 MOD[14:3]
  regs[1] = ((uint32_t)p.prescaler << 27) | (1UL << 15) | ((MOD_VAL & 0xFFFUL) << 3) | 1UL;

  // R2: control=010
  //   Muxout[28:26] = 110 (digital lock detect)
  //   Ref doubler[25]=0, RDIV2[24]=0, R counter[23:14]=1
  //   Charge pump current[12:9] = 7 (2.50 mA @ Rset=5.1k)
  //   LDF[8]=0 (frac-N), LDP[7]=0 (10ns), PD polarity[6]=1 (positiva)
  {
    const uint8_t muxout = 0b110;
    const uint8_t chargePump = 7;
    const uint32_t rCounter = 1;
    regs[2] = ((uint32_t)muxout << 26)
            | (rCounter << 14)
            | ((uint32_t)chargePump << 9)
            | (1UL << 6)
            | 2UL;
  }

  // R3: control=011 -> ABP=0 (6ns, recomendado p/ fractional-N), clock divider off
  regs[3] = 3UL;

  // R4: control=100
  //   Feedback select[23]=1 (direto do VCO, combina com o cálculo do N)
  //   RF divider select[22:20] = outDivSel
  //   Band select clock divider[19:12] = 200 (valor seguro/padrão)
  //   RF output enable[5] = saidaHabilitada
  //   Output power[4:3] = codigoPotencia
  {
    const uint32_t bandSelClk = 200;
    regs[4] = (1UL << 23)
            | ((uint32_t)p.outDivSel << 20)
            | (bandSelClk << 12)
            | ((saidaHabilitada ? 1UL : 0UL) << 5)
            | ((uint32_t)(codigoPotencia & 0x3) << 3)
            | 4UL;
  }

  // R5: control=101 -> bits reservados fixos + LD pin mode = 01 (digital lock detect)
  regs[5] = (1UL << 22) | (1UL << 20) | (1UL << 19) | 5UL;
}

void adfEscreverRegistro(uint32_t valor) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(ADF_LE, LOW);
  SPI.transfer((valor >> 24) & 0xFF);
  SPI.transfer((valor >> 16) & 0xFF);
  SPI.transfer((valor >> 8) & 0xFF);
  SPI.transfer(valor & 0xFF);
  digitalWrite(ADF_LE, HIGH);
  delayMicroseconds(2);
  digitalWrite(ADF_LE, LOW);
  SPI.endTransaction();
}

void adfEnviarTudo() {
  // Ordem recomendada pelo datasheet: R5, R4, R3, R2, R1, R0
  adfEscreverRegistro(regs[5]);
  adfEscreverRegistro(regs[4]);
  adfEscreverRegistro(regs[3]);
  adfEscreverRegistro(regs[2]);
  adfEscreverRegistro(regs[1]);
  adfEscreverRegistro(regs[0]);
}

bool aplicarParametros() {
  // Guarda os últimos parâmetros válidos do ADF, pra poder remontar o
  // registrador R4 (só com a saída desligada) mesmo quando ele não é
  // o chip ativo no momento.
  static PLLParams ultimoPADF;
  static bool ultimoPADFValido = false;

  ChipAtivo novoChip = selecionarChip(freqHz, chipAtivo);
  if (novoChip != chipAtivo) {
    chipAtivo = novoChip;
    definirChaveRF(chipAtivo);
  }

  if (chipAtivo == CHIP_ADF) {
    PLLParams p;
    if (!calcularPLL(freqHz, refFreqHz, p)) return false;
    ultimoPADF = p;
    ultimoPADFValido = true;
    montarRegistradores(p, potIndex, true); // saída ligada - é o chip ativo
    adfEnviarTudo();

    si5351.output_enable(SI5351_CLK0, 0); // desliga a saída do Si5351 (inativo)
  } else {
    if (freqHz < FREQ_MIN_ABS_HZ || freqHz > 150000000.0) return false;
    // Reconfigura a referência do Si5351 caso tenha mudado (o método
    // init() recalcula os PLLs internos em função do cristal informado)
    si5351.init(SI5351_CRYSTAL_LOAD_8PF, (uint32_t)refFreqSi5351Hz, 0);
    si5351.set_freq((uint64_t)(freqHz * 100.0), SI5351_CLK0);
    si5351.drive_strength(SI5351_CLK0, tabelaDriveSi5351[potIndexSi5351]);
    si5351.output_enable(SI5351_CLK0, 1); // liga a saída (é o chip ativo)

    if (ultimoPADFValido) {
      montarRegistradores(ultimoPADF, potIndex, false); // saída desligada - ADF inativo
      adfEnviarTudo();
    }
    // Se o ADF nunca chegou a ser configurado ainda (recém-ligado, começando
    // pela faixa do Si5351), não há registrador válido pra reescrever - tudo
    // bem, ele ainda não travou em nenhuma frequência mesmo.
  }
  return true;
}

// ------------------------- PERSISTÊNCIA (NVS) --------------------------------
void salvarConfig() {
  prefs.begin("adf4351", false);
  prefs.putDouble("freq", freqHz);
  prefs.putDouble("ref", refFreqHz);
  prefs.putDouble("refSi", refFreqSi5351Hz);
  prefs.putUChar("pot", potIndex);
  prefs.putUChar("potSi", potIndexSi5351);
  prefs.end();
}

void carregarConfig() {
  prefs.begin("adf4351", true);
  freqHz          = prefs.getDouble("freq", freqHz);
  refFreqHz       = prefs.getDouble("ref", refFreqHz);
  refFreqSi5351Hz = prefs.getDouble("refSi", refFreqSi5351Hz);
  potIndex        = prefs.getUChar("pot", potIndex);
  potIndexSi5351  = prefs.getUChar("potSi", potIndexSi5351);
  prefs.end();
}

// ------------------------- INTERFACE (TFT) --------------------------------
#define COR_FUNDO      ST77XX_BLACK
#define COR_TITULO     ST77XX_CYAN
#define COR_LABEL      ST77XX_WHITE
#define COR_VALOR      ST77XX_YELLOW
#define COR_SELECIONADO ST77XX_GREEN
#define COR_LOCK_OK    ST77XX_GREEN
#define COR_LOCK_NOK   ST77XX_RED

// Formata um passo de sintonia com a unidade mais legível (Hz/kHz/MHz),
// sem casas decimais desnecessárias - ex: 1000000 -> "1 MHz", 100 -> "100 Hz"
void formatarPasso(double hz, char *buf, size_t bufLen) {
  if (hz >= 1e6) {
    snprintf(buf, bufLen, "%.0f MHz", hz / 1e6);
  } else if (hz >= 1e3) {
    snprintf(buf, bufLen, "%.0f kHz", hz / 1e3);
  } else {
    snprintf(buf, bufLen, "%.0f Hz", hz);
  }
}

void desenharEstatico() {
  tft.fillScreen(COR_FUNDO);
  tft.setTextWrap(false);

  tft.setTextColor(COR_TITULO);
  tft.setTextSize(1);
  tft.setCursor(4, 2);
  tft.print("ADF4351+Si5351 PU2SWX");

  tft.setTextColor(COR_LABEL);

  if (paginaDoModo(modoAtual) == 0) {
    // ---- Página 1: frequência + potência/drive ----
    tft.setCursor(4, 14);
    tft.print("Freq (MHz):");

    tft.setCursor(4, 48);
    tft.print("Potencia:");

    tft.setCursor(4, 62);
    tft.print("Chip ativo:");

    tft.setCursor(4, 76);
    tft.print("Passo:");
  } else {
    // ---- Página 2: referência de cada módulo ----
    tft.setCursor(4, 14);
    tft.print("Ref ADF:");

    tft.setCursor(4, 38);
    tft.print("Ref Si5351:");

    tft.setCursor(4, 62);
    tft.print("Passo:");
  }

  tft.setCursor(4, 104);
  tft.print("PLL:");
}

void desenharValores() {
  char buf[24];
  uint8_t pagina = paginaDoModo(modoAtual);

  if (pagina == 0) {
    // ---- Frequência ----
    // Parte grande (MHz.kHz) + parte pequena com os dígitos de Hz coladas
    // logo depois, pra dar resolução de 1 Hz sem ocupar muito espaço.
    tft.fillRect(4, 24, 160, 20, COR_FUNDO);
    tft.setCursor(4, 24);
    tft.setTextSize(2);
    tft.setTextColor(modoAtual == MODO_FREQ ? COR_SELECIONADO : COR_VALOR);

    int64_t totalHz  = (int64_t)(freqHz + 0.5);
    int64_t mhzParte = totalHz / 1000000LL;
    int64_t khzParte = (totalHz / 1000LL) % 1000LL;
    int64_t hzParte  = totalHz % 1000LL;
    char bufGrande[12], bufPequeno[6];
    snprintf(bufGrande, sizeof(bufGrande), "%lld.%03lld", (long long)mhzParte, (long long)khzParte);
    snprintf(bufPequeno, sizeof(bufPequeno), "%03lld", (long long)hzParte);

    tft.print(bufGrande);
    int16_t xDepoisGrande = tft.getCursorX();
    tft.setTextSize(1);
    tft.setCursor(xDepoisGrande, 24 + 6); // desce um pouco pra alinhar com o texto grande
    tft.print(bufPequeno);
    tft.print(" MHz");

    // ---- Potência / drive (depende do chip ativo) ----
    tft.fillRect(80, 48, 80, 12, COR_FUNDO);
    tft.setCursor(80, 48);
    tft.setTextSize(1);
    tft.setTextColor(modoAtual == MODO_POTENCIA ? COR_SELECIONADO : COR_VALOR);
    if (chipAtivo == CHIP_ADF) {
      tft.print(tabelaPotencia[potIndex]);
      tft.print(" dBm");
    } else {
      tft.print(tabelaDriveMA[potIndexSi5351]);
      tft.print(" mA");
    }

    // ---- Chip ativo ----
    tft.fillRect(80, 62, 80, 12, COR_FUNDO);
    tft.setCursor(80, 62);
    tft.setTextColor(COR_VALOR);
    tft.print(chipAtivo == CHIP_ADF ? "ADF4351" : "Si5351");

    // ---- Passo ----
    tft.fillRect(50, 76, 110, 12, COR_FUNDO);
    tft.setCursor(50, 76);
    tft.setTextColor(COR_VALOR);
    if (modoAtual == MODO_FREQ) {
      char bufPasso[16];
      formatarPasso(passosFreq[indicePassoFreq], bufPasso, sizeof(bufPasso));
      tft.print(bufPasso);
    } else {
      tft.print("--");
    }

    tft.fillRect(0, 90, 160, 10, COR_FUNDO);
    tft.setCursor(4, 90);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Editando: ");
    tft.print(modoAtual == MODO_FREQ ? "FREQUENCIA" : "POTENCIA");

  } else {
    // ---- Referência do ADF4351 ----
    tft.fillRect(80, 14, 80, 12, COR_FUNDO);
    tft.setCursor(80, 14);
    tft.setTextSize(1);
    tft.setTextColor(modoAtual == MODO_REF_ADF ? COR_SELECIONADO : COR_VALOR);
    dtostrf(refFreqHz / 1e6, 0, 4, buf);
    tft.print(buf);
    tft.print(" MHz");

    // ---- Referência do Si5351 ----
    tft.fillRect(90, 38, 70, 12, COR_FUNDO);
    tft.setCursor(90, 38);
    tft.setTextColor(modoAtual == MODO_REF_SI5351 ? COR_SELECIONADO : COR_VALOR);
    dtostrf(refFreqSi5351Hz / 1e6, 0, 4, buf);
    tft.print(buf);
    tft.print(" MHz");

    // ---- Passo ----
    tft.fillRect(50, 62, 110, 12, COR_FUNDO);
    tft.setCursor(50, 62);
    tft.setTextColor(COR_VALOR);
    char bufPasso[16];
    if (modoAtual == MODO_REF_ADF) {
      formatarPasso(passosRef[indicePassoRef], bufPasso, sizeof(bufPasso));
    } else {
      formatarPasso(passosRef[indicePassoRefSi], bufPasso, sizeof(bufPasso));
    }
    tft.print(bufPasso);

    tft.fillRect(0, 90, 160, 10, COR_FUNDO);
    tft.setCursor(4, 90);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Editando: ");
    tft.print(modoAtual == MODO_REF_ADF ? "REF ADF" : "REF SI5351");
  }
}

void desenharLock(bool travado) {
  tft.fillRect(40, 104, 100, 12, COR_FUNDO);
  tft.setCursor(40, 104);
  tft.setTextSize(1);
  tft.setTextColor(travado ? COR_LOCK_OK : COR_LOCK_NOK);
  tft.print(travado ? "LOCKED" : "SEM LOCK");
}

// ------------------------- BOTÃO (curto / longo) --------------------------------
bool botaoEstadoAnterior = HIGH;
unsigned long botaoTempoPressionado = 0;
bool botaoTratado = false;
const unsigned long TEMPO_PRESSAO_LONGA = 600; // ms

void tratarBotao() {
  bool estado = digitalRead(ENC_SW);

  if (botaoEstadoAnterior == HIGH && estado == LOW) {
    // borda de descida - início da pressão
    botaoTempoPressionado = millis();
    botaoTratado = false;
  }

  if (estado == LOW && !botaoTratado) {
    if (millis() - botaoTempoPressionado >= TEMPO_PRESSAO_LONGA) {
      // pressão longa: troca de modo (FREQ -> POTENCIA -> REF_ADF -> REF_SI5351 -> ...)
      uint8_t paginaAntiga = paginaDoModo(modoAtual);
      modoAtual = (Modo)((modoAtual + 1) % MODO_COUNT);
      salvarConfig();
      botaoTratado = true;
      if (paginaDoModo(modoAtual) != paginaAntiga) {
        desenharEstatico(); // layout muda entre página 1 e página 2
      }
      desenharValores();
    }
  }

  if (botaoEstadoAnterior == LOW && estado == HIGH) {
    // borda de subida - soltou o botão
    if (!botaoTratado) {
      // pressão curta: alterna o passo de sintonia
      if (modoAtual == MODO_FREQ) {
        indicePassoFreq = (indicePassoFreq + 1) % NUM_PASSOS_FREQ;
      } else if (modoAtual == MODO_REF_ADF) {
        indicePassoRef = (indicePassoRef + 1) % NUM_PASSOS_REF;
      } else if (modoAtual == MODO_REF_SI5351) {
        indicePassoRefSi = (indicePassoRefSi + 1) % NUM_PASSOS_REF;
      }
      precisaRedesenharTudo = false;
      desenharValores();
    }
  }

  botaoEstadoAnterior = estado;
}

// ------------------------- SETUP / LOOP --------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(ADF_LE, OUTPUT);
  digitalWrite(ADF_LE, LOW);
  pinMode(ADF_CE, OUTPUT);
  digitalWrite(ADF_CE, HIGH); // habilita o chip
  pinMode(ADF_MUXOUT, INPUT);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), encoderISR, CHANGE);

  pinMode(RF_SW_PIN, OUTPUT);

  SPI.begin(); // pinos padrão VSPI do ESP32: SCK=18, MISO=19, MOSI=23

  Wire.begin(); // pinos padrão do ESP32: SDA=21, SCL=22
  bool si5351Ok = si5351.init(SI5351_CRYSTAL_LOAD_8PF, (uint32_t)refFreqSi5351Hz, 0);
  if (!si5351Ok) {
    Serial.println("AVISO: Si5351 nao encontrado no barramento I2C!");
  }

  tft.initR(INITR_BLACKTAB); // ajuste para INITR_GREENTAB/INITR_144GREENTAB se a imagem sair deslocada
  tft.setRotation(3);

  carregarConfig();
  chipAtivo = (freqHz < LIMIAR_BAIXO_HZ) ? CHIP_SI5351 : CHIP_ADF;
  definirChaveRF(chipAtivo);
  desenharEstatico();
  desenharValores();

  precisaRecalcular = true;
}

void loop() {
  // ---- leitura do encoder ----
  int32_t delta;
  noInterrupts();
  delta = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (delta != 0) {
    switch (modoAtual) {
      case MODO_FREQ:
        freqHz += delta * passosFreq[indicePassoFreq];
        if (freqHz < FREQ_MIN_ABS_HZ) freqHz = FREQ_MIN_ABS_HZ;
        if (freqHz > FREQ_MAX_ABS_HZ) freqHz = FREQ_MAX_ABS_HZ;
        break;

      case MODO_POTENCIA: {
        // Ajusta a tabela correspondente ao chip que está ativo agora
        if (chipAtivo == CHIP_ADF) {
          int8_t novoIndex = (int8_t)potIndex + (delta > 0 ? 1 : -1);
          if (novoIndex < 0) novoIndex = 0;
          if (novoIndex > 3) novoIndex = 3;
          potIndex = (uint8_t)novoIndex;
        } else {
          int8_t novoIndex = (int8_t)potIndexSi5351 + (delta > 0 ? 1 : -1);
          if (novoIndex < 0) novoIndex = 0;
          if (novoIndex > 3) novoIndex = 3;
          potIndexSi5351 = (uint8_t)novoIndex;
        }
        break;
      }

      case MODO_REF_ADF:
        refFreqHz += delta * passosRef[indicePassoRef];
        if (refFreqHz < 1000000.0) refFreqHz = 1000000.0;
        if (refFreqHz > 60000000.0) refFreqHz = 60000000.0;
        break;

      case MODO_REF_SI5351:
        refFreqSi5351Hz += delta * passosRef[indicePassoRefSi];
        if (refFreqSi5351Hz < 1000000.0) refFreqSi5351Hz = 1000000.0;
        if (refFreqSi5351Hz > 60000000.0) refFreqSi5351Hz = 60000000.0;
        break;

      default:
        break;
    }
    precisaRecalcular = true;
    desenharValores();
  }

  tratarBotao();

  if (precisaRecalcular) {
    ChipAtivo chipAntes = chipAtivo;
    bool ok = aplicarParametros();
    if (!ok) {
      tft.fillRect(0, 104, 160, 12, COR_FUNDO);
      tft.setCursor(4, 104);
      tft.setTextColor(COR_LOCK_NOK);
      tft.print("FREQ INVALIDA");
    } else if (chipAtivo != chipAntes) {
      desenharValores(); // atualiza indicador "Chip ativo" e unidade da potência
    }
    precisaRecalcular = false;
  }

  // ---- status de lock (lido periodicamente) ----
  static unsigned long ultimaLeituraLock = 0;
  if (millis() - ultimaLeituraLock > 200) {
    bool travado = digitalRead(ADF_MUXOUT) == HIGH;
    if (travado != ultimoLock) {
      desenharLock(travado);
      ultimoLock = travado;
    }
    ultimaLeituraLock = millis();
  }
}
