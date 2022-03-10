#include <arduinoFFT.h> //Librería para calcular la transformada de Fourier
#include <FastLED.h>

//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

//SEÑAL DE AUDIO
#define SAMPLES 512         //Tiene que ser potencia de 2. Arduino Nano solo tiene memoria para guardar 128 muestras
#define SAMPLING_FREQ 40000 //Determina la frecuencia máxima que se puede analizar con FFT. Freq_max = sampling_freq/2 
#define AMPLITUDE 2500      //Can be used as a "sensitivity" control (1000)
#define AUDIO_IN_PIN A0     //Entrada analógica a la que se ha conectado el microfono
#define NOISE 500           //Los valores que estén por debajo de NOISE se ignoran
//******************************************************
#define NUM_BANDS 16      
#define MAX_BAR_HEIGHT 9   //Número de leds por columna

//LEDS
#define NUM_LEDS NUM_BANDS*MAX_BAR_HEIGHT
#define LEDS_PIN 14   //D5 en la NodeMCU ESP8266
#define COLOR_ORDER GRB
#define MAX_BRIGHTNESS 150  //Brillo máximo
#define LED_TYPE WS2812B
CRGB leds[NUM_LEDS];


unsigned int sampling_period_us;
unsigned long newTime;
double vReal[SAMPLES];
double vImag[SAMPLES];
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);
//***********************************************************
int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //Tenemos 16 bandas
//***********************************************************
int oldBarHeights[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//***********************************************************
int peak[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//HSV
byte base_hue = 0;
//Palettes
DEFINE_GRADIENT_PALETTE (ocean_palette){
  0, 13, 4, 156,
  75, 39, 142, 228,
  145, 40, 160, 255,
  255, 0, 255, 255
};

DEFINE_GRADIENT_PALETTE (fire_palette){
  0, 255, 0, 0,
  75, 255, 149, 0,
  150, 255, 0, 100,
  255, 255, 255, 255
};

CRGBPalette16 myPal;


void setup() {
  // put your setup code here, to run once:
  //Configuramos el ADC para obtener 58.6 KHz de velocidad de muestreo
  Serial.begin(115200);

  /*
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  */
  
  
  sampling_period_us = round(1000000 * (1.0/SAMPLING_FREQ));
  FastLED.addLeds<LED_TYPE, LEDS_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);

  //Elegimos Palette
  //myPal = ocean_palette;
  myPal = fire_palette;
}

void loop() {
  FastLED.clear();
  Serial.println("0");

  // Reset bandValues[]
  for (int i = 0; i<NUM_BANDS; i++){
    bandValues[i] = 0;
  }
  
  //Muestreamos la señal eléctrica recibida del microfono
  for(int i=0; i<SAMPLES; i++){
    newTime = micros();   //Se desborda a los 70 minutos aprox.                
    vReal[i] = analogRead(AUDIO_IN_PIN);
    vImag[i] = 0;
    while((micros()-newTime) < sampling_period_us) {  }
  }
  
  FFT.DCRemoval();  //Elimina el offset de la señal eléctrica
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  //Analizamos los resultados del FFT
  for(int i=2; i<(SAMPLES/2); i++){
    if(vReal[i] > NOISE) {    //Filtro de ruido
  
    //16 bands, 300Hz a 18000KHz, 256 muestras
      if (i<=2 )           bandValues[0]  += (int)vReal[i];
      if (i>1   && i<=2  ) bandValues[1]  += (int)vReal[i];
      if (i>2   && i<=3  ) bandValues[2]  += (int)vReal[i];
      if (i>3   && i<=4  ) bandValues[3]  += (int)vReal[i];
      if (i>4   && i<=6  ) bandValues[4]  += (int)vReal[i];
      if (i>6   && i<=9 ) bandValues[5]  += (int)vReal[i];
      if (i>9  && i<=13 ) bandValues[6]  += (int)vReal[i];
      if (i>13  && i<=18 ) bandValues[7]  += (int)vReal[i];
      if (i>18  && i<=26 ) bandValues[8]  += (int)vReal[i];
      if (i>26  && i<=37 ) bandValues[9]  += (int)vReal[i];
      if (i>37  && i<=53 ) bandValues[10] += (int)vReal[i];
      if (i>53  && i<=76 ) bandValues[11] += (int)vReal[i];
      if (i>76  && i<=108) bandValues[12] += (int)vReal[i];
      if (i>108 && i<=153) bandValues[13] += (int)vReal[i];
      if (i>153 && i<=218) bandValues[14] += (int)vReal[i];
      if (i>218          ) bandValues[15] += (int)vReal[i];  
    
    }
  }

  for(int band=0; band<NUM_BANDS; band++){
      //Escalamos las barras
      int barHeight = bandValues[band] / AMPLITUDE;
      if(barHeight > MAX_BAR_HEIGHT)  barHeight = MAX_BAR_HEIGHT;

      //Hacemos promedio con el valor anterior
      barHeight = round(((oldBarHeights[band] * 0.5) + 1.2* barHeight) / 2);

      //Movemos la "cima"
      if(barHeight > peak[band]){
        peak[band] = min(MAX_BAR_HEIGHT, barHeight);
      }

      drawWithCHSV(band, barHeight, base_hue);
      drawPeak(band, 255, 255, 255);

      oldBarHeights[band] = barHeight; 
     
  }

  //Bajamos la cima
  EVERY_N_MILLISECONDS(110){
    for (byte band = 0; band < NUM_BANDS; band++)
      if (peak[band] > 0) peak[band] -= 1;
  }

  if(base_hue>245)base_hue = 0;
  else base_hue +=10;

  FastLED.show();
}


void drawWithCHSV(int num_band, int barHeight, byte hue){
  int pos = num_band;
  for(int i=0; i<barHeight; i++){
    if((i%2)!= 0 && i!=0){
      pos += NUM_BANDS*2-(num_band*2)-1;
    }
    else if(i!=0){
      pos += num_band*2+1;
    }
    leds[pos] = CHSV(hue, 255, 255);
    hue+=5;
  }
}

void drawPeak(int num_band, byte R, byte G, byte B){
  int pos = num_band;
  for(int i=0; i<=peak[num_band]; i++){
    if((i%2)!= 0 && i!=0){
      pos += NUM_BANDS*2-(num_band*2)-1;
    }
    else if(i!=0){
      pos += num_band*2+1;
    }
  }
  leds[pos].setRGB(R, G, B);
}
