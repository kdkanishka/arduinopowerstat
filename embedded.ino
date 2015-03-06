// Arduino PIN definitions
const int _VIN_PIN = A0;
const int _CT_PIN = A1;
//required data for Rpi
float supplyVoltage = 0.0;
float supplyAmp = 0.0;
float supplyFrequency = 0.0;
float totalKWattSeconds = 0.0;

//data for internal calculations
float _vcc = 0;

/**
 *	calculates initial internal VCC and keep it for future reference
 */
long _readVcc() {
	long result;
	// Read 1.1V reference against AVcc
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  	delay(2); // Wait for Vref to settle
  	ADCSRA |= _BV(ADSC); // Convert
  	while (bit_is_set(ADCSRA,ADSC));
  	result = ADCL;
  	result |= ADCH<<8;
  	result = 1126400L / result; // Back-calculate AVcc in mV
  	return result;
}

/**
 * Measure supply voltage
 */
void measureSupplyVoltage(){  
	int samplingCount=512;
	float adcResolution = 1023.0;
	float zSq=0;
	float halfVcc = _vcc / 2.0;
	//ADC sampling
	for(int i=0; i < samplingCount; i++){
		int sensorValue = analogRead(_VIN_PIN);
		float voltage = (sensorValue * _vcc) / adcResolution;
		zSq+=pow(voltage-halfVcc,2.0);
	}

	float R1=100000.0; //100K in voltage divider
	float R2=10000.0; //10K in voltage divider
	float transformerFact = 21.8; //np/ns

	float vRms = sqrt(zSq / samplingCount);
	float vRealSec = vRms/(R2/(R1+R2));
	float inputVoltage = vRealSec * transformerFact;
	if(inputVoltage <= 1){
		inputVoltage = 0;
	}
	supplyVoltage = inputVoltage;
	//Serial.print(inputVoltage);
	//Serial.println(" V");
}

/**
 * Measure Amperage
 */
void measureSupplyAmperage(){
	int samplingCount=512;
	float adcResolution = 1023.0;
	float zSq=0;
	float halfVcc = _vcc / 2.0;
	for(int i=0;i<samplingCount;i++){
		int sensorValue = analogRead(_CT_PIN);
		float ctmonVoltage = (sensorValue * _vcc) / adcResolution; // see text
		zSq+=pow(ctmonVoltage-halfVcc,2.0);
	}
	//this vRms is propotional to the amperage
	float ctMonFact = 30.0;
	float vRms = sqrt(zSq / samplingCount);
	float roundedvRms = round(vRms * 100)/100.0; //round value to two decimal points
	float amps = roundedvRms * ctMonFact;
	supplyAmp = amps;
	//Serial.print(amps);Serial.println("A");
}

/**
 * Measure supply frequency
 */
void measureSupplyFrequency(){
  int t1 = millis();
  int sampleCount = 255;
  int times[sampleCount];  
  int samples[sampleCount];
  
  //sampling
  for(int i=0;i<sampleCount;i++){
     int sensorValue = analogRead(_VIN_PIN);
     samples[i] = sensorValue;
     times[i] = millis()-t1;
     delayMicroseconds(39);
  }
  
  //analyzing sampled data
  //step 1 : find peak 1
  int peakIdx1 = findPeakVal(0,sampleCount,samples,sampleCount);
  if(peakIdx1 > -1){
    //Serial.println(peakIdx1); 
  }
  int T1 = times[peakIdx1];
  //step 2 : find peak 2
  int peakIdx2 = findPeakVal(peakIdx1+1,sampleCount,samples,sampleCount);
  if(peakIdx2 > -1){
    //Serial.println(peakIdx2); 
  }
  int T2 = times[peakIdx2];
  int T  = T2-T1;
  //Serial.println(T);
  float frequency = (1.0/T)*1000;
  if(frequency>5){
    //Serial.print("AC Voltage Frequency : "); Serial.println(frequency);
    supplyFrequency = frequency;
  }
}

//returns time index at the  value
int findPeakVal(int from,int to,int *samples,int samplesSize){
  int peakIdx = -1;
  int r=0; int shift = 10;
  for(r=from; r<samplesSize-shift ; r++){
    //check has peak between the range (r to r+shift)
    bool hasPeak = _hasPeak(r,shift,samples);
    if(hasPeak==true){
      //Serial.print("Peak found ");Serial.print(r);Serial.print(" ");Serial.println(r+shift);
      //if a peak is detected in this range lets find the peak value index
      int peakIndex = _findPeakIndex(r,r+shift,samples);
      peakIdx = peakIndex;
      //Serial.print("Peak index ");Serial.println(peakIndex);
      break;
    } 
  }
  return peakIdx;
}

int _findPeakIndex(int b1,int b2,int *samples){
  int maxIdx = b1;
  int maxVal = samples[b1];
  for(int i=b1;i<=b2;i++){
    if(samples[i]>=maxVal){
      maxIdx = i;
      maxVal = samples[i];
    }
  }
  return maxIdx;
}

bool _hasPeak(int r,int shift,int *samples){
 bool hasPeak = false;
 //calculate average
 int tot = 0;
 for(int i=r; i<r+shift ; i++){
   tot = tot + samples[i];
 }
 float average = tot * 1.0 / shift;
 //if average if higher than or equals  boundary values this can be a 
 if((samples[r] <= average) && (average >= samples[r+shift-1])){
   hasPeak = true; 
 }
 return hasPeak; 
}

void serialPrint(String pref,String sufx){
	Serial.print(pref);Serial.println(sufx);
}


void setup()
{
	Serial.begin(9600);
	_vcc = _readVcc()/1000.0;	
	Serial.print("Internal VCC :");Serial.println(_vcc);
}

void loop()
{
	unsigned long t1 = millis();
	measureSupplyVoltage();
	measureSupplyAmperage();
	unsigned long t2 = millis();
	//calculate ws
	float kWattDelta = (supplyVoltage * supplyAmp)/1000.0; 
	long tDelta = t2 - t1 ; //in millis
	float kwsDelta = (kWattDelta * tDelta)/1000.0;
	totalKWattSeconds += kwsDelta;

	//measure frequency
	if(supplyVoltage > 0){
		measureSupplyFrequency();
	}else{
		supplyFrequency = 0;
	}


	serialPrint(String(supplyVoltage),"V");
	serialPrint(String(supplyAmp),"A");
	serialPrint(String(supplyFrequency),"Hz");
	serialPrint(String(totalKWattSeconds),"KWS");
}


