// Arduino PIN definitions
const int _VIN_PIN = A0;
const int _CT_PIN = A1;
//required data for Rpi
float supplyVoltage = 0.0;
float supplyAmp = 0.0;
float supplyFrequency = 0.0;
float totalWattSeconds = 0.0;

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
	Serial.print(inputVoltage);
	Serial.println(" V");
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
	Serial.print(amps);Serial.println("A");
}


void setup()
{
	Serial.begin(9600);
	_vcc = _readVcc()/1000.0;	
	Serial.print("Internal VCC :");Serial.println(_vcc);
}

void loop()
{
	unsigned long calculateBegins = millis();
	measureSupplyVoltage();
	measureSupplyAmperage();
	unsigned long calculationsEnds = millis();
}
