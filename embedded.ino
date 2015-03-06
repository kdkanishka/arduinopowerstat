//required data for Rpi
float supplyVoltage = 0.0;
float supplyAmp = 0.0;
float supplyFrequency = 0.0;
float totalWattSeconds = 0.0;

//data for internal calculations
float _vcc = 0;

//functions to calculate power info
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

void setup()
{
	Serial.begin(9600);
	_vcc = _readVcc()/1000.0;	
	Serial.print("Internal VCC :");Serial.println(_vcc);
}

void loop()
{
	
}
