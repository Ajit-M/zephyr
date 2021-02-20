// Helper function to read and write uart data from Frsky 

#include <device.h>

void rc_input(deveice *uartrc_dev);

/*
* Hardware Serial Supported:
*  STM32F4 
*/


class SBUS{
	public:
		SBUS(HardwareSerial& bus);
		void begin();
		bool read(uint16_t* channels, bool* failsafe, bool* lostFrame);
		bool readCal(float* calChannels, bool* failsafe, bool* lostFrame);
		// void write(uint16_t* channels);
		// void writeCal(float *channels);
		// void setEndPoints(uint8_t channel,uint16_t min,uint16_t max);
		// void getEndPoints(uint8_t channel,uint16_t *min,uint16_t *max);
		// void setReadCal(uint8_t channel,float *coeff,uint8_t len);
		// void getReadCal(uint8_t channel,float *coeff,uint8_t len);
		// void setWriteCal(uint8_t channel,float *coeff,uint8_t len);
		// void getWriteCal(uint8_t channel,float *coeff,uint8_t len);
		~SBUS();
  private:
		const uint32_t _sbusBaud = 100000;
		static const uint8_t _numChannels = 16;
		const uint8_t _sbusHeader = 0x0F;
		const uint8_t _sbusFooter = 0x00;
		const uint8_t _sbus2Footer = 0x04;
		const uint8_t _sbus2Mask = 0x0F;
		const uint32_t SBUS_TIMEOUT_US = 7000;
		uint8_t _parserState, _prevByte = _sbusFooter, _curByte;
		static const uint8_t _payloadSize = 24;
		uint8_t _payload[_payloadSize];
		const uint8_t _sbusLostFrame = 0x04;
		const uint8_t _sbusFailSafe = 0x08;
		const uint16_t _defaultMin = 172;
		const uint16_t _defaultMax = 1811;
		uint16_t _sbusMin[_numChannels];
		uint16_t _sbusMax[_numChannels];
		float _sbusScale[_numChannels];
		float _sbusBias[_numChannels];
		float **_readCoeff, **_writeCoeff;
		uint8_t _readLen[_numChannels],_writeLen[_numChannels];
		bool _useReadCoeff[_numChannels], _useWriteCoeff[_numChannels];
		HardwareSerial* _bus;
		bool parse();
		void scaleBias(uint8_t channel);
		float PolyVal(size_t PolySize, float *Coefficients, float X);
};

#endif