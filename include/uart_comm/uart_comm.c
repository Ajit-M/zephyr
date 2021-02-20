#include "frsky.h"


/*

Arduino Library - Milliselapsed


*/



int begin(){
    // initialize parsing state
        _parserState = 0;
    // initialize default scale factors and biases
	for (uint8_t i = 0; i < _numChannels; i++) {
		setEndPoints(i,_defaultMin,_defaultMax);
	}
    return 0;
}

bool read(uint16_t* channels, bool* failsafe, bool* lostFrame, int uart_confg_ret, const struct device *uart_dev){
    // parse the SBUS packet
	if (parse(uart_confg_ret, uart_dev)) {
		if (channels) {
			// 16 channels of 11 bit data
			channels[0]  = (uint16_t) ((_payload[0]    |_payload[1] <<8)                     & 0x07FF);
			channels[1]  = (uint16_t) ((_payload[1]>>3 |_payload[2] <<5)                     & 0x07FF);
			channels[2]  = (uint16_t) ((_payload[2]>>6 |_payload[3] <<2 |_payload[4]<<10)  	 & 0x07FF);
			channels[3]  = (uint16_t) ((_payload[4]>>1 |_payload[5] <<7)                     & 0x07FF);
			channels[4]  = (uint16_t) ((_payload[5]>>4 |_payload[6] <<4)                     & 0x07FF);
			channels[5]  = (uint16_t) ((_payload[6]>>7 |_payload[7] <<1 |_payload[8]<<9)   	 & 0x07FF);
			channels[6]  = (uint16_t) ((_payload[8]>>2 |_payload[9] <<6)                     & 0x07FF);
			channels[7]  = (uint16_t) ((_payload[9]>>5 |_payload[10]<<3)                     & 0x07FF);
			channels[8]  = (uint16_t) ((_payload[11]   |_payload[12]<<8)                     & 0x07FF);
			channels[9]  = (uint16_t) ((_payload[12]>>3|_payload[13]<<5)                     & 0x07FF);
			channels[10] = (uint16_t) ((_payload[13]>>6|_payload[14]<<2 |_payload[15]<<10) 	 & 0x07FF);
			channels[11] = (uint16_t) ((_payload[15]>>1|_payload[16]<<7)                     & 0x07FF);
			channels[12] = (uint16_t) ((_payload[16]>>4|_payload[17]<<4)                     & 0x07FF);
			channels[13] = (uint16_t) ((_payload[17]>>7|_payload[18]<<1 |_payload[19]<<9)  	 & 0x07FF);
			channels[14] = (uint16_t) ((_payload[19]>>2|_payload[20]<<6)                     & 0x07FF);
			channels[15] = (uint16_t) ((_payload[20]>>5|_payload[21]<<3)                     & 0x07FF);
		}
		// What should be the default value of lostFrame variable ?
		if (lostFrame) {
			// count lost frames
			if (_payload[22] & _sbusLostFrame) {
				*lostFrame = true;
			} else {
				*lostFrame = false;
				}
		}
		if (failsafe) {
			// failsafe state
			if (_payload[22] & _sbusFailSafe) {
				*failsafe = true;
			}
			else{
				*failsafe = false;
			}
		}
		// return true on receiving a full packet
		return true;
  	} else {
		// return false if a full packet is not received
		return false;
	}

}

bool parse(int uart_confg_ret, const struct device *uart_dev){
    // reset the parser state if too much time has passed
	// static elapsedMicros _sbusTime = 0;
	// if (_sbusTime > SBUS_TIMEOUT_US) {_parserState = 0;}
	// see if serial data is available

    unsigned char data_read;
    int uart_read_ret;

	while (uart_confg_ret == 0) {
		// _sbusTime = 0;
		// _curByte = _bus->read();
        uart_read_ret = uart_poll_in(uart_dev, &data_read);
		// find the header
        if (uart_read_ret == 0){
            if (_parserState == 0) {
                    if (((uint8_t)data_read == _sbusHeader) && ((_prevByte == _sbusFooter) || ((_prevByte & _sbus2Mask) == _sbus2Footer))) {
                        _parserState++;
                    } else {
                        _parserState = 0;
                    }
            } else {
                // strip off the data
                if ((_parserState-1) < _payloadSize) {
                    _payload[_parserState-1] = (uint8_t)data_read;
                    _parserState++;
                }
                // check the end byte
                if ((_parserState-1) == _payloadSize) {
                    if (((uint8_t)data_read == _sbusFooter) || (((uint8_t)data_read & _sbus2Mask) == _sbus2Footer)) {
                        _parserState = 0;
                        return true;
                    } else {
                        _parserState = 0;
                        return false;
                    }
                }
            }
            _prevByte = (uint8_t)data_read;
        }
    }
	// return false if a partial packet
	return false;
}