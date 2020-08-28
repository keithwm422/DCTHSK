
#ifndef TwoWire_1_h
#define TwoWire_1_h

#include <inttypes.h>
#include "Stream.h"

#define BUFFER_LENGTH     64

#define IDLE 0
#define MASTER_TX 1
#define MASTER_RX 2
#define SLAVE_RX 3

#define BOOST_PACK_WIRE 1

class TwoWire_1 : public Stream
{

	private:
                static uint8_t rxBuffer[];
		static uint8_t rxReadIndex;
		static uint8_t rxWriteIndex;

		static uint8_t txAddress;
		static uint8_t txBuffer[];
		static uint8_t txReadIndex;
		static uint8_t txWriteIndex;

		static uint8_t i2cModule;
		static uint8_t slaveAddress;

		static uint8_t transmitting;
		static uint8_t currentState;
		static void (*user_onRequest)(void);
		static void (*user_onReceive)(int);
		static void onRequestService(void);
		static void onReceiveService(uint8_t*, int);
		uint8_t getError(uint8_t);
		uint8_t getRxData(unsigned long cmd);
		uint8_t sendTxData(unsigned long cmd, uint8_t data);
		void forceStop(void);

    public:
		TwoWire_1(void);
		TwoWire_1(unsigned long);
		void begin();
		void begin(uint8_t);
		void begin(int);
		void beginTransmission(uint8_t);
		void beginTransmission(int);
		uint8_t endTransmission(void);
		uint8_t endTransmission(uint8_t);
		uint8_t requestFrom(uint8_t, uint8_t);
		uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
		uint8_t requestFrom(int, int);
		uint8_t requestFrom(int, int, int);
		virtual size_t write(uint8_t);
		virtual size_t write(const uint8_t *, size_t);
		virtual int available(void);
		virtual int read(void);
		virtual int peek(void);
		virtual void flush(void);
		void onReceive( void (*)(int) );
		void onRequest( void (*)(void) );


		inline size_t write(unsigned long n) { return write((uint8_t)n); }
		inline size_t write(long n) { return write((uint8_t)n); }
		inline size_t write(unsigned int n) { return write((uint8_t)n); }
		inline size_t write(int n) { return write((uint8_t)n); }
		using Print::write;

		//Stellarpad-specific functions
		void I2CIntHandler(void);
		void setModule(unsigned long);

};

extern TwoWire_1 Wire_1;
extern "C" void I2CIntHandler_1(void);
#endif
