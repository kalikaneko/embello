// Native mode RF69 driver.

template< typename SPI >
class RF69 {
public:
    void init (uint8_t id, uint8_t group, int freq, uint8_t sphere=0);
    void encrypt (const char* key);
    void txPower (uint8_t level);
    
    int receive (void* ptr, int len);
    void send (uint8_t address, const void* ptr, int len);
    void sleep ();
    
    uint32_t  frf;
    int16_t afc;
    uint8_t rssi;
    uint8_t lna;
    int16_t fei;
    uint8_t currentThreshold;
    uint8_t flags;
    uint8_t goodStep;
    uint8_t badStep;
    uint8_t lowThreshold;
    uint8_t highThreshold;
    uint8_t myId;
    uint8_t myGroup;
    uint8_t mySphere;
    uint8_t powerValuesTX;
    uint8_t powerValuesRX;
    int16_t appAverage;
    int16_t lowestAfc;
    int16_t highestAfc;
    uint8_t beforeTX;

protected:
    enum {
        REG_FIFO          = 0x00,
        REG_OPMODE        = 0x01,
        REG_FRFMSB        = 0x07,
        REG_OSC1          = 0x0A,
        REG_PALEVEL       = 0x11,
        REG_LNAVALUE      = 0x18,
        REG_AFCMSB        = 0x1F,
        REG_AFCLSB        = 0x20,
        REG_FEIMSB        = 0x21,
        REG_FEILSB        = 0x22,
        REG_RSSIVALUE     = 0x24,
        REG_IRQFLAGS1     = 0x27,
        REG_IRQFLAGS2     = 0x28,
        REG_RSSITHRESHOLD = 0x29,
        REG_SYNCVALUE1    = 0x2F,
        REG_SYNCVALUE2    = 0x30,
        REG_PACKETCONFIG1 = 0x37,
        REG_PAYLOADLENGTH = 0x38,
        REG_NODEADDR      = 0x39,
        REG_BCASTADDR     = 0x3A,
        REG_FIFOTHRESH    = 0x3C,
        REG_PKTCONFIG2    = 0x3D,
        REG_AESKEYMSB     = 0x3E,

        MODE_SLEEP        = 0<<2,
        MODE_STDBY        = 1<<2,
        MODE_TRANSMIT     = 3<<2,
        MODE_RECEIVE      = 4<<2,

        START_TX          = 0xC2,
        STOP_TX           = 0x42,

        RCCALSTART        = 0x80,
        RCCALDONE         = 0x40,
        
        MASTER            = 0x80,
        NOISEY            = 0x40,
        WAITPAYLOAD       = 0x20,

        IRQ1_MODEREADY    = 1<<7,
        IRQ1_RXREADY      = 1<<6,
        IRQ1_PLLLOCK      = 1<<4,
        IRQ1_RSSI         = 1<<3,
        IRQ1_SYNADDRMATCH = 1<<0,

        IRQ2_FIFONOTEMPTY = 1<<6,
        IRQ2_PACKETSENT   = 1<<3,
        IRQ2_PAYLOADREADY = 1<<2,
    };

    uint8_t readReg (uint8_t addr) {
        return spi.rwReg(addr, 0);
    }
    void writeReg (uint8_t addr, uint8_t val) {
        spi.rwReg(addr | 0x80, val);
    }
    void setMode (uint8_t newMode);
    void configure (const uint8_t* p);
    void setFrequency (uint32_t freq);

    SPI spi;
    volatile uint8_t mode;
};

// driver implementation

template< typename SPI >
void RF69<SPI>::setMode (uint8_t newMode) {
    mode = newMode;
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | newMode);
    while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0)
        ;
}

template< typename SPI >
void RF69<SPI>::setFrequency (uint32_t hz) {
    // accept any frequency scale as input, including KHz and MHz
    // multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
    while (hz < 100000000)
        hz *= 10;

    // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
    // use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
    // due to this, the lower 6 bits of the calculated factor will always be 0
    // this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
    // 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000  
    frf = (hz << 2) / (32000000L >> 11);
    writeReg(REG_FRFMSB, frf >> 10);
    writeReg(REG_FRFMSB+1, frf >> 2);
    writeReg(REG_FRFMSB+2, frf << 6);
}

template< typename SPI >
void RF69<SPI>::configure (const uint8_t* p) {
    while (true) {
        uint8_t cmd = p[0];
        if (cmd == 0)
            break;
        writeReg(cmd, p[1]);
        p += 2;
    }
}

static const uint8_t configRegs [] = {
    0x01, 0x00, // OpMode = sleep
    0x02, 0x00, // DataModul = packet mode, fsk
    0x03, 0x02, // BitRateMsb, data rate = 49,261 khz
    0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650
    0x05, 0x02, // FdevMsb = 45 KHz
    0x06, 0xE1, // FdevLsb = 45 KHz
    0x0B, 0x20, // AfcLowBetaOn
    0x19, 0x4A, // RxBw 100 KHz
    0x1A, 0x42, // AfcBw 125 KHz
    0x1E, 0x0C, // AfcAutoclearOn, AfcAutoOn
    //0x25, 0x40, //0x80, // DioMapping1 = SyncAddress (Rx)
    0x29, 0xC2, //0xFF, // RssiThresh - 127.5dB
    0x2D, 0x05, // PreambleSize = 5
    0x2E, 0x88, // SyncConfig = sync on, sync size = 2
    0x2F, 0x2D, // SyncValue1 = 0x2D
    0x37, 0xD4, // 0b1101 010X PacketConfig1 = varable, white, crc, filt node or bcast
//    0x37, 0xD0, // 0b1101 010X PacketConfig1 = varable, white, crc, NOfilt node or bcast
    0x38, 0x42, // PayloadLength on RX is 66 bytes
//    0x39, 0x00, // Node address
    0x3A, 0xFF, // Broadcast Address
    0x3C, 0xC2, // 0b1 1000010 TX FifoTresh
    0x3D, 0x12, // 0b0001 0010 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
    0x6F, 0x20, // TestDagc ...
    0x71, 0x02, // RegTecstAfc
    0
};

// My approach to the address byte (FIFO + 1):
//  Same approach to group as RFM12B - group number is used as hardware 
//  sync character if this becomes corrupted in flight then packet will not
//  be seen. The option of group = 0 could be considered if sync pattern
//  was revised.
//  The address byte supported by the RFM69 hardware has I consider a number of
//  uses other than duplicating the group sync pattern character. Since it is
//  inside the CRC its integrity is assured.
//  Devices having a community of interest could share a sphere of influence 
//  address always addressing packets to a designated broadcast address which 
//  they would all receive. Replies could be directed to specific nodes within 
//  the spere. Nodes outside the sphere would not receive these packets due to
//  hardware filtering, provided the node numbers were not duplicated.
//  Central nodes could receive all packets by switching off packet 
//  filtering. The number of spheres's is limited by the need to address
//  individual nodes directly, and by only 256 unique addresses being supported
//  by the hardware.
template< typename SPI >
void RF69<SPI>::init (uint8_t id, uint8_t group, int freq, uint8_t sphere) {
    myId = id;
    myGroup = group;
    mySphere = sphere;
    // 10 MHz, i.e. 30 MHz / 3 (or 4 MHz if clock is still at 12 MHz)
    spi.master(3);

    do
        writeReg(REG_SYNCVALUE1, 0xAA);
    while (readReg(REG_SYNCVALUE1) != 0xAA);
//    do
//        writeReg(REG_SYNCVALUE1, 0x55);
//    while (readReg(REG_SYNCVALUE1) != 0x55);

    currentThreshold = readReg(REG_RSSITHRESHOLD);
    flags &= ~WAITPAYLOAD;
    goodStep = 8;
    badStep = 3;// Initial value to reduce sensitivty (lift the RSSI Threshold)
    highThreshold = 0;
    lowThreshold = ~0;
    
    lowestAfc = 32767;
    highestAfc = -32768;
        
    configure(configRegs);
    setFrequency(freq);

    writeReg(REG_SYNCVALUE2, group);
    writeReg(REG_NODEADDR, myId);
    writeReg(REG_BCASTADDR, mySphere);
    
    setMode(MODE_STDBY);
    writeReg(REG_OSC1, RCCALSTART);
    while (!(readReg(REG_OSC1) & RCCALDONE))
        ;
    setMode(MODE_SLEEP); 
    powerValuesTX = readReg(REG_PALEVEL);
    flags |= NOISEY;
// Debug code
    if (myId == 12) {
        flags |= MASTER;
    }
// End Debug   
}

template< typename SPI >
void RF69<SPI>::encrypt (const char* key) {
    uint8_t cfg = readReg(REG_PKTCONFIG2) & ~0x01;
    if (key) {
        for (int i = 0; i < 16; ++i) {
            writeReg(REG_AESKEYMSB + i, *key);
            if (*key != 0)
                ++key;
        }
        cfg |= 0x01;
    }
    writeReg(REG_PKTCONFIG2, cfg);
}

template< typename SPI >
void RF69<SPI>::txPower (uint8_t level) {
    powerValuesTX = readReg(REG_PALEVEL);
    writeReg(REG_PALEVEL, (powerValuesTX & ~0x1F) | level);// Don't change PA0-2
}

template< typename SPI >
void RF69<SPI>::sleep () {
    setMode(MODE_SLEEP);
}

template< typename SPI >
int RF69<SPI>::receive (void* ptr, int len) {

    currentThreshold = readReg(REG_RSSITHRESHOLD);  // in case it is has changed

    switch (mode) {
    case MODE_RECEIVE: {
        static uint8_t lastFlag;
        if ((readReg(REG_IRQFLAGS1) & IRQ1_RXREADY) != lastFlag) {
            lastFlag ^= IRQ1_RXREADY;
            if (lastFlag) { // flag just went from 0 to 1
                rssi = readReg(REG_RSSIVALUE);  // Timing critical

#if RF69_SPI_BULK
                spi.enable();
                spi.transfer(REG_FEIMSB);
                fei = spi.transfer(0) << 8;     // Timing critical
                fei |= spi.transfer(0);
                spi.transfer(REG_AFCMSB);
                afc = spi.transfer(0) << 8;
                afc |= spi.transfer(0);
                spi.disable();
#else
                fei = readReg(REG_FEIMSB) << 8; // Timing critical
                fei |= readReg(REG_FEILSB);     // Timing critical
                afc = readReg(REG_AFCMSB) << 8;
                afc |= readReg(REG_AFCLSB);

#endif
                lna = (readReg(REG_LNAVALUE) >> 3) & 0x7;

// TODO A scallywag could cause our sensitivity to be backed right off by
//      just broadcasting carrier. Perhaps this code wait until some packet
//      source verification.
                // This section reduces the radio sensitivity
                // if afc is way off and payload didn't happen
                // after the previous IRQ1_RXREADY 
                if ((afc & 0x7F80) && (flags & WAITPAYLOAD)) {
                    currentThreshold-= badStep;
                    if (currentThreshold < 80) currentThreshold = 80;
                    writeReg(REG_RSSITHRESHOLD, currentThreshold);
                    goodStep = 4; //8;   // Count for next uplift
                    flags |= NOISEY;     // in sensitivity
                }
                flags |= WAITPAYLOAD;  // IRQ2_PAYLOADREADY expected next


            }
        }
// The criteria for an incoming packet have been met but it may just be noise.
//        int count;
//        while ((readReg(REG_IRQFLAGS2) & IRQ2_FIFONOTEMPTY) != IRQ2_FIFONOTEMPTY)
//            ;
//        count = readReg(REG_FIFO);






        if (readReg(REG_IRQFLAGS2) & IRQ2_PAYLOADREADY) {
        //  aes can't verify the packet isn't from a scallywag, our node could
        //  be lured away by gradually changing the frequency which we would
        //  track.
            goodStep--;          // Countdown to an increase in sensitivity
            if (!goodStep) {
                badStep = 1;       // Coarse tuning completed
                flags &= ~NOISEY;

                if (currentThreshold > highThreshold) 
                  highThreshold = currentThreshold; // Store bounds of
                if (currentThreshold < lowThreshold)
                  lowThreshold = currentThreshold;  // relative stability 
                if (!currentThreshold++) currentThreshold = 255;
                goodStep = 4; //255;  // Wait a long time before another increase 
                writeReg(REG_RSSITHRESHOLD, currentThreshold);
            }
            flags  &= ~WAITPAYLOAD;

            
#if RF69_SPI_BULK
            spi.enable();
            spi.transfer(REG_FIFO);
            int count = spi.transfer(0);
            for (int i = 0; i < count; ++i) {
                uint8_t v = spi.transfer(0);
                if (i < len)
                    ((uint8_t*) ptr)[i] = v;
            }
            spi.disable();
#else
            int count = readReg(REG_FIFO);
            for (int i = 0; i < count; ++i) {
                uint8_t v = readReg(REG_FIFO);
                if (i < len)
                    ((uint8_t*) ptr)[i] = v;
            }
#endif

// Development Debug of frequency tracking
            
            if (!(flags & NOISEY) && !(flags & MASTER) && ((afc & 0xFFE0))){
            
                if  (afc < 0) {  // Test sign of AFC error
                    frf--;
                }    
                else {
                    frf++;
                }
                
                writeReg(REG_FRFMSB, frf >> 10);
                writeReg(REG_FRFMSB+1, frf >> 2);
                writeReg(REG_FRFMSB+2, frf << 6);

                setMode(MODE_STDBY);
                writeReg(REG_OSC1, RCCALSTART);
                while (!(readReg(REG_OSC1) & RCCALDONE))
                    ;
                setMode(MODE_RECEIVE); 
 
            }
// End Debug
                       
            appAverage -= appAverage >> 3;  // Eight samples
            appAverage += afc >> 3;         //
            
            if(afc < lowestAfc) lowestAfc = afc;
            if(afc > highestAfc) highestAfc = afc;    


            return count;
        }
        break;
    }
    case MODE_TRANSMIT:
        break;
    default:
        setMode(MODE_RECEIVE);
    }
    return -1;
}

template< typename SPI >
void RF69<SPI>::send (uint8_t address, const void* ptr, int len) {

    if ((beforeTX = readReg(REG_IRQFLAGS1)) & (IRQ1_RXREADY + IRQ1_RSSI))
        return;   // Lost TX packet!
// Can't delay loop because RX isn't interrupt driven.

    // while the mode is MODE_TRANSMIT, receive polling will not interfere
    setMode(MODE_TRANSMIT);

#if RF69_SPI_BULK
    spi.enable();
    spi.transfer(REG_FIFO | 0x80);
    spi.transfer(len + 4);
    spi.transfer(address);
    spi.transfer(myId);
    spi.transfer(powerValuesTX);
    spi.transfer(currentThreshold);
    spi.transfer(rssi);
    powerValuesRX = ((powerValuesRX & 0xF8) | lna);   
    spi.transfer(powerValuesRX);
    
    for (int i = 0; i < len; ++i)
        spi.transfer(((const uint8_t*) ptr)[i]);
    spi.disable();
#else
    writeReg(REG_FIFOTHRESH, STOP_TX);  // Wait for FIFO to be filled
    writeReg(REG_FIFO, len + 6);
    writeReg(REG_FIFO, address);
    writeReg(REG_FIFO, myId);
    writeReg(REG_FIFO, powerValuesTX);
    writeReg(REG_FIFO, currentThreshold);
    writeReg(REG_FIFO, rssi);
    powerValuesRX = ((powerValuesRX & 0xF8) | lna);   
    writeReg(REG_FIFO, powerValuesRX);
    for (int i = 0; i < len; ++i)
        writeReg(REG_FIFO, ((const uint8_t*) ptr)[i]);
    writeReg(REG_FIFOTHRESH, START_TX); // Release FIFO for transmission
#endif

    while ((readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) == 0)
        chThdYield();

    setMode(MODE_RECEIVE);
}
