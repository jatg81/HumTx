#include <RF69Mod.h>
#include <Arduino.h>

volatile uint16_t rf69_crc;
volatile uint8_t rf69_buf[72];
static byte nodeid;                      // only used in the easyPoll code
static uint8_t ezInterval;               // number of seconds between transmits
static uint8_t ezSendBuf[RF69_MAXDATA];  // data to send
static char ezSendLen;                   // number of bytes to send
static uint8_t ezPending;                // remaining number of retries
static long ezNextSend[2];               // when was last retry [0] or data [1] sent
static volatile uint8_t rxfill;          // number of data bytes in rf69_buf
static volatile int8_t rxstate;          // current transceiver state
uint8_t* recvBuf;
uint32_t frf;  
uint8_t  group;
uint8_t  node;
uint8_t  rssi;
uint16_t crc;


enum { TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE, TXRECV };    // transceiver states, these determine what to do with each interrupt

static ROM_UINT8 configRegs [] ROM_DATA = {
  0x01, 0x04, // OpMode = standby
  0x02, 0x00, // DataModul = packet mode, fsk
  0x03, 0x02, // BitRateMsb, data rate = 49,261 khz
  0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650
  0x05, 0x05, // FdevMsb = 90 KHz
  0x06, 0xC3, // FdevLsb = 90 KHz
  0x0B, 0x20, // AfcCtrl, afclowbetaon
  0x19, 0x42, // RxBw ...
  0x1E, 0x2C, // FeiStart, AfcAutoclearOn, AfcAutoOn
  0x25, 0x80, // DioMapping1 = SyncAddress (Rx)
  // 0x29, 0xDC, // RssiThresh ...
  0x2E, 0x90, // SyncConfig = sync on, sync size = 3
  0x2F, 0xAA, // SyncValue1 = 0xAA
  0x30, 0x2D, // SyncValue2 = 0x2D
  0x37, 0x00, // PacketConfig1 = fixed, no crc, filt off
  0x38, 0x00, // PayloadLength = 0, unlimited
  0x3C, 0x8F, // FifoTresh, not empty, level 15
  0x3D, 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
  0x6F, 0x20, // TestDagc ...
  0
};

static void spiConfigPins () {
    SS_PORT |= _BV(SS_BIT); 
    SS_DDR |= _BV(SS_BIT);
    PORTB |= _BV(SPI_SS);
    DDRB |= _BV(SPI_SS) | _BV(SPI_MOSI) | _BV(SPI_SCK);
}

struct PreventInterrupt {
    PreventInterrupt () { EIMSK &= ~ _BV(INT0); }
    ~PreventInterrupt () { EIMSK |= _BV(INT0); }
};

static void spiInit (void) {
    spiConfigPins();
    SPCR = _BV(SPE) | _BV(MSTR);
    SPSR |= _BV(SPI2X);
}

static uint8_t spiTransferByte (uint8_t out) {
    SPDR = out;
    while (!(SPSR & _BV(SPIF)))
        ;
    return SPDR;
}

static uint8_t spiTransfer (uint8_t cmd, uint8_t val) {
    SS_PORT &= ~ _BV(SS_BIT);
    spiTransferByte(cmd);
    uint8_t in = spiTransferByte(val);
    SS_PORT |= _BV(SS_BIT);
    return in;
}


uint8_t control(uint8_t cmd, uint8_t val) {
    PreventInterrupt irq0;
    return spiTransfer(cmd, val);
}

static void writeReg (uint8_t addr, uint8_t value) {
    control(addr | 0x80, value);
}

static uint8_t readReg (uint8_t addr) {
    return control(addr, 0);
}

void setFrequency (uint32_t freq) {
        frf = ((freq << 2) / (32000000L >> 11)) << 6;
}

static void initRadio (ROM_UINT8* init) {
    spiInit();
    do
        writeReg(REG_SYNCVALUE1, 0xAA);
    while (readReg(REG_SYNCVALUE1) != 0xAA);
    do
        writeReg(REG_SYNCVALUE1, 0x55);
    while (readReg(REG_SYNCVALUE1) != 0x55);
    for (;;) {
        uint8_t cmd = ROM_READ_UINT8(init);
        if (cmd == 0) break;
        writeReg(cmd, ROM_READ_UINT8(init+1));
        init += 2;
    }
}


void configure () {
    initRadio(configRegs);
    writeReg(REG_SYNCVALUE3, group);
    writeReg(REG_FRFMSB, frf >> 16);
    writeReg(REG_FRFMSB+1, frf >> 8);
    writeReg(REG_FRFMSB+2, frf);
    rxstate = TXIDLE;
}

static void setMode (uint8_t mode) {
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | mode);
}

void interrupt () {
    if (rxstate == TXRECV) {
        // The following line attempts to stop further interrupts
        writeReg(REG_DIOMAPPING1, 0x40);  // Interrupt on PayloadReady
        IRQ_ENABLE; // allow nested interrupts from here on
        for (;;) { // busy loop, to get each data byte as soon as it comes in
            if (readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY|IRQ2_FIFOOVERRUN)) {
                if (rxfill == 0)
                    recvBuf[rxfill++] = group;
                uint8_t in = readReg(REG_FIFO);
                recvBuf[rxfill++] = in;
                crc = _crc16_update(crc, in);
                if (rf69_len > RF69_MAXDATA)
                    rxfill = RF_MAX; // bail out now, the length is invalid
                if (rxfill >= rf69_len + 5 || rxfill >= RF_MAX)
                    break;
            }
        }
    } else if (readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT) {
        // rxstate will be TXDONE at this point
        rxstate = TXIDLE;
        setMode(MODE_STANDBY);
        writeReg(REG_DIOMAPPING1, 0x80); // SyncAddress
    }
}

static void flushFifo () {
    while (readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN))
        readReg(REG_FIFO);
}

void sleep (bool off) {
    setMode(off ? MODE_SLEEP : MODE_STANDBY);
    rxstate = TXIDLE;
}

int16_t readRSSI() {
    int16_t rssi = 0;
    rssi = -readReg(REG_RSSIVALUE);
    rssi >>= 1;
    return rssi;
}

uint8_t rf69_initialize (uint8_t id, uint8_t band, uint8_t grp, uint16_t off) {
    uint8_t freq = 0;
    switch (band) {
        case RF69_433MHZ: freq = 43; break;
        case RF69_868MHZ: freq = 86; break;
        case RF69_915MHZ: freq = 90; break;
    }
    setFrequency(freq * 10000000L + band * 2500L * off);
    group = grp;
    node = id & RF69_HDR_MASK;
    delay(20); // needed to make RFM69 work properly on power-up
    if (node != 0)
        attachInterrupt(0, interrupt, RISING);
    else
        detachInterrupt(0);
    configure();
    return nodeid = id;
}

uint8_t rf69_recvDone () {
    switch (rxstate) {
    case TXIDLE:
        rxfill = rf69_buf[2] = 0;
        crc = _crc16_update(~0, group);
        recvBuf = rf69_buf;
        rxstate = TXRECV;
        flushFifo();
        writeReg(REG_DIOMAPPING1, DMAP1_SYNCADDRESS);    // Interrupt trigger
        setMode(MODE_RECEIVER);
        writeReg(REG_AFCFEI, AFC_CLEAR);
        break;
    case TXRECV:
        if (rxfill >= rf69_len + 5 || rxfill >= RF_MAX) {
            rxstate = TXIDLE;
            setMode(MODE_STANDBY);
            if (crc != 0 | rf69_len > RF69_MAXDATA)
                rf69_crc = 1; // force bad  for invalid packet
            if (!(rf69_hdr & RF69_HDR_DST) || node == 31 ||
                    (rf69_hdr & RF69_HDR_MASK) == node)
                rf69_crc = 0; // it's for us, good packet received
        return 1;
        }
        break;
    }
    return 0; // keep going, not done yet
}

uint8_t rf69_canSend () {
    if (rxstate == TXRECV && rxfill == 0 && readRSSI() < CSMA_LIMIT) {  //if signal stronger than -90dBm is detected assume channel activity
        rxstate = TXIDLE;
        setMode(MODE_STANDBY);
        return true;
    }
    return false;
}

void rf69_sendStart (uint8_t hdr, const void* ptr, uint8_t len) {
    rf69_buf[2] = len;
    for (int i = 0; i < len; ++i)
        rf69_data[i] = ((const uint8_t*) ptr)[i];
    rf69_hdr = hdr & RF69_HDR_DST ? hdr : (hdr & ~RF69_HDR_MASK) + node;
    crc = _crc16_update(~0, group);
    rxstate = - (2 + rf69_len); // preamble and SYN1/SYN2 are sent by hardware
    flushFifo();
    setMode(MODE_TRANSMITTER);
    writeReg(REG_DIOMAPPING1, 0x00); // PacketSent

    while (rxstate < TXDONE)
        if ((readReg(REG_IRQFLAGS2) & IRQ2_FIFOFULL) == 0) {
            uint8_t out = 0xAA;
            if (rxstate < 0) {
                out = recvBuf[3 + rf69_len + rxstate];
                crc = _crc16_update(crc, out);
            } else {
                switch (rxstate) {
                    case TXCRC1: out = crc; break;
                    case TXCRC2: out = crc >> 8; break;
                }
            }
            writeReg(REG_FIFO, out);
            ++rxstate;
        }
}

void rf69_sendNow (uint8_t hdr, const void* ptr, uint8_t len) {
    uint32_t now = millis();
    while (!rf69_canSend()&& millis() - now < RF69_CSMA_LIMIT_MS) rf69_recvDone();
    rf69_sendStart(hdr, ptr, len);
}


bool rf69_sendWithRetry (uint8_t nodeId,const void* ptr, uint8_t len,uint8_t retries, uint8_t retryWaitTime) {
    uint32_t sentTime;
    for (uint8_t i = 0; i <= retries; i++)
    {
      rf69_sendNow (RF69_HDR_ACK|nodeId, ptr,len);
      sentTime = millis();
      while (millis() - sentTime < retryWaitTime)
      {
        if (rf69_recvDone()&& rf69_hdr==(RF69_HDR_CTL|RF69_HDR_DST|nodeId)) return true;
      }
    }
    return false;
}
    

uint8_t rf69_receiveStart (uint8_t nodeIdSrc, const void* ptr, uint8_t len){
  if(rf69_crc ==0 && (rf69_hdr&RF69_HDR_MASK)==nodeIdSrc && rf69_buf[3]==nodeid && len==rf69_len-1){
    for (int i = 0; i < len; ++i)
    ((uint8_t*) ptr)[i]=rf69_buf[i+4];
    return 1;
  }
  return 0;
}


void rf69_sleep (char n) {
    sleep(n == RF69_SLEEP);
}
