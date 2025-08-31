#include <xc.h>
#include <pic16f872.h>

//config
#pragma config CP    = OFF
#pragma config WRT   = OFF
#pragma config CPD   = OFF
#pragma config LVP   = OFF
#pragma config BOREN = OFF
#pragma config PWRTE = OFF
#pragma config WDTE  = OFF
#pragma config FOSC  = XT  //0.1 - 4 MHz  (HS = 4-20MHz)
#define   _XTAL_FREQ   2000000 //2 MHz clock input

#define   i2caddress   0x58    //directly above eeprom address space: 0x50 - 0x57, maxium 7 bits

//function declaration
void setPmtVoltage(uint16_t value);
void setHvState(uint8_t onOff);
void setGainVoltage(uint16_t value);
void setfbGainVoltage(uint16_t value);
void setLedVoltage(uint16_t value);
void shiftOutAndLoadDAC(uint16_t value, uint8_t AdcCsPin, uint8_t channel);
uint16_t getPMTlowAmpSignal(void);
uint16_t getPMThighAmpSignal(void);
uint16_t getPMTvoltage(void);

int currentRegister = 0;

//i2c registers all, two bytes
#define regHighVoltageAdjust     0x10 
#define regHighVoltageFeedback   0x11
#define regOffsetVoltage         0x20
#define regFbGainVoltage         0x30
#define regLowAmpSignal          0x31
#define regHighAmpSignal         0x32 
#define regCalibrationLedVoltage 0x40


volatile uint8_t  i2c_register      = 0; //written in interrupt
volatile uint8_t  i2c_messageLength = 0;
volatile uint16_t i2c_data          = 0;

void writeI2Cregister(uint16_t value){
    switch(i2c_register){
        case regHighVoltageAdjust:
            setPmtVoltage(value);
        break;
        case regOffsetVoltage:
            setGainVoltage(value);
        break;
        case regFbGainVoltage:
            setfbGainVoltage(value);
        break;
        case regCalibrationLedVoltage:
            setLedVoltage(value);
        break;
    }
    i2c_register = 0x00;
}

uint16_t readI2Cregister(void){
    switch(i2c_register){
        case regHighVoltageFeedback:
            return getPMTvoltage();
        break;
        case regLowAmpSignal:
            return getPMTlowAmpSignal();
        break;
        case regHighAmpSignal:
            return getPMThighAmpSignal();
        break;
        default:
            return 0xffff;
        break;
    }
    i2c_register = 0x00;
}

void __interrupt() ISR() {
    if(PIR1bits.SSPIF){ //interrupt from i2c for each completed byte
        SSPCONbits.CKP = 0; //start clock stretching
        
        //clear overflow
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
          volatile uint8_t discarded = SSPBUF; // Read the previous value to clear the buffer
          SSPCONbits.SSPOV = 0; // Clear the overflow flag
          SSPCONbits.WCOL = 0;  // Clear the collision bit
          SSPCONbits.CKP = 1;
        }
        
        PORTCbits.RC6 ^= 0x01; //indicate activity with pin RC6 (connected to led in my case)
        if(SSPSTATbits.R_nW){
            //the i2c address is a read address
            if(SSPSTATbits.D_nA){
                //read from master
                switch(i2c_messageLength){
                    case 2:
                        SSPBUF = i2c_data & 0xff; //send second byte (LSB))
                        i2c_messageLength = 0; //completed successfully 
                        i2c_register = 0;
                    break;
                    default:
                        i2c_data = 0;
                        i2c_messageLength = 0;
                        i2c_register = 0;
                    break;
                }
            }else{
                volatile uint8_t discarded = SSPBUF;     //discard address
                i2c_data = readI2Cregister();
                SSPBUF = (i2c_data>>8) & 0xff; //send first byte (MSB)
                i2c_messageLength = 2;
            }
        }else{
            //the i2c address is a write address
            if(SSPSTATbits.D_nA){
                //write from master
                switch(i2c_messageLength){
                    case 1: //set register
                        i2c_register = SSPBUF;
                        i2c_messageLength = 2;
                    break;
                    case 2:
                        i2c_data += (uint16_t)SSPBUF;
                        i2c_messageLength = 3;
                    break;
                    case 3:
                        i2c_data += ((uint16_t)SSPBUF)<<8;
                        writeI2Cregister(i2c_data);
                        //intended fallthrough, when message complete
                    default: //or on error
                        i2c_data = 0;
                        i2c_messageLength = 0;
                    break;
                }
            }else{
                volatile uint8_t discarded = SSPBUF;     //discard address
                i2c_messageLength = 1;
            }
        }
        SSPCONbits.CKP = 1;  //stop clock stretching
        PIR1bits.SSPIF = 0;  //reset interrupt flag
    }
}

void setup(){
    //setup PORTA
    PORTA = 0x00;
    TRISA = 0x3F; //all channels input
    //setup PORTB
    PORTB = 0xF6;
    TRISB = 0x09; //INT input, PGM input, all others output
    //setup PORTC
    PORTC = 0x20; //set loadDac high
    TRISC = 0x9B; //all input, but HV_en RC2 and PIC_DAC_Load RC5
    
    //setup I2C
    SSPADD = i2caddress << 1;
    SSPSTATbits.SMP  = 1;  //100KHz mode
    SSPSTATbits.CKE  = 0;  //I2C spec
    SSPCONbits.CKP   = 1;
    SSPCONbits.SSPM  = 0x6; //i2c slave, 7bit address
    SSPCONbits.SSPEN = 1;
    //I2C interrupt
    PIR1bits.SSPIF   = 0;  //reset interrupt flag
    PIE1bits.SSPIE   = 1;  //enable i2c interrupt
    INTCONbits.GIE   = 1;  //enable global interrupts
    INTCONbits.PEIE  = 1;  //enable peripheral interrupts
    
    //setup timer2 and PWM output
    PR2               = 16; //total frequency _XTAL_FREQ/256 = 31.25 kHz
    CCPR1L            = 8;  //50% PWM ratio generating HV
    T2CONbits.T2CKPS  = 0;  //prescaler  1
    T2CONbits.TMR2ON  = 1;  //start timer2
    CCP1CONbits.CCP1M = 12;  //PWM mode 
    
    //setup ADC
    ADCON1bits.ADFM  = 1;
    ADCON1bits.PCFG  = 0xC; // RA3 = VREF+, RA2 = VREF-, RA5, RA1 and RA0 inputs;
    ADCON0bits.ADCS  = 2; //clock source for ADC, F_osc/32, good for XTAL > 5MHz
    ADCON0bits.CHS   = 0; //select RA0 as input
    ADCON0bits.ADON  = 1; //enable ADC peripheral
    
}

void main(void) {
    setup();
    setLedVoltage(0);
    setPmtVoltage(0);
    setGainVoltage(0);
    setfbGainVoltage(0);
    while(1){
        __delay_ms(1000);
        PORTCbits.RC6 = 0x00; //reset pin RC6 to off
    }
}

//input value of 0xffff corresponds to 4.095V
void setPmtVoltage(uint16_t value){
    shiftOutAndLoadDAC(0xf00 & (value>>4), _PORTB_RB6_MASK, 0);
    shiftOutAndLoadDAC(value, _PORTB_RB6_MASK, 1);
}

void setGainVoltage(uint16_t value){
    shiftOutAndLoadDAC(0xf00 & (value>>4), _PORTB_RB1_MASK, 1);
    shiftOutAndLoadDAC(value, _PORTB_RB1_MASK, 0);
}

void setLedVoltage(uint16_t value){
    shiftOutAndLoadDAC(value, _PORTB_RB2_MASK, 1);
}

void setfbGainVoltage(uint16_t value){
    shiftOutAndLoadDAC(value, _PORTB_RB2_MASK, 0);
}

//channel = 0 is A, channel = 1 is B
void shiftOutAndLoadDAC(uint16_t value, uint8_t AdcCsPin, uint8_t channel){
    PORTB &= ~AdcCsPin; //CS for DAC
    PORTBbits.RB4 = 0; //drive clock low
    PORTBbits.RB5 = 1; //do not load both channels
    PORTBbits.RB4 = 1; //drive clock high, 
    
    PORTBbits.RB4 = 0; //drive clock low
    PORTBbits.RB5 = channel; //do not load both channels
    PORTBbits.RB4 = 1; //drive clock high, 
    
    for(int8_t bitOffset = 11; bitOffset>=0; bitOffset--){
        PORTBbits.RB4 = 0; //drive clock low
        PORTBbits.RB5 = (uint8_t) ((value&(1<<bitOffset))>>bitOffset);
        PORTBbits.RB4 = 1; //drive clock high, 
    }
    PORTB |= AdcCsPin; //CS for DAC
    PORTCbits.RC5 = 0; //load value to output of DAC
    PORTCbits.RC5 = 1;
    return;
}

uint16_t getADCreading(char channel){
    ADCON0bits.CHS = channel;
    __delay_us(20);
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO){}; //wait for conversion to finish
    return ((uint16_t)(ADRESH<<8))+ADRESL;
}

uint16_t getPMTlowAmpSignal(void){
    return getADCreading(0);
}

uint16_t getPMThighAmpSignal(void){
    return getADCreading(1);
}

uint16_t getPMTvoltage(void){
    return getADCreading(4);
}