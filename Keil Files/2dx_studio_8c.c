
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

void ToF(void) { //gets one measurement
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
	uint16_t Distance;
	uint8_t dataReady;
	

	GPIO_PORTN_DATA_R ^= 0b00000001;                                    
  SysTick_Wait10ms(10);                                                        
  GPIO_PORTN_DATA_R ^= 0b00000001; 
	UART_printf(printf_buffer);
	
	/* Those basic I2C read functions can be used to check your own I2C functions */
	//status = VL53L1X_GetSensorId(dev, &wordData);
	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(5);
	}

	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	/* This function must to be called to initialize the sensor with the default setting  */
	status = VL53L1X_SensorInit(dev);

	/* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

		status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	//wait until the ToF sensor's data is ready
	while (dataReady == 0){
		status = VL53L1X_CheckForDataReady(dev, &dataReady);
		VL53L1_WaitMs(dev, 5);
	}
	dataReady = 0;
	 
	//read the data values from ToF sensor
	status = (VL53L1X_GetDistance(dev, &Distance));					//The Measured Distance value
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
	// print the resulted readings to UART
	sprintf(printf_buffer,"%u\r\n",Distance);
	// UART_printf(printf_buffer);
	SysTick_Wait10ms(5);
}

void PortM0_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;  //0b00000000    								      // Make PM0 input
  GPIO_PORTM_DEN_R = 0b00001111;  //0b00000001
	return;
}

void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 // Activate the clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTH_DIR_R = 0b00001111;  //0b00000000    								      
  GPIO_PORTH_DEN_R = 0b00001111;  //0b00000011
	return;
}

void rotate(int delay, int step){ //clock wise rotation
	
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1us(delay);
		}
		
}

void rotateCounter(int delay, int step){ //Counter clock wise rotation
	
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1us(delay);
		}
		
}

void PortQ_Init(void) {
	//Use PortQ pins for output
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R14;                // activate clock for Port Q
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R14) == 0){};    // allow time for clock to stabilize
  GPIO_PORTQ_DIR_R |= 0xFF;                                        // make PN0 out (PN0 built-in LED1)
  GPIO_PORTQ_DEN_R |= 0xFF;                                        // enable digital I/O on PN0

    return;
}


void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************


int main(void) {

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH0H1H2H3_Init();
	PortM0_Init();
	PortQ_Init();
	
	int enable = 0;
	int delay = 2000; //delay to get fastest functional spin
	int step = 1;
	int count = 0;
	int enableToF = 0;
	int enableToF2 = 0;
	int rotateClockwise = 1;
	int rotateCount = 0;
	int ToF_zero = 0;
	int turn_off_ToF = 0;
	
	/// Showcasing bus speed, uncomment for measuring with waveforms
//	while(1) {
//		SysTick_Wait(1000000);
//		GPIO_PORTQ_DATA_R ^= 0b11111111;
//	} 
	
	while(1){	
		while((GPIO_PORTM_DATA_R&0b00000010)==0){ //switch is active low, so when Port M1 sees 0
			count = 0;
			enableToF = !enableToF;
			while((GPIO_PORTM_DATA_R&0b00000010)==0) {SysTick_Wait1us(10);} //blocking statement for reaction
		}
		while((GPIO_PORTM_DATA_R&0b00000001)==0){ //switch is active low, so when Port M0 sees 0
			enable = !enable;
			while((GPIO_PORTM_DATA_R&0b00000001)==0) {SysTick_Wait1us(10);} //blocking statement for reaction
		}
		
		if(enableToF && rotateCount == 0){
			ToF_zero = 1;
		}
		
		if(!enableToF){
			ToF_zero = 0;
		}
		
		if(enable){
			if(rotateClockwise){
				rotateCounter(delay, step);
				rotateCount++;
				
		}else{
			rotate(delay, step);
			rotateCount++;
			}
		}
		
		if(ToF_zero){
			count++;
		}
		
		if(count % 16 == 0 && count != 0 ){	// 360/32 = 11.25, 512/16 = 32
			ToF();
		}
		
		if(rotateCount >= 512){
			rotateCount = 0; 
			rotateClockwise = !rotateClockwise;
		}
		
			
		if(count >= 512 ){ //512 steps is a full rotation, deemed to be 512 in studio 4B
			//enable = 0; //stops stepper motor after 360 degrees
			//enableToF = 0; //stops time of flight sensor
			count = 0; //reset count so we can operate again
		}
	}
}
