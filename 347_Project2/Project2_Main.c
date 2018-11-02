#include "ADCSWTrigger.h"
#include "tm4c123gh6pm.h"
#include "PLL.h"

#define SYSCTL_RCC_USEPWMDIV  0x00100000 // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M   0x000E0000 // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2   0x00000000 // PWM clock /2

#define LEFTFORWARD 					(*((volatile unsigned long *)0x40007010)) //PD2
#define LEFTBACKWARD 			 		(*((volatile unsigned long *)0x40007020)) //PD3
#define RIGHTFORWARD			   	(*((volatile unsigned long *)0x40007100)) //Pd6
#define RIGHTBACKWARD 				(*((volatile unsigned long *)0x40007200)) //Pd7
	
#define LED										(*((volatile unsigned long*)0x40025038)) // PF3-1

#define RED 0x02;
#define BLUE 0x04;
#define GREEN 0x08;

unsigned int s = 0;
	
unsigned long period    = 50000;
unsigned long dutyCycle = 45998;

// Calculated PWM for 0%, 25%, 50%, 75%, 100%
// Exact values for 0% and 100% are never used
double speeds[5] = {45998,30999,24998,12499,4};

unsigned char GO; 
unsigned char direction = 0xFF; 

unsigned char pressed = 0;
unsigned char released = 1;

//   Function Prototypes
void PortF_Init(void);
void PortD_Init(void);

//int period = 0; // set value that will give 40Hz / 25 ms
char sample = 0;
unsigned long ain1, ain2, ain3;

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void SysTick_Init(void);

volatile unsigned long ADCvalue;
// The digital number ADCvalue is a representation of the voltage on PE4 
// voltage  ADCvalue
// 0.00V     0
// 0.75V    1024
// 1.50V    2048
// 2.25V    3072
// 3.00V    4095
int main(void){unsigned long volatile delay;
  PLL_Init();                           // 80 MHz
  ADC_Init298();         // ADC initialization PE2/AIN1
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_DIR_R |= 0x04;             // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;          // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;             // enable digital I/O on PF2
                                        // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
	SysTick_Init();
	PLL_Init();                           // set system clock to 50 MHz
	PortD_Init();
	
  while(1){
    if(sample){
			sample = 0;	
			ADC_In298(&ain1, &ain2, &ain3); // Ensure sampler works
			GPIO_PORTF_DATA_R ^= 0x04;
			
			PWM1_0_CMPA_R = ain3 -1;
			PWM1_0_CMPB_R = ain3 -1;
		}
		
		LEFTFORWARD   = 0xFF;
		LEFTBACKWARD  = 0x00;					
	  RIGHTFORWARD  = 0xFF;
		RIGHTBACKWARD = 0x00; 
		
  }
}

// Can sample at 20 or 10 Hz also
void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = 2000000-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
}

void SysTick_Handler(){
	sample = 1;
}

void PortD_Init(void){ 
	volatile unsigned long delay;	
  SYSCTL_RCGC2_R 		 |= 0x00000008;  	// (a) activate clock for port D
	delay = SYSCTL_RCGC2_R;	
	
	
	GPIO_PORTD_LOCK_R = 0x4C4F434B; 
  GPIO_PORTD_CR_R = 0xCF;           // allow changes to PD0   
  GPIO_PORTD_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTD_DIR_R = 0xCF;          // 5) PD0-1 output    1100.1111
  GPIO_PORTD_DEN_R = 0xCF;          // 7) enable digital pins PD0-1   	
	
	SYSCTL_RCGCPWM_R 	 |= 0x00000002;		// STEP 1: activate clock for PWM Module 1
	SYSCTL_RCGCGPIO_R  |= 0x00000008;   // STEP 2: enable GPIO clock
	
  GPIO_PORTD_AFSEL_R |= 0x03;   			// STEP 3: enable alt function on   PD0-1
	
		// STEP 4: configure alt funt PF4-0
	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~0x000000FF)| 0x00000055;
	
	// STEP 5: configure the use of PWM divide
	SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;  // PWM divider
	SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;  // clear the PWM divider field
	SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;   // configure for /2 diveder

	// STEP 6: confiure genertor countdown mode
	PWM1_0_CTL_R  &= ~0xFFFFFFFF;
	PWM1_0_GENA_R |= 0x0000008C;
	PWM1_0_GENB_R |= 0x0000080C;
	
	PWM1_0_LOAD_R = period - 1;           // STEP 7 : set period
  PWM1_0_CMPA_R = dutyCycle - 1;    		// STEP 8: set duty cycle
  PWM1_0_CMPB_R =	dutyCycle - 1;
  PWM1_0_CTL_R  |= 0x00000001;   	  // STEP 9: start the M1PWM5 generator
  PWM1_ENABLE_R |= 0x00000003;			// STEP 10: enable   M1PWM0-1 outputs
}
