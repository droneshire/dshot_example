#include "main.h"


///////////////////////////////////////////////////////////////////////////////////////////////
// SW Dshot 150 for the F0 driving 4 outputs
///////////////////////////////////////////////////////////////////////////////////////////////

// defines from SPL that are used 
//-----------------------------------------------------------------------------------------------------------------------------------------
#define TIM_IT_Update             ((uint16_t)0x0001)
#define TIM_FLAG_Update        ((uint16_t)0x0001)
#define GPIO_Mode_OUT          ((uint16_t)0x0001)
#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected    */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected    */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected    */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected    */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected    */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected    */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected    */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected    */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected    */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected    */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected   */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected   */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected   */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected   */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected   */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected   */
// end of defines from SPL
//-----------------------------------------------------------------------------------------------------------------------------------------

// SW Dshot defines
//-----------------------------------------------------------------------------------------------------------------------------------------
#define DSHOT_PIN1 GPIO_Pin_0
#define DSHOT_PIN1_NUM 0
#define DSHOT_PIN1_GPIO GPIOA

#define DSHOT_PIN2 GPIO_Pin_1
#define DSHOT_PIN2_NUM 1
#define DSHOT_PIN2_GPIO GPIOA

#define DSHOT_PIN3 GPIO_Pin_2
#define DSHOT_PIN3_NUM 2
#define DSHOT_PIN3_GPIO GPIOA

#define DSHOT_PIN4 GPIO_Pin_3
#define DSHOT_PIN4_NUM 3
#define DSHOT_PIN4_GPIO GPIOA

#define SW_DSHOT_TIM TIM15
#define SW_DSHOT_TIM_IRQn TIM15_IRQn
#define SW_DSHOT_TIM_IRQHandler TIM15_IRQHandler
#define SW_DSHOT_TIM_APB_ENABLE RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

#define DSHOT_BIT_WIDTH 107// timer tics. timer runs with 16Mhz in this demo so it will be ~150k


static uint8_t DshotBitArr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint8_t Dshot_One_Time, Dshot_Zero_Time, Dshot_Till0_Time, Dshot_Till1_Time, Dshot_complete_Time;

void initSoftDshot(void){
	
	// calculate timings
	Dshot_One_Time = DSHOT_BIT_WIDTH/4*3;
	Dshot_Zero_Time = Dshot_One_Time/2;

	Dshot_Till0_Time = Dshot_Zero_Time;
	Dshot_Till1_Time = Dshot_One_Time-Dshot_Zero_Time;
	Dshot_complete_Time = DSHOT_BIT_WIDTH-Dshot_One_Time;
	
	// enable clock
	SW_DSHOT_TIM_APB_ENABLE;
	
	// init IO pins as output
	DSHOT_PIN1_GPIO->MODER |= (((uint32_t)GPIO_Mode_OUT) << (DSHOT_PIN1_NUM*2));
	DSHOT_PIN2_GPIO->MODER |= (((uint32_t)GPIO_Mode_OUT) << (DSHOT_PIN2_NUM*2));
	DSHOT_PIN3_GPIO->MODER |= (((uint32_t)GPIO_Mode_OUT) << (DSHOT_PIN3_NUM*2));
	DSHOT_PIN4_GPIO->MODER |= (((uint32_t)GPIO_Mode_OUT) << (DSHOT_PIN4_NUM*2));
	
	// all pins low
	DSHOT_PIN1_GPIO->BRR = DSHOT_PIN1;
	DSHOT_PIN2_GPIO->BRR = DSHOT_PIN2;
	DSHOT_PIN3_GPIO->BRR = DSHOT_PIN3;
	DSHOT_PIN4_GPIO->BRR = DSHOT_PIN4;
	
	// init timer
	SW_DSHOT_TIM->PSC = 2; // run with 16mhz (from 48Mhz mcu clock)
	SW_DSHOT_TIM->CNT = 0;
	
	// init interrupt
	NVIC_SetPriority(SW_DSHOT_TIM_IRQn , 0); // set it to highest priority
	SW_DSHOT_TIM->DIER |= TIM_IT_Update; // enable update interrupt
	SW_DSHOT_TIM->CR1 |= TIM_CR1_CEN; // enable tim 
}

void StartSoftDshot(uint16_t *DshotValues, uint8_t *DshotTlmRequests){
	
	// generate bit arrays
	for(uint8_t i = 0; i<4; i++){
		uint8_t bitArr[16];

		uint16_t MotorVal11B = DshotValues[i];
		
		
		// normalize
		if(MotorVal11B > 2047) MotorVal11B = 2047;
		if(MotorVal11B < 48) MotorVal11B = 0; 
			
		bitArr[0]   = (MotorVal11B&0x400) ? 1 : 0;
		bitArr[1]   = (MotorVal11B&0x200) ? 1 : 0;
		bitArr[2]   = (MotorVal11B&0x100) ? 1 : 0;
		bitArr[3]   = (MotorVal11B&0x80)   ? 1 : 0;
		bitArr[4]   = (MotorVal11B&0x40)   ? 1 : 0;
		bitArr[5]   = (MotorVal11B&0x20)   ? 1 : 0;
		bitArr[6]   = (MotorVal11B&0x10)   ? 1 : 0;
		bitArr[7]   = (MotorVal11B&0x8)     ? 1 : 0;
		bitArr[8]   = (MotorVal11B&0x4)     ? 1 : 0;
		bitArr[9]   = (MotorVal11B&0x2)     ? 1 : 0;
		bitArr[10] = (MotorVal11B&0x1)     ? 1 : 0;
		bitArr[11] = (DshotTlmRequests[i]&0x01);

		//getCRC
		bitArr[12] = bitArr[0]^bitArr[4]^bitArr[  8];
		bitArr[13] = bitArr[1]^bitArr[5]^bitArr[  9];
		bitArr[14] = bitArr[2]^bitArr[6]^bitArr[10];
		bitArr[15] = bitArr[3]^bitArr[7]^bitArr[11];

		for(uint8_t j = 0; j<16; j++){
			if(i == 0) DshotBitArr[j] = (!bitArr[j]);
			else DshotBitArr[j] += (!bitArr[j])<<i;
		}
	}	
	
	SW_DSHOT_TIM->ARR = 0xFFFF; // to not have it overrun again
	SW_DSHOT_TIM->CNT = 0xFFFE; // to have it overflow instandly
	NVIC_EnableIRQ(SW_DSHOT_TIM_IRQn);
}

// generate Dshots
void __attribute__((optimize("O1"))) SW_DSHOT_TIM_IRQHandler(void){
	static uint8_t DshotPos = 0; // 0-16
	static uint8_t DshotBitStep = 0; // 0 = risingEdge, 1= bit 0 falling edge, 2 bit 1 falling edge
	SW_DSHOT_TIM->SR = (uint16_t)~TIM_FLAG_Update; // clear update flag
	
	if(DshotBitStep == 0){ // rising edage
		// all Pins LOW
		DSHOT_PIN1_GPIO->BSRR = DSHOT_PIN1;
		DSHOT_PIN2_GPIO->BSRR = DSHOT_PIN2;
		DSHOT_PIN3_GPIO->BSRR = DSHOT_PIN3;
		DSHOT_PIN4_GPIO->BSRR = DSHOT_PIN4;
		SW_DSHOT_TIM->ARR = Dshot_Till0_Time; // set the next interrupt time
		DshotBitStep = 1;
	}else if(DshotBitStep == 1){
		uint8_t checkStat = DshotBitArr[DshotPos];
		if(checkStat&0x1) DSHOT_PIN1_GPIO->BRR = DSHOT_PIN1;
		if(checkStat&0x2) DSHOT_PIN2_GPIO->BRR = DSHOT_PIN2;
		if(checkStat&0x4) DSHOT_PIN3_GPIO->BRR = DSHOT_PIN3;
		if(checkStat&0x8) DSHOT_PIN4_GPIO->BRR = DSHOT_PIN4;
		SW_DSHOT_TIM->ARR = Dshot_Till1_Time; // set the next interrupt time
		DshotBitStep = 2;
	}else{
		// do all pins low is faster then check if they already low
		DSHOT_PIN1_GPIO->BRR = DSHOT_PIN1;
		DSHOT_PIN2_GPIO->BRR = DSHOT_PIN2;
		DSHOT_PIN3_GPIO->BRR = DSHOT_PIN3;
		DSHOT_PIN4_GPIO->BRR = DSHOT_PIN4;
		SW_DSHOT_TIM->ARR = Dshot_complete_Time; // set the next interrupt time
		DshotBitStep = 0;
		if(++DshotPos == 16){
			NVIC_DisableIRQ(SW_DSHOT_TIM_IRQn);
			DshotPos = 0; 
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Demo Loop
///////////////////////////////////////////////////////////////////////////////////////////////

#define LOOPTIME_us 2500 // 400hz looptime

int main(void){
	// init clock
	SystemInit();
	
	//GPIO clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; 
	
	// tim 3 to have a delay timer for that demo
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 47; // run with 1mhz (from 48Mhz mcu clock)
	TIM3->ARR = 0xFFFF;
	TIM3->CR1 |= TIM_CR1_CEN; // enable tim 
	
	initSoftDshot();
	
	while (1){
		static uint8_t TLMrequests[4] = {0,0,0,0};
		static uint16_t MotorValues[4] = {0,0,0,0};
		static uint32_t loopCounter = 0;
		uint16_t loopStart = TIM3->CNT;
		
		
		// ramp the motors up and down
		if(loopCounter > 800){// after 2 seconds
			static uint8_t countUpMotor1 = 1;
			if(MotorValues[0] < 2047 && countUpMotor1)MotorValues[0]++;
			if(MotorValues[0] > 0 && !countUpMotor1)MotorValues[0]--;
			if(MotorValues[0] == 2047 && countUpMotor1) countUpMotor1 = 0;
			if(MotorValues[0] == 0 && !countUpMotor1) countUpMotor1 = 1;
		}
		
		if(loopCounter > 900){
			static uint8_t countUpMotor2 = 1;
			if(MotorValues[1] < 2047 && countUpMotor2)MotorValues[1]++;
			if(MotorValues[1] > 0 && !countUpMotor2)MotorValues[1]--;
			if(MotorValues[1] == 2047 && countUpMotor2) countUpMotor2 = 0;
			if(MotorValues[1] == 0 && !countUpMotor2) countUpMotor2 = 1;
		}
		
		if(loopCounter > 1000){
			static uint8_t countUpMotor3 = 1;
			if(MotorValues[2] < 2047 && countUpMotor3)MotorValues[2]++;
			if(MotorValues[2] > 0 && !countUpMotor3)MotorValues[2]--;
			if(MotorValues[2] == 2047 && countUpMotor3) countUpMotor3 = 0;
			if(MotorValues[2] == 0 && !countUpMotor3) countUpMotor3 = 1;
		}
		
		if(loopCounter > 1100){
			static uint8_t countUpMotor4 = 1;
			if(MotorValues[3] < 2047 && countUpMotor4)MotorValues[3]++;
			if(MotorValues[3] > 0 && !countUpMotor4)MotorValues[3]--;
			if(MotorValues[3] == 2047 && countUpMotor4) countUpMotor4 = 0;
			if(MotorValues[3] == 0 && !countUpMotor4) countUpMotor4 = 1;
		}
		
		StartSoftDshot(MotorValues, TLMrequests);
		
		loopCounter++;
		while((uint16_t)((uint16_t)TIM3->CNT - (uint16_t)loopStart) < LOOPTIME_us); // wait till the looptime is over		
	}
}









