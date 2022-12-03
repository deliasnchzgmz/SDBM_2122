/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
#include "stm32l152c_discovery.h"
#include "stm32l152c_discovery_glass_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned char game;
// game 1
unsigned char pressed = 0;
unsigned char display[6];
bool active1 = false;
bool active2 = false;
bool change = false;

// game 2
unsigned short numero;
unsigned char numero_ant = 0;
unsigned int random_count;
signed int player1_time = 0;
signed int player2_time = 0;
unsigned int game2_winner = 0;
bool end_game2 = false;
unsigned int ADC_value = 0;
unsigned int mode = 0;
bool isNegative = false;
unsigned short before[3] = {1668, 2000, 2500};
unsigned short after[3] = {2500, 2000, 1668};
unsigned short current_note = 0;
unsigned short prev_note = 0;
bool sound = false;
bool end_count = false;

uint8_t input[2] = " ";
bool init = true;
uint8_t message[103] = "\n\rWelcome to the game of reflexes, select: game 1 REACTION TIME (press 1), game 2 COUNTDOWN (press 2)\n\r";
uint8_t g1[10] = "\n\rGAME 1\n\r";
uint8_t g2[10] = "\n\rGAME 2\n\r";
uint8_t winner_p1[29] = "\n\rThe winner is Player 1!!\n\r";
uint8_t winner_p2[29] = "\n\rThe winner is Player 2!!\n\r";
bool listen = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
static void MX_DAC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void initial(){

    for(int j=0;j<103;j++){
      HAL_UART_Transmit(&huart3, &(message[j]), 1, 10000);
    }
    listen = true;
    while(input[1]!=49&&input[1]!=50);
    listen = false;
    switch(input[1]){
    case 49:
      game = 1;
      break;
    case 50:
      game = 2;
      break;
    }
    input[1] = 32;
}

int min(int a, int b){

  int c = abs(a);
  int d = abs(b);
  if(!change&&(a!=b)){
    return (c > d) ? 2 : 1; // returns 1 if abs(player1_time) is smaller than abs(player2_time)
  }else{
    return 3;
  }

}

void Bin2Ascii(unsigned short number, unsigned char* cadena){

  unsigned short parcial, j, cociente, divisor;

  parcial = number;
  divisor = 10000;
  *(cadena) = ' ';
  for(j=1;j<6;j++){
    cociente =parcial/divisor;
    *(cadena+j) = '0'+ (unsigned char) cociente;
    parcial = parcial - (cociente * divisor);
    divisor = divisor / 10;
  }
  *(cadena+6) = 0;

}

void EXTI0_IRQHandler(void){
  if (EXTI->PR!=0){
    change = true;
      if(game == 2){
        game=1;
      }
      else{
        game = 2;
      }
      EXTI->PR = 0x01;
  }
}

void EXTI9_5_IRQHandler(void){

  switch(game){
  case 1: // GAME 1
    if((EXTI->PR)==0x0040){ // PB6, BUTTON 1
        if(active1){
          pressed = 1;
          Bin2Ascii((TIM4->CNT),display);
        }
        EXTI->PR = 0x0040;
      }
    if((EXTI->PR)==0x0080){ // PB7, BUTTON 2
      if(active1){
        pressed = 2;
        Bin2Ascii((TIM4->CNT),display);
      }
      EXTI->PR = 0x0080;
    }
    break;

  case 2: // GAME 2
    if((EXTI->PR)==0x0040){ //PB6, BUTTON 1
        if(active2){
          player1_time = (TIM4->CNT)-(10*mode); // stores the time the button is pressed
        }
        EXTI->PR = 0x0040;
      }
    if((EXTI->PR)==0x0080){ // PB7, BUTTON 2
        if(active2){
          player2_time = (TIM4->CNT)-(10*mode); // stores the time the button is pressed
        }
        EXTI->PR = 0x0080;
      }
    break;
  }

}

void TIM3_IRQHandler(void) {
 if ((TIM3->SR & 0x0004)!=0){ // Is the event channel 2 from TIM3?
     switch(game){
       case 1:
         if((pressed==0)&&(!change)&&(!active1)){
           GPIOD->BSRR = (1<<2);
           TIM4->CNT = 0;
           TIM4->CCR1 = 50*1000;
           TIM4->CR1 |= 0x0001; // CEN = 1 -> Starting CNT
           TIM4->EGR |= 0x0001; //Update registers
           TIM3->CR1 = 0x0000;
           TIM3->CNT = 0;
           TIM3->EGR |= 0x0001;
           active1 = true;
         }else if((pressed!=0)&&(!change)&&(active1)){
           active1 = false;
         }
         break;
       case 2:
         if ((TIM3->SR & 0x0004)!=0){ // If the comparison is successful, then the IRQ is launched
                                      // and this ISR is executed. This line check which event
                                       // launched the ISR
           if(end_count&&player1_time==0&&player2_time==0){ //if none of the players has pressed the button after 2s
             end_game2 = true;
           }
           if(numero==0&&!end_count){
             end_count = true;
             TIM3->CR1 = 0x0000; // CEN = 1 -> stop counter
             TIM3->EGR |= 0x0001; // UG = 1 -> Generate an update event to update all registers
             TIM3->CCR2 = 2000;
             TIM3->CNT = 0;
             TIM3->CR1 |= 0x0001;
             TIM3->EGR |= 0x0001;
           }else{
             if(numero>=0){
               numero--; // Decrease in 1 the number to be shown in the LCD
             }
             TIM3->CCR2 += mode; // Update the comparison value, adding 1000 steps = 1 second
             TIM3->SR = 0x0000; // Clear all flags
           }
           if((game2_winner!=0)&&sound){
             if(current_note<2){
               current_note++;
             }else{
               sound = false;
               current_note = 0;
               TIM3->CR1 = 0x0000; // CEN = 1 -> Stopping CNT
               TIM3->CNT = 0;
               TIM3->EGR = 0x0001; // UG = 1 -> Update event*/
               TIM3->SR = 0x0000;
             }
           }

           if(((game2_winner!=0))&&active2){//wait 3s until the game restarts
             active2 = false;
           }

         break;
     }

 }
 TIM3->SR = 0x0000;
}
}

void espera(int tiempo){
  while (tiempo>0&&(!change)){
    tiempo--;
  }
}

void game1(){ //reaction time game implementation function
  TIM3->CNT = 0;
  TIM3->CCR2 = ((rand() % (10 - 3 + 1)) + 3)*1000;
  TIM3->CR1 |= 0x0001; // CEN = 1 -> Starting CNT
  TIM3->EGR |= 0x0001; // UG = 1 -> Update event*/
  while(pressed == 0&&(!change)); //waiting
  switch (pressed){
    case 1:
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t *)" P1");
      for(int j=0;j<29;j++){
        HAL_UART_Transmit(&huart3, &(winner_p1[j]), 1, 10000);
      }
      espera(2000000);
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t *)display);
      for(int j=0;j<6;j++){
        HAL_UART_Transmit(&huart3, &(display[j]), 1, 10000);
        }
      break;
    case 2:
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t *)" P2");
      for(int j=0;j<29;j++){
        HAL_UART_Transmit(&huart3, &(winner_p2[j]), 1, 10000);
      }
      espera(2000000);
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t *)display);
      for(int j=0;j<6;j++){
        HAL_UART_Transmit(&huart3, &(display[j]), 1, 10000);
        }
    default:
      break;
    }
  TIM3->CNT = 0; // 3s for the game to restart
  TIM3->CR1 |= 0x0001;
  TIM3->EGR |= 0x0001;
  TIM3->CCR2 = 3000; //añado 3s
  while(active1&&(!change)); //wait some time to restart the game in case pa0 not pressed
  pressed = 0; //reset the game
  GPIOD->BSRR = (1 << 2)<<16;
  BSP_LCD_GLASS_Clear(); //clear lcd
  TIM3->CR1 = 0x0000; // CEN = 1 -> Stopping CNT
  TIM3->CNT = 0;
  TIM3->EGR = 0x0001; // UG = 1 -> Update event*/
  TIM3->SR = 0x0000;
  TIM4->CR1 = 0x0000; // CEN = 1 -> Stopping CNT
  TIM4->CNT = 0;
  TIM4->EGR = 0x0001; // UG = 1 -> Update event*/
  TIM4->SR = 0x0000;
}

void countdown(){
  while((numero>0)&&(active2)&&(!change)){
    if ((numero_ant != numero)&&(numero>=random_count)) { // If the ISR has changed the number to show …
        numero_ant = numero; // Store new number to be used next time
        Bin2Ascii(numero, display); // Convert number to text
        BSP_LCD_GLASS_Clear();
        BSP_LCD_GLASS_DisplayString((uint8_t *)display); // Display number in the LCD
    }else if((numero<random_count)){
      BSP_LCD_GLASS_Clear();
    }
  }
}

void playSequence(bool n){
  sound = true;
  if(n){ //if the winner pressed the button before the countdown ended
    while(sound&&!change&&!end_game2){
      prev_note = current_note;
      TIM2->ARR = before[current_note];
      TIM2->CCR1 = before[current_note]/2;
      TIM3->CNT = 0;
      TIM3->CCR2 = 250;
      TIM3->CR1 |= 0x0001;
      TIM3->EGR |= 0x0001;
      while(current_note==prev_note&&!change); //while note is playing
    }
  }else{
    while(sound&&!change&&!end_game2){
      prev_note = current_note;
      TIM2->ARR = after[current_note];
      TIM2->CCR1 = after[current_note]/2;
      TIM3->CNT = 0;
      TIM3->CCR2 = 250;
      TIM3->CR1 |= 0x0001;
      TIM3->EGR |= 0x0001;
      while(current_note==prev_note&&!change); //while note is playing
  }
}
  TIM2->ARR = 2;
  TIM2->CCR1 = 0;
}

void game2(int mode){ // countdown game implementation function
  numero = 10;
  end_game2 = false;
  end_count = false;
  random_count = ((rand() % (8 - 3 + 1)) + 3); //(rand() % (upper - lower + 1)) + lower;
  TIM3->CNT = 0;
  TIM3->CCR2 = mode; // Record that stores the value for the comparison.
  TIM3->CR1 |= 0x0001; // Enable the counter
  TIM4->CNT = 0;
  TIM4->CR1 |= 0x0001;
  TIM4->EGR |= 0x0001;
  TIM3->EGR |= 0x0001; //Update registers
  active2 = true;
  countdown(); // countdown starts
  while(((player1_time==0)||(player2_time==0))&&(!change)&&(!end_game2)); // wait until both players have pressed their buttons
  game2_winner = min(player1_time, player2_time);

  switch(game2_winner){
  case 1:
    if(player1_time<0){ // if player pressed the button before the countdown ended
      isNegative = true;
      player1_time = abs(player1_time);
      Bin2Ascii(player1_time, display);
      for(int i = 0; i<6;i++){
        if(display[i]!=0x20&&display[i]!=0x30){
          break;
        }else if(display[i+1]!=0x30){
          display[i] = 0x2D;  // write '-' before the absolute value number
          if(i>0&&display[i-1]==0x30){
            display[i-1] = 0x20; // write space before '-'
          }
          break;
        }
      }
    }else{
      Bin2Ascii(player1_time, display);
      isNegative = false;
    }
    GPIOD->BSRR |= (1 << 2);
    BSP_LCD_GLASS_Clear();
    BSP_LCD_GLASS_DisplayString((uint8_t *)" P1");
    for(int j=0;j<29;j++){
      HAL_UART_Transmit(&huart3, &(winner_p1[j]), 1, 10000);
    }
    espera(2000000);
    BSP_LCD_GLASS_Clear();
    BSP_LCD_GLASS_DisplayString((uint8_t *)display); // Display number in the LCD
    for(int j=0;j<6;j++){
      HAL_UART_Transmit(&huart3, &(display[j]), 1, 10000);
      }
    break;
  case 2:
    if(player2_time<0){ // if player pressed the button before the countdown ended
      isNegative = true;
      player2_time = abs(player2_time);
      Bin2Ascii(player2_time, display);
      for(int i = 0; i<6;i++){
        if(display[i]!=0x20&&display[i]!=0x30){
          break;
        }else if(display[i+1]!=0x30){
          display[i] = 0x2D;
          if(i>0&&display[i-1]==0x30){
            display[i-1] = 0x20; // write space before '-'
          }
          break;
        }
      }
    }else{
      Bin2Ascii(player2_time, display);
      isNegative = false;
    }
    GPIOA->BSRR |= (1 << 12);
    BSP_LCD_GLASS_Clear();
    BSP_LCD_GLASS_DisplayString((uint8_t *)" P2");
    for(int j=0;j<29;j++){
      HAL_UART_Transmit(&huart3, &(winner_p2[j]), 1, 10000);
    }
    espera(2000000);
    BSP_LCD_GLASS_Clear();
    BSP_LCD_GLASS_DisplayString((uint8_t *)display); // Display number in the LCD
    for(int j=0;j<6;j++){
      HAL_UART_Transmit(&huart3, &(display[j]), 1, 10000);
      }
    break;
  default:
    break;

  }
  playSequence(isNegative);
  if(end_game2){
    BSP_LCD_GLASS_Clear();
    BSP_LCD_GLASS_DisplayString((uint8_t *)" END");
  }
  TIM3->CR1 = 0;
  TIM3->EGR |= 0x0001;
  TIM3->CNT = 0;
  TIM3->CR1 |= 0x0001;
  TIM3->EGR |= 0x0001;
  TIM3->CCR2 = 3000; // set 3s to restart
  while((active2)&&(!change));
  TIM4->CNT = 0;
  TIM3->CNT = 0;
  TIM4->CR1 = 0x0000;
  TIM3->CR1 = 0x0000;
  TIM4->EGR |= 0x0001;
  TIM3->EGR |= 0x0001; // stop game 2

  game2_winner = 0;
  player1_time = 0;
  player2_time = 0; // reset game 2

  GPIOA->BSRR |= (1 << 12)<<16;
  GPIOD->BSRR |= (1 << 2)<<16;

  BSP_LCD_GLASS_Clear();
}

int game2_mode(int value){
  unsigned int output = 0;
  if (value<1365){
    output = 500; //TIM3->CCR2 500ms
  }else if(value>=1365&&value<2730){
    output = 1000; // TIM3->CCR2 1s
  }else if(value>=2730){
    output = 2000; // TIM3->CCR2 2s
  }
  return output;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_BarLevelConfig(0);
  BSP_LCD_GLASS_Clear();

  MX_GPIO_Init();
  MX_ADC_Init();
  MX_LCD_Init();
  MX_TS_Init();
  MX_DAC_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //PA0 configured as digital input (00) and pull-down(10)
   GPIOA->MODER &= ~(1<<0*2);
   GPIOA->MODER &= ~(1<<(0*2+1));
   GPIOA->PUPDR &= ~(1<<0*2);
   GPIOA->PUPDR |= (1<<(0*2+1));
   EXTI->RTSR |= (1<<0); //rising edge triggers interruption
   EXTI->FTSR &= ~(1<<0); //falling edge disable
   SYSCFG->EXTICR[0] = 0;//EXTI0 associated with PA0
   EXTI->IMR |= 0X01; //unmask exti0
   NVIC->ISER[0] |= (1<<6);

   //PB7 configured as digital input
   GPIOB->MODER &= ~(1<<7*2);
   GPIOB->MODER &= ~(1<<(7*2+1)); //PB7 BUTTON PLAYER 2
   GPIOB->PUPDR &= ~(1<<(7*2+1)); //Pull up
   GPIOB->PUPDR |= (1<<(7*2));
   EXTI->FTSR |= (1<<7); //FALLING edge triggers interruption
   EXTI->RTSR &= ~(1<<7);  //RISING edge disable
   //SYSCFG->EXTICR[1] = 0x1000; //EXTI7 associated with PB7
   EXTI->IMR |= 0X0080;  //UNMASK EXTI7
   //NVIC->ISER[0] |= (1<<23);


   //PB6 configured as digital input
   GPIOB->MODER &= ~(1<<6*2);
   GPIOB->MODER &= ~(1<<(6*2+1)); //PB6 BUTTON PLAYER 1
   GPIOB->PUPDR &= ~(1<<(6*2+1)); //Pull up
   GPIOB->PUPDR |= (1<<(6*2));
   EXTI->FTSR |= (1<<6); //FALLING edge triggers interruption
   EXTI->RTSR &= ~(1<<6); //RISING edge disable
   SYSCFG->EXTICR[1] = 0x1100; //EXTI6 associated with PB6 and PB7
   EXTI->IMR |= 0X0040;  //UNMASK EXTI6
   NVIC->ISER[0] |= (1<<23);

   //LEDS configuration (PA12 AND PD2)
   // DIGITAL OUTPUT (01)
   GPIOA->MODER |= (1<<12*2);
   GPIOA->MODER &= ~(1<<(12*2+1));

   GPIOD->MODER |= (1<<2*2);
   GPIOD->MODER &= ~(1<<(2*2+1));

   //TIM3CH2 TOC WITH INTERRUPTION
   // Internal clock selection: CR1, CR2, SMRC
   TIM3->CR1 = 0x0000; // ARPE = 0 -> Not PWM, it is TOC
   // CEN = 0; Counter off
   TIM3->CR2 = 0x0000; // Always 0x0000 in this subject
   TIM3->SMCR = 0x0000; // Always 0x0000 in this subject
   // Setting up the counter functionality: PSC, CNT, ARR y CCRx
   TIM3->PSC = 31999; // Preescaler=32000 -> F_counter=32000000/32000 = 1000 steps/second
   TIM3->CNT = 0; // Initial value for CNT
   TIM3->ARR = 0xFFFF; // Recommended value = FFFF
   TIM3->CCR2 = 0;
   // IRQ or no-IRQ selection: DIER
   TIM3->DIER = 0x0004; // IRQ enabled only for channel 2 -> CC2IE = 1
   // Counter output mode
   TIM3->CCMR1 = 0x0000; // CC2S = 0 (TOC)
   // OC2M = 011 (external output with toggle in channel 2)
   // OC2PE = 0 (without preload)
   TIM3->CCER = 0x0000; // CC2P = 0 (always for TOC)
   TIM3->SR = 0; // Clear all flags
   // Enabling IRQ source for TIM4 in NVIC (position 30)
   NVIC->ISER[0] |= (1 << 29);

   //TIM4CH1 TOC WITHOUT INTERRUPTION
   // Internal clock selection: CR1, CR2, SMRC
   TIM4->CR1 = 0x0000; // ARPE = 0 -> Not PWM, it is TOC
                       // CEN = 0; Counter off
   TIM4->CR2 = 0x0000; // Always 0x0000 in this subject
   TIM4->SMCR = 0x0000; // Always 0x0000 in this subject
   // Setting up the counter functionality: PSC, CNT, ARR y CCRx
   TIM4->PSC = 31999; // Preescaler=32000 -> F_counter=32000000/32000 = 1000 steps/second
   TIM4->CNT = 0; // Initial value for CNT
   TIM4->ARR = 0xFFFF; // Recommended value = FFFF
   TIM4->CCR1 = 0;
   // IRQ or no-IRQ selection: DIER
   TIM4->DIER = 0x0000; // NO IRQ ENABLED
   // Counter output mode
   TIM4->CCMR1 = 0x0000; // CC1S = 0 (TOC)
                         // OC1M = 011 (external output with toggle in channel 2)
                         // OC1PE = 0 (without preload)
   TIM4->CCER = 0x0000; // CC1P = 0 (always for TOC)
   TIM4->SR = 0; // Clear all flags

   // PA5 INITIALIZATION AS AF
   GPIOA->MODER |=  (1<<(5*2+1));
   GPIOA->MODER &= ~(1<<(5*2));
   // PA5 (TIM2_CH1) alternate function config
   GPIOA->AFR[0] &= ~(0x0F << (4*5));
   GPIOA->AFR[0] |= 1 << (4*5);

   // TIMER 2 CONFIGURATION
   TIM2->CR1 = 0; // clean
   TIM2->CR1 = 0x0080; // 1000 0000 ARPE bit on (0x0080)
   TIM2->CR2 = 0; // zero
   TIM2->SMCR = 0; // zero
   TIM2->PSC = 31;
   // Preescaler=31:f_timer=32000000/32 - 1 = 1000000 steps/s
   TIM2->CNT = 0; // Counter init
   TIM2->ARR = 2;
   TIM2->CCR1 = 0;
   // ARR = (Fclk / (Fpwm * (PSC + 1))) - 1
   // ARR for Fpwm 600 = 1667, CCR = 834
   // ARR for Fpwm 500 = 2000, CCR = 1000
   // ARR for Fpwm 400 = 2500, CCR = 1250
   // ARR for Fpwm 300 = 3333, CCR = 1667
   TIM2->DIER = 0x0000;
   TIM2->CCMR1 &= ~(0x0FF << (8*0)); // CCMR1 (channel 1)
   TIM2->CCMR1 |= ((6 << 4)|(1<<3));
   TIM2->CCER &= ~(0x0F << 0);
   TIM2->CCER |= (1 << 0);
   // Enable the counter
   TIM2->CR1 |= 0x0001;
   TIM2->EGR |= 0x0001;
   TIM2->SR = 0;

   // ADC CONFIGURATION
   // POTENTIOMETER CONNECTED TO PA4
   // It has to retrieve the voltage value  (0, 1365, 2730)
   GPIOA->MODER |= (1 << 4*2);  // Analog mode (11)
   GPIOA->MODER |= (1 << (4*2+1));
   // We use channel 1
   ADC1->CR2 &= ~(1 << 0); //ADON=0 ADC powered off
   ADC1->CR1 = 0x00000000; // OVRIE = 0 (overrun IRQ disabled)
                           // RES = 00 (resolution = 12 bits)
                           // SCAN = 0 (scan mode disabled)
                           // EOCIE = 0 (EOC IRQ disabled)

   ADC1->CR2 = 0x00000412; // EOCS = 1 (EOC is activated after each conversion)
                           // DELS = 001 (delay till data is read)
                           // CONT = 1 (continuous conversion)
   ADC1->SQR1 = 0x00000000; // 1 channel in the sequence
   ADC1->SQR5 = 0x00000004; // The selected channel is AIN4 (check pins table)
   ADC1->CR2 |= 0x00000001; // ADC powered ON
   HAL_UART_Receive_IT(&huart3, &input[1], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    initial();
    change = false;
    input[1] = 0;
      switch(game){
      case 1:

        BSP_LCD_GLASS_Clear();
        BSP_LCD_GLASS_DisplayString((uint8_t *)" GAME1");
        for(int j=0;j<10;j++){
          HAL_UART_Transmit(&huart3, &(g1[j]), 1, 10000);
        }
        game1();
        break;
       case 2:
        ADC1->CR2 |= 0x40000000; // When ADONS = 1, I start conversion (SWSTART = 1)
        while ((ADC1->SR&0x0040)==0); // Wait till conversion is finished
        ADC_value = (int)ADC1->DR; //store the value
        mode = game2_mode(ADC_value);
        BSP_LCD_GLASS_Clear();
        BSP_LCD_GLASS_DisplayString((uint8_t *)" GAME2");
        for(int j=0;j<10;j++){
          HAL_UART_Transmit(&huart3, &(g2[j]), 1, 10000);
        }
        espera(2000000);
        game2(mode);
        break;
      }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LCD;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_4;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

  if(input[1]==88||input[1]==120){
    change = true;
  }
  if((input[1]==49||input[1]==50)){
    if(!listen){
      input[1] = 32;
    }
  }
  HAL_UART_Receive_IT(huart, &(input[1]), 1); // Vuelve a activar Rx por haber acabado el buffer
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

