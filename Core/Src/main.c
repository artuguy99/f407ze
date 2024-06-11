#include "main.h"
#include <stdio.h>
/* RAM */
#define RAM_START         (0x20000000u)
#define RAM_SIZE          (128 * 1024) // 128 KB
/* Stacks */
#define MAIN_STACK        (RAM_START + RAM_SIZE)
#define TASK_NUMBER_MAX   (16)
#define TASK_STACK_SIZE   (1024u)

uint32_t __uCurrentTaskIdx = 0;
uint32_t __puTasksPSP[TASK_NUMBER_MAX] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint32_t get_current_psp() {
  return __puTasksPSP[__uCurrentTaskIdx];
}

void save_current_psp(uint32_t psp) {
  __puTasksPSP[__uCurrentTaskIdx] = psp;
}

void select_next_task() {
  /* Round-Robin scheduler */
  __uCurrentTaskIdx++;
  // check if a task is register at current slot
  if (__uCurrentTaskIdx >= TASK_NUMBER_MAX || __puTasksPSP[__uCurrentTaskIdx] == 0) {
    __uCurrentTaskIdx=0;
  }
}

void start_scheduler() {
  printf("Start Scheduler!\n\r");

  // start with the first task
  __uCurrentTaskIdx = 0;
  // Step 3: 
  // prepare PSP of the first task
  __asm volatile("BL get_current_psp"); // return PSP in R0
  __asm volatile("MSR PSP, R0");  // set PSP  <- R0, PSP is R13

  // change to use PSP
  __asm volatile("MRS R0, CONTROL");  // set R0  <- CONTROL, 
  __asm volatile("ORR R0, R0, #2"); // set bit[1] SPSEL, R0 |= 0X02;
  __asm volatile("MSR CONTROL, R0");  //  set CONTROL  <- R0

  // Step 4: Set sysTick interrupt
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
  LL_SYSTICK_EnableIT();

  // Step 5: optional
  // Move to Unprivileged level
  __asm volatile("MRS R0, CONTROL");
  __asm volatile("ORR R0, R0, #1"); // Set bit[0] nPRIV
  __asm volatile("MSR CONTROL, R0");
  // right after here, access is limited

  // Step 6: Run the 1st task
  // get the handler of the first task by tracing back from PSP which is at R4 slot
  void (*handler)() = (void (*))((uint32_t*)__puTasksPSP[__uCurrentTaskIdx])[14];

  // execute the handler
  handler();
}


void init_task(void (*handler)) {
  int i=0;
  // find an empty slot
  for(; i<TASK_NUMBER_MAX; i++) {
    if (__puTasksPSP[i] == 0) break;
  }

  if(i >= TASK_NUMBER_MAX) {
    printf("Can not register a new task anymore!\n");
    return;
  } else {
    printf("Register a task %p at slot %i\n\r", handler, i);
  }

  // calculate new PSP
  uint32_t* psp = (uint32_t*)(MAIN_STACK - (i+1)*TASK_STACK_SIZE);

  // fill dummy stack frame
  *(--psp) = 0x01000000u; // Dummy xPSR, just enable Thumb State bit;
  *(--psp) = (uint32_t) handler; // PC
  *(--psp) = 0xFFFFFFFDu; // LR with EXC_RETURN to return to Thread using PSP
  *(--psp) = 0x12121212u; // Dummy R12
  *(--psp) = 0x03030303u; // Dummy R3
  *(--psp) = 0x02020202u; // Dummy R2
  *(--psp) = 0x01010101u; // Dummy R1
  *(--psp) = 0x00000000u; // Dummy R0
  *(--psp) = 0x11111111u; // Dummy R11
  *(--psp) = 0x10101010u; // Dummy R10
  *(--psp) = 0x09090909u; // Dummy R9
  *(--psp) = 0x08080808u; // Dummy R8
  *(--psp) = 0x07070707u; // Dummy R7
  *(--psp) = 0x06060606u; // Dummy R6
  *(--psp) = 0x05050505u; // Dummy R5
  *(--psp) = 0x04040404u; // Dummy R4

  // save PSP
  __puTasksPSP[i] = (uint32_t)psp;
}

/* Tasks */
void task1_main(void) {
  while(1) {
    printf("1111\n\r");
  }
}

void task2_main(void) {
  while(1) {
    printf("2222\n\r");
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Reset of all peripherals, Initializes the Flash interface. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  
  /* USER CODE BEGIN 2 */
  init_task(task1_main);
  init_task(task2_main);

  start_scheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    LL_GPIO_TogglePin(GPIOF, LL_GPIO_PIN_8);
    LL_mDelay(200);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

__attribute__ ((naked)) void SysTick_Handler(void)
{
  	  /* Save the context of current task */
	  // save LR back to main, must do this firstly
	  __asm volatile("PUSH {LR}");
	  // get current PSP
	  __asm volatile("MRS R0, PSP");
	  // save R4 to R11 to PSP Frame Stack
	  __asm volatile("STMDB R0!, {R4-R11}"); // R0 is updated after decrement
	  // save current value of PSP
	  __asm volatile("BL save_current_psp"); // R0 is first argument

	  /* Do scheduling */

	  // select next task
	  __asm volatile("BL select_next_task");

	  /* Retrieve the context of next task */

	  // get its past PSP value
	  __asm volatile("BL get_current_psp"); // return PSP is in R0
	  // retrieve R4-R11 from PSP Fram Stack
	  __asm volatile("LDMIA R0!, {R4-R11}"); // R0 is updated after increment
	  // update PSP
	  __asm volatile("MSR PSP, R0");

	  // exit
    __asm volatile("POP {LR}");
	  __asm volatile("BX LR");
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  // LL_Init1msTick(168000000);
  // LL_SetSystemCoreClock(168000000);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ByteReceivedCallback(void)
{
    uint8_t ch;
    // read the data from the data reception register
    // by doing this, we also clear the interrupt flag
    ch = LL_USART_ReceiveData8(USART1);

    //waiting until the Transmit Empty flag is set
    while(!LL_USART_IsActiveFlag_TXE(USART1));

    LL_USART_TransmitData8(USART1, ch);

    //Wait until the transmit complete Flag to be raised
    while (!LL_USART_IsActiveFlag_TC(USART1));
}

/* USER CODE BEGIN 4 */
int _write (int fd, char * ptr, int len)
{
  int i;

  for (i = 0; i < len; i++) {
    while(!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, *ptr++);
    // SER_PutChar (*ptr++);
  }
  while (!LL_USART_IsActiveFlag_TC(USART1));
  return (i);
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
