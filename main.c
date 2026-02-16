#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include<math.h>
#include "stm32f4xx.h"

/*******************************************************************************************************
 *         ADC  Macros
 * ******************************************************************************************************
 */
#define ADC_RESOLUTION      4095.0f
#define VREF                3.3f
#define R_FIXED            10000.0f
#define NTC_BETA            3950.0f
#define T0_KELVIN          298.15f
#define NTC_R0            10000.0f
#define STEP_SIZE (VREF / ADC_RESOLUTION)
/***********************************************************************************************************************************
 *              ADC NTC_Status_t;
 * ********************************************************************************************************************************
 */
  typedef enum
{
    NTC_OK = 0,
    NTC_ERROR,
   NTC_ADC_BUSY
}NTC_Status_t;

/****************************************************************************************************
 *            ADC STRUCTURE
 * ********************************************************************************************************
 */typedef struct
 {
     float R0;
     float Beta;
     float T0;
     float Fixed_Resistor;
     float step_size;
 }NTC_Config_t;
 /************************************************************************************************************************
  *       ADC FUNCTION
  * ******************************************************************************************************************
  */

   static void ADC_init(void);
   static  void LED_init(void);
   static void TIM7_Init(void) ;
   static uint16_t ADC_Read(void);
   static float Get_Temperature(void);



/************************************************************************************************************
 *               ADC CONFIGURATION   ADC_init
 * ***********************************************************************************************************
 */
static void ADC_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    GPIOA->MODER |= (3 << (0 * 2));
    GPIOA->PUPDR &= ~(3 << (0 * 2));

    ADC1->SQR3 = 0;
    ADC1->SQR1 = 0;
    ADC1->SMPR2 |= (7 << (0 * 3));
    ADC1->CR2  |= ADC_CR2_ADON;
}
/************************************************************************************************************
 *               ADC CONFIGURATION    ADC_Read   function
 * ***********************************************************************************************************
 */

    static uint16_t ADC_Read(void)
    {

    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;

    }
    /************************************************************************************************************
     *               ADC Get_Temperature  function
     * ***********************************************************************************************************
     */
    float Get_Temperature(void)
    {
        uint16_t adc_val = ADC_Read();
        float v_ntc = adc_val * STEP_SIZE;
        if (v_ntc <= 0.001f || v_ntc >= (VREF - 0.001f))
             return -100.0f;
        float r_ntc = (R_FIXED * v_ntc) / (VREF - v_ntc);
        float tempK = 1.0f /( (1.0f / T0_KELVIN) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0) );
        return (tempK - 273.15f);
   }


    /*************************************************************************************************
     *      LED INIT FUNCTION
     * ************************************************************************************************
     */
        static  void LED_init(void)
        {
        	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
            GPIOA->MODER &= ~(0x3 << (5 * 2));
            GPIOA->MODER |=  (0x1 << (5 * 2));

        }
        /*************************************************************************************************
             *      TIM INIT FUNCTION
             * *************************************************************************************************/



        static void TIM7_Init(void)
    {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 16000 - 1;
    TIM7->ARR = 0;
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->EGR |= TIM_EGR_UG;
    NVIC_EnableIRQ(TIM7_IRQn);

}
        /*************************************************************************************************
         *      TIM ISR FUNCTION
        ************************************************************************************************
                     */

        void TIM7_IRQHandler(void)
               {
                   if (TIM7->SR & TIM_SR_UIF)
                   {
                       TIM7->SR &= ~TIM_SR_UIF;
                               GPIOA->ODR ^= (1 << 5);
                           }


               }




    /************************************************************************************************************
         *              MAIN  function
         * ***********************************************************************************************************
         */

    int main(void)
    {
    	ADC_init();
        LED_init();
        TIM7_Init();
        while (1)
        {
            float tempC = Get_Temperature();

            if(tempC >= 100.0f)
            {
            	TIM7->ARR = 1000-1;
                TIM7->CR1 |= TIM_CR1_CEN;


            }
            else if(tempC >= 80.0f)
            {
            	TIM7->ARR = 500-1;
                TIM7->CR1 |= TIM_CR1_CEN;


            }
            else
            {
                GPIOA->BSRR = (1 << (5 + 16));
                TIM7->ARR = 0;
                TIM7->CR1 &=~ TIM_CR1_CEN;
            }
        }



    }




































