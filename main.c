#include <stm32f10x.h>
void configure_adc_in8(){
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // validation horloge ADC1
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // passage de l'horloge ADC1 à 12MHz
    ADC1->CR2|= ADC_CR2_ADON; // démarrage ADC1
    ADC1->SQR1&= ADC_SQR1_L; // fixe le nombre de conversion à 1
    ADC1->SQR3|= 8; // indique la voie à convertir
    ADC1->CR2 |= ADC_CR2_CAL; // dÈbut de la calibration
    while ((ADC1->CR2 & ADC_CR2_CAL)); // attente de la fin de la calibration
}
int convert_single(){
    ADC1->CR2 |= ADC_CR2_ADON; // lancement de la conversion
    while(!(ADC1->SR & ADC_SR_EOC) ) {} // attente de la fin de conversion
    ADC1->SR &= ~ADC_SR_EOC; // validation de la conversion
    return ADC1->DR & ~((0x0F) << 12); // retour de la conversion
}
void configure_gpio_pb0_analog_input(){
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRL &= ~((0x1 << 0) | (0x1 << 1) | (0x1 << 2) | (0x1 << 3));
}
void configure_gpio_pa6_alternate_push_pull(){
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~(0xF << 24);
    GPIOA->CRL |= (0xA << 24);
}

void configure_pwm_ch1_20khz(TIM_TypeDef *TIMER){
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIMER->PSC = 0; 
    TIMER->ARR = 0xE0F;
    TIMER->CCMR1 &= ~TIM_CCMR1_OC1M_0;
    TIMER->CCMR1 |= TIM_CCMR1_OC1M_1| TIM_CCMR1_OC1M_2;
    TIMER->CCER |= TIM_CCER_CC1E;
}

void start_timer(TIM_TypeDef *TIMER){
    TIMER->CR1 |= TIM_CR1_CEN;
}

void set_pulse_percentage(TIM_TypeDef *TIMER, int pulse){
    TIMER->CCR1 = TIMER->ARR*pulse/100;
}
int main(void)
{

    int res = 0;
    // Configuration de la PWM
    configure_gpio_pa6_alternate_push_pull (); 
    configure_pwm_ch1_20khz(TIM3); 
    set_pulse_percentage(TIM3, 0); 
    // Configuration de l'ADC
    configure_gpio_pb0_analog_input();
    configure_adc_in8();
    // Démarrage de la PWM
    start_timer(TIM3);
    while(1)
	{
        res = convert_single(); // conversion
        set_pulse_percentage(TIM3, 100 * res / 0xFFF ); // mise à jour de l’intensité de la led
	}
    return 0;



}