
#define  Yled_on    GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define  Yled_off   GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define  Bled_on    GPIO_ResetBits(GPIOB, GPIO_Pin_15)
#define  Bled_off   GPIO_SetBits(GPIOB, GPIO_Pin_15)

void LED_Config(void);
