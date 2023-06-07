#include "../system/include/cmsis/stm32f4xx.h"

#define CPU_CLOCK SystemCoreClock
#define MY_BDR 115200

#define MYBRR (CPU_CLOCK / (16 * MY_BDR))

void GPIO_init() {
    // Вкл. тактирование
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // назначили пины
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    // Альтернативные функции
    GPIOA->AFR[0] |= (0x77 << 8);
    GPIOA->AFR[1] |= (0x77 << 4);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0);
}

void USART_init() {
    //// включаем USART2 PA2, PA3
    // Вкл тактирование
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // Задали частоту работы
    USART2->BRR = MYBRR;
    // Настроили на чтение и запись
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    // Установили STOP бит
    USART2->CR2 |= (USART_CR2_STOP_1);
    // Включили DMA для USART
    USART2->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    // Вкл USART
    USART2->CR1 |= USART_CR1_UE;

    //// Включаем USART1 PA9, PA10
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = MYBRR;
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
    USART1->CR2 |= (USART_CR2_STOP_1);
    USART1->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    USART1->CR1 |= USART_CR1_UE;
}

#define SIZE 128

uint8_t usart1_receive_buffer[(2 * SIZE)];
uint8_t usart2_receive_buffer[(2 * SIZE)];
uint8_t mrk1 = 0x01, mrk2 = 0x01;  // маркеры окнчания и начала передачи данных

void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {
        GPIOD->ODR ^= GPIO_ODR_OD12;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
//      mrk1 =0x1;
    }
}

void DMA2_Stream2_IRQHandler(void) {
    if ((DMA2->LISR & DMA_LISR_TCIF2) == DMA_LISR_TCIF2) {
        GPIOD->ODR ^= GPIO_ODR_OD13;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
   //     mrk2 = 0x01;
    }
}

void DMA_init() {
    // Включили тактирование DMA
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Выбираем канал
    DMA1_Stream6->CR |= (0x4 << 25);  // Канал 4
    DMA2_Stream2->CR |= (0x4 << 25);  // Канал 4
    
    // Устанавливаем размер ячейки (8-бит(можно 16-бит, 32-бит))
    DMA1_Stream6->CR &= ~DMA_SxCR_MSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_MSIZE;

    // Установите размер периферийных данных равным 8 битам
    DMA1_Stream6->CR &= ~DMA_SxCR_PSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_PSIZE;

    // Установите режим увеличения объема памяти
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
    DMA2_Stream2->CR |= DMA_SxCR_MINC;

    // Включаем режим работы
    DMA1_Stream6->CR |= (0x01<<6);  // Из памяти в перефирию
    DMA2_Stream2->CR &= ~(3UL<<6);  // Из переферии в память

    // Включить циклическую запись
    DMA1_Stream6->CR |= DMA_SxCR_CIRC;
    DMA2_Stream2->CR |= DMA_SxCR_CIRC;

    //  Включаем прерывания
    DMA1_Stream6->CR |= DMA_SxCR_TCIE;
    DMA2_Stream2->CR |= DMA_SxCR_TCIE;

    // Количество элементов данных, подлежащих передаче	
    DMA1_Stream6->NDTR = SIZE;
    DMA2_Stream2->NDTR = SIZE;

    // Задаем адрес переферии
    DMA1_Stream6->PAR = (uint32_t)(&USART2->DR);
    DMA2_Stream2->PAR = (uint32_t)(&USART1->DR);

    // Задаем адрес памяти
    DMA1_Stream6->M0AR = (uint32_t)&usart2_receive_buffer[0];
    DMA2_Stream2->M0AR = (uint32_t)&usart1_receive_buffer[0];

    // Включаем DMA
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;

    // Настройка прерываний
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 5);

    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_SetPriority(DMA2_Stream2_IRQn, 4);
}

void write_dma(uint8_t *df, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *(usart2_receive_buffer + i) = *(df + i);
    }

    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    DMA1_Stream6->NDTR |= size;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
 //   while (!mrk1) {}
 //   mrk1 = 0x0;
}

void read_dma(uint8_t *df, uint16_t size) {
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    DMA2_Stream2->NDTR |= size;
    DMA2_Stream2->CR |= DMA_SxCR_EN;

  //  while (!mrk2) {
        //GPIOD->ODR ^= GPIO_ODR_OD13;
  //  }
  //  mrk2 = 0x0; 
    for (uint16_t i = 0; i < size; i++) {
        *(df + i) = *(usart1_receive_buffer + i);
    }
}

int main(void) {
    SystemCoreClockUpdate();
    GPIO_init();
    USART_init();
    DMA_init();
    // инициализируем массив который будет отдан на запись
    uint8_t temp[(2 * SIZE)];
    for (int i = 0; i < (2 * SIZE); i++) {
        temp[i] = (i % 254);
    }
    uint8_t temp2[(2 * SIZE)];
    while(1) {
        write_dma(&temp[0], SIZE);
        for (int i = 0; i < 10000; i++);
        read_dma(&temp2[0], SIZE);
    }
}


