#include "HardwareSerial.h"

/**
  * @brief  串口对象构造函数
  * @param  串口外设地址
  * @retval 无
  */
HardwareSerial::HardwareSerial(USART_TypeDef *_USARTx)
{
    this->USARTx = _USARTx;
    USART_Function = 0;
}

/**
  * @brief  串口中断入口
  * @param  无
  * @retval 无
  */
void HardwareSerial::IRQHandler()
{
    if(LL_USART_IsActiveFlag_RXNE(USARTx) && LL_USART_IsEnabledIT_RXNE(USARTx))
    {
        uint8_t c = LL_USART_ReceiveData8(USARTx);
        uint16_t i = (uint16_t)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
        if (i != _rx_buffer_tail)
        {
            _rx_buffer[_rx_buffer_head] = c;
            _rx_buffer_head = i;
        }

        if(USART_Function)
        {
            USART_Function();
        }

//        USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
    }
}

/**
  * @brief  串口初始化
  * @param  BaudRate: 波特率
  * @retval 无
  */
void HardwareSerial::begin(uint32_t BaudRate)
{
    begin(BaudRate, SERIAL_Config_Default);
}

/**
  * @brief  串口初始化
  * @param  BaudRate: 波特率
  * @param  Config: 配置参数
  * @retval 无
  */
void HardwareSerial::begin(uint32_t BaudRate, SERIAL_Config Config)
{
    begin(BaudRate, Config, USART_ChannelPriority_Default);
}

/**
  * @brief  串口初始化
  * @param  BaudRate: 波特率
  * @param  Config: 配置参数
  * @param  ChannelPriority: 通道优先级
  * @retval 无
  */
void HardwareSerial::begin(uint32_t BaudRate, SERIAL_Config Config, uint8_t ChannelPriority)
{
    //GPIO端口设置
    LL_GPIO_InitTypeDef GPIO_InitStructure;
    LL_USART_InitTypeDef USART_InitStructure;
    uint16_t Tx_Pin, Rx_Pin;
    IRQn_Type ItChannel;
    GPIO_TypeDef *GPIOx;

    if(USARTx == USART1)
    {
        Tx_Pin = LL_GPIO_PIN_9;
        Rx_Pin = LL_GPIO_PIN_10;

        GPIOx = GPIOA;

        /* Peripheral clock enable */
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
        ItChannel = USART1_IRQn;
    }
    else if(USARTx == USART2)
    {
        Tx_Pin = LL_GPIO_PIN_2;
        Rx_Pin = LL_GPIO_PIN_3;

        GPIOx = GPIOA;

        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
        ItChannel = USART2_IRQn;
    }

  //    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOx, ENABLE);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    GPIO_InitStructure.Pin = Tx_Pin | Rx_Pin;
    GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStructure.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOx, &GPIO_InitStructure);
    
    /* USART1 interrupt Init */
    NVIC_SetPriority(ItChannel, ChannelPriority);
    NVIC_EnableIRQ(ItChannel);
    //USART 初始化设置
    USART_InitStructure.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStructure.BaudRate = BaudRate;
    USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
    USART_InitStructure.Parity = LL_USART_PARITY_NONE;
    USART_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USARTx, &USART_InitStructure);
    LL_USART_ConfigAsyncMode(USARTx);

    LL_USART_EnableIT_RXNE(USARTx);
    LL_USART_Enable(USARTx);

}

/**
  * @brief  关闭串口
  * @param  无
  * @retval 无
  */
void HardwareSerial::end(void)
{
	  LL_USART_Disable(USARTx);

}

/**
  * @brief  串口中断回调
  * @param  Function: 回调函数
  * @retval 无
  */
void HardwareSerial::attachInterrupt(USART_CallbackFunction_t Function)
{
    USART_Function = Function;
}

/**
  * @brief  获取可从串行端口读取的字节数
  * @param  无
  * @retval 可读取的字节数
  */
int HardwareSerial::available(void)
{
    return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
}

/**
  * @brief  读取传入的串行数据(字符)
  * @param  无
  * @retval 可用的传入串行数据的第一个字节 (如果没有可用的数据, 则为-1)
  */
int HardwareSerial::read(void)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer_head == _rx_buffer_tail)
    {
        return -1;
    }
    else
    {
        unsigned char c = _rx_buffer[_rx_buffer_tail];
        _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
        return c;
    }
}

/**
  * @brief  返回传入串行数据的下一个字节(字符), 而不将其从内部串行缓冲区中删除
  * @param  无
  * @retval 可用的传入串行数据的第一个字节 (如果没有可用的数据, 则为-1)
  */
int HardwareSerial::peek(void)
{
    if (_rx_buffer_head == _rx_buffer_tail)
    {
        return -1;
    }
    else
    {
        return _rx_buffer[_rx_buffer_tail];
    }
}

/**
  * @brief  清空串口缓存
  * @param  无
  * @retval 无
  */
void HardwareSerial::flush(void)
{
    while(read() >= 0);
}

/**
  * @brief  串口写入一个字节
  * @param  写入的字节
  * @retval 字节
  */
size_t HardwareSerial::write(uint8_t n)
{
	 while(!LL_USART_IsActiveFlag_TC(USARTx)){};
	 LL_USART_TransmitData8(USARTx, n);
   return 1;
}

//Creat Object For User
HardwareSerial Serial(USART1);//TX-PA9 RX-PA10
HardwareSerial Serial2(USART2);//TX-PA2 RX-PA3

//USARTx_IRQHandler
extern "C" {
    void USART1_IRQHandler(void)
    {
        Serial.IRQHandler();
    }

    void USART2_IRQHandler(void)
    {
        Serial2.IRQHandler();
    }
}
