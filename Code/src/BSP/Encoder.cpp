// #include "Encoder.hpp"

// Encoder::Encoder(TIM_HandleTypeDef* handle, bool flip)
//  : m_handle(handle), m_flip(flip)
// {
//     m_cnt = 0;
//     m_diff = 0;
// }

// void Encoder::init()
// {
//     HAL_TIM_Encoder_Start(m_handle,  TIM_CHANNEL_1 | TIM_CHANNEL_2);
//     (m_handle->Instance)->EGR=TIM_EGR_UG;
//     (m_handle->Instance)->CR1=TIM_CR1_CEN;
// }

// int32_t Encoder::update()
// {
//     int32_t tmp = (m_handle->Instance)->CNT;
//     m_diff = tmp - m_cnt;
//     m_cnt = tmp;
//     return m_cnt;
// }

// int32_t Encoder::getCount()
// {
//     return m_flip ? -1*m_cnt : m_cnt;
// }

// int32_t Encoder::getSpeed()
// {
//     return m_flip ? -1*m_diff : m_diff;
// }

// void Encoder::reset()
// {
//     (m_handle->Instance)->CNT = 0;
//     m_cnt = 0;
//     m_diff = 0;
// }