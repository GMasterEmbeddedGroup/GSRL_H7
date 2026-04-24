#include "dvc_buzzer.hpp"

TuneList::TuneList(TuneItem *items, uint16_t itemNum, bool loop)
    : m_loop(loop), m_tuneItems(items), m_tuneItemNum(itemNum)
{
}

TuneItem *TuneList::tickCurrentItem()
{
    if (m_tuneItems == nullptr || m_tuneItemNum == 0 || m_finished) {
        return nullptr;
    }

    TuneItem *currentTuneItem  = &m_tuneItems[m_currentItemIndex];
    uint16_t durationTickLimit = currentTuneItem->durationIntervalNum;
    if (durationTickLimit == 0U) {
        durationTickLimit = 1U;
    }

    if (++m_currentItemDurationCount >= durationTickLimit) {
        m_currentItemDurationCount = 0U;
        if (m_currentItemIndex + 1U >= m_tuneItemNum) {
            if (m_loop) {
                m_currentItemIndex = 0U;
            } else {
                m_finished = true;
            }
        } else {
            ++m_currentItemIndex;
        }
    }

    return currentTuneItem;
}

TuneList *TuneList::reset()
{
    m_currentItemIndex         = 0U;
    m_currentItemDurationCount = 0U;
    m_finished                 = false;
    return this;
}

Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timerClockHz, bool silentAll)
    : m_htim(htim), m_channel(channel), m_timerClockHz(timerClockHz), m_silentAll(silentAll)
{
}

void Buzzer::loadTuneList(TuneList *tuneList)
{
    if (m_currentTuneList != nullptr) {
        m_currentTuneList->reset();
    }
    if (tuneList != nullptr) {
        tuneList->reset();
    }
    m_currentTuneList = tuneList;
}

void Buzzer::playSchedule()
{
    if (m_silentAll) return;
    if (m_currentTuneList == nullptr) {
        stop();
        return;
    }

    playTuneItem(m_currentTuneList->tickCurrentItem());
}

void Buzzer::setFrequency(uint32_t freqHz, float duty)
{
    m_duty = clampDuty(duty);
    if (freqHz == 0U) {
        stop();
        return;
    }

    if (m_started && m_requestedHz == freqHz) {
        applyPwm(freqHz);
        return;
    }

    m_requestedHz = freqHz;
    applyPwm(freqHz);
    start();
}

float Buzzer::clampDuty(float duty)
{
    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 1.0f) {
        return 1.0f;
    }
    return duty;
}

uint32_t Buzzer::getTimerCounterClockHz() const
{
    if (m_htim == nullptr) {
        return 0U;
    }
    return m_timerClockHz / (m_htim->Init.Prescaler + 1U);
}

void Buzzer::applyPwm(uint32_t freqHz)
{
    if (m_htim == nullptr || freqHz == 0U) {
        return;
    }

    const uint32_t timerCounterClockHz = getTimerCounterClockHz();
    if (timerCounterClockHz == 0U) {
        return;
    }

    uint32_t periodCounts = timerCounterClockHz / freqHz;
    if (periodCounts < 2U) {
        periodCounts = 2U;
    }
    if (periodCounts > 65536U) {
        periodCounts = 65536U;
    }

    const uint32_t arr = periodCounts - 1U;
    uint32_t ccr       = static_cast<uint32_t>(periodCounts * m_duty);
    if (ccr > (arr + 1U)) {
        ccr = arr + 1U;
    }

    __HAL_TIM_SET_AUTORELOAD(m_htim, arr);
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, ccr);

    m_frequencyHz = timerCounterClockHz / periodCounts;
}

void Buzzer::playTuneItem(const TuneItem *tuneItem)
{
    if (tuneItem == nullptr || tuneItem->muted || tuneItem->tuneHz == TuneHz::REST) {
        stop();
        return;
    }

    const uint32_t targetHz = static_cast<uint32_t>(tuneItem->tuneHz);
    if (m_started && m_requestedHz == targetHz) {
        return;
    }

    setFrequency(targetHz, m_duty);
}

void Buzzer::start()
{
    if (m_htim == nullptr || m_started || m_frequencyHz == 0U) {
        return;
    }

    HAL_TIM_PWM_Start(m_htim, m_channel);
    m_started = true;
}

void Buzzer::stop()
{
    if (m_htim == nullptr || !m_started) {
        m_requestedHz = 0U;
        m_frequencyHz = 0U;
        return;
    }

    __HAL_TIM_SET_COMPARE(m_htim, m_channel, 0U);
    HAL_TIM_PWM_Stop(m_htim, m_channel);
    m_requestedHz = 0U;
    m_frequencyHz = 0U;
    m_started     = false;
}