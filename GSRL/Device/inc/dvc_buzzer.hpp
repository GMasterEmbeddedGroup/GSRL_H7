#pragma once

#include "tim.h"
#include <stddef.h>
#include <stdint.h>

// clang-format off
enum class TuneHz : uint32_t {
    REST = 0,

    // Octave 4
    C4  = 262, CS4 = 277, D4  = 294,
    DS4 = 311, E4  = 330, F4  = 349,
    FS4 = 370, G4  = 392, GS4 = 415,
    A4  = 440, AS4 = 466, B4  = 494,

    // Octave 5
    C5  = 523, CS5 = 554, D5  = 587,
    DS5 = 622, E5  = 659, F5  = 698,
    FS5 = 740, G5  = 784, GS5 = 831,
    A5  = 880, AS5 = 932, B5  = 988,

    // Octave 6
    C6  = 1047, CS6 = 1109, D6  = 1175,
    DS6 = 1245, E6  = 1319, F6  = 1397,
    FS6 = 1480, G6  = 1568, GS6 = 1661,
    A6  = 1760, AS6 = 1865, B6  = 1976,

    // Octave 7
    C7  = 2093, CS7 = 2217, D7  = 2349,
    DS7 = 2489, E7  = 2637, F7  = 2794,
    FS7 = 2960, G7  = 3136, GS7 = 3322,
    A7  = 3520, AS7 = 3729, B7  = 3951,

    // Octave 8
    C8  = 4186, CS8 = 4435, D8  = 4699,
    DS8 = 4978, E8  = 5274, F8  = 5588,
    FS8 = 5920, G8  = 6272, GS8 = 6645,
    A8  = 7040, AS8 = 7459, B8  = 7902,
};
// clang-format on

struct TuneItem {
    uint16_t durationIntervalNum;
    TuneHz tuneHz;
    bool muted;
};

class TuneList
{
public:
    TuneList(TuneItem *items, uint16_t itemNum, bool loop = false);

    template <size_t N>
    explicit TuneList(TuneItem (&items)[N], bool loop = false)
        : TuneList(items, static_cast<uint16_t>(N), loop)
    {
    }

    TuneItem *tickCurrentItem();
    TuneList *reset();

private:
    bool m_loop                         = false;   // 是否循环
    TuneItem *m_tuneItems               = nullptr; // the Items to be loaded into buzzer class
    uint16_t m_tuneItemNum              = 0;       // the num of all tune items
    uint16_t m_currentItemIndex         = 0;       // inside variable, current tune index
    uint16_t m_currentItemDurationCount = 0;       // inside counter of ticks
    bool m_finished                     = false;   // finsih flag
};

class Buzzer
{
public:
    explicit Buzzer(TIM_HandleTypeDef *htim = &htim12,
                    uint32_t channel        = TIM_CHANNEL_2,
                    uint32_t timerClockHz = 84000000U, bool silentAll = false);

    void loadTuneList(TuneList *tuneList);
    void playSchedule();
    void setFrequency(uint32_t freqHz, float duty = 0.5f);

private:
    static float clampDuty(float duty);
    uint32_t getTimerCounterClockHz() const;
    void applyPwm(uint32_t freqHz);
    void playTuneItem(const TuneItem *tuneItem);
    void start();
    void stop();

    TIM_HandleTypeDef *m_htim   = nullptr;
    uint32_t m_channel          = 0U;
    uint32_t m_timerClockHz     = 0U;
    bool m_silentAll            = false; // 是否不发声音
    uint32_t m_requestedHz      = 0U;
    uint32_t m_frequencyHz      = 0U;
    float m_duty                = 0.5f;
    bool m_started              = false;
    TuneList *m_currentTuneList = nullptr;
};