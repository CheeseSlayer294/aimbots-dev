#include "player.hpp"

#include <cstdint>

#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

namespace utils::Music {

// built around A = 440 Hz
static constexpr uint32_t NOTE_E4 = 330;
static constexpr uint32_t NOTE_F4 = 340;
static constexpr uint32_t NOTE_Gb4 = 370;
static constexpr uint32_t NOTE_G4 = 392;
static constexpr uint32_t NOTE_Ab4 = 415;
static constexpr uint32_t NOTE_A4 = 440;
static constexpr uint32_t NOTE_Bb4 = 469;
static constexpr uint32_t NOTE_B4 = 494;
static constexpr uint32_t NOTE_C5 = 523;
static constexpr uint32_t NOTE_Db5 = 554;
static constexpr uint32_t NOTE_D5 = 588;
static constexpr uint32_t NOTE_Eb5 = 622;
static constexpr uint32_t NOTE_E5 = 659;
static constexpr uint32_t NOTE_F5 = 699;
static constexpr uint32_t NOTE_Gb5 = 740;
static constexpr uint32_t NOTE_G5 = 784;
static constexpr uint32_t NOTE_Ab5 = 830;
static constexpr uint32_t NOTE_A5 = 880;
static constexpr uint32_t NOTE_Bb5 = 932;
static constexpr uint32_t NOTE_B5 = 988;
static constexpr uint32_t NOTE_C6 = 1047;
static constexpr uint32_t NOTE_Db6 = 1108;
static constexpr uint32_t NOTE_D6 = 1174;
static constexpr uint32_t NOTE_Eb6 = 1244;
static constexpr uint32_t NOTE_E6 = 1318;
static constexpr uint32_t NOTE_F6 = 1396;

// Windows XP Startup BPM
static constexpr uint32_t XP_BPM = 110;
static constexpr uint32_t XP_MS_PER_16th = (uint32_t)(((1.0f / XP_BPM) * 60.0f * 1000.0f) / 4.0f);

struct MusicNote {
    uint32_t frequency;
};

static uint32_t lastXPTime = 0;
static uint32_t currentXPNote = 0;
static uint32_t lastXPFreq = 0;
static bool xpTuneFinished = false;

static MusicNote xpStartupNotes[16] = {
    {NOTE_Eb6},{NOTE_Eb6},{NOTE_Eb6},{NOTE_Eb5},
    {NOTE_Bb5},{0},{NOTE_Ab5},{NOTE_Ab5},
    {NOTE_Eb5},{0},{NOTE_Eb6},{NOTE_Eb6},
    {NOTE_Bb5},{NOTE_Bb5},{0},{0}
    };

static constexpr size_t XP_NOTE_COUNT = sizeof(xpStartupNotes) / sizeof(MusicNote);

void continuePlayingXPStartupTune(src::Drivers* drivers) {
    if (xpTuneFinished) return;
    if (lastXPTime == 0) lastXPTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastXPTime;

    if (timeSinceLast >= XP_MS_PER_16th) {
        lastXPTime = tap::arch::clock::getTimeMilliseconds();
        if (lastXPFreq != xpStartupNotes[currentXPNote].frequency) tap::buzzer::playNote(&drivers->pwm, xpStartupNotes[currentXPNote].frequency);
        lastXPFreq = xpStartupNotes[currentXPNote].frequency;
        currentXPNote++;
        xpTuneFinished = currentXPNote == XP_NOTE_COUNT;
    }
}

// Tokyo Drift Theme

static constexpr uint32_t TD_BPM = 120;
static constexpr uint32_t TD_MS_PER_16th = (uint32_t)(((1.0f / TD_BPM) * 60.0f * 1000.0f) / 4.0f);

static uint32_t lastTDTime = 0;
static uint32_t currentTDNote = 0;
static uint32_t lastTDFreq = 0;

static MusicNote tokyoDriftNotes[16] = {
    {NOTE_Bb4},{NOTE_Bb4},{NOTE_Bb4},{NOTE_B4},
    {NOTE_B4},{NOTE_B4},{NOTE_Eb5},{NOTE_Eb5},
    {NOTE_Bb5},{0},{0},{0},
    {NOTE_Bb5},{0},{0},{0}
    };

static constexpr size_t TD_NOTE_COUNT = sizeof(tokyoDriftNotes) / sizeof(MusicNote);

void continuePlayingTokyoDriftTune(src::Drivers* drivers) {
    if (lastTDTime == 0) lastTDTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastTDTime;

    if (timeSinceLast >= TD_MS_PER_16th) {
        lastTDTime = tap::arch::clock::getTimeMilliseconds();
        if (lastTDFreq != tokyoDriftNotes[currentTDNote].frequency) tap::buzzer::playNote(&drivers->pwm, tokyoDriftNotes[currentTDNote].frequency);
        lastTDFreq = tokyoDriftNotes[currentTDNote].frequency;
        currentTDNote = (currentTDNote + 1) % TD_NOTE_COUNT;
    }
}

// PacMan Theme

static bool isPacManDone = false;

static constexpr uint32_t PM_BPM = 130;
static constexpr uint32_t PM_MS_PER_16th = (uint32_t)(((1.0f / PM_BPM) * 60.0f * 1000.0f) / 4.0f);

static uint32_t lastPMTime = 0;
static uint32_t currentPMNote = 0;
static uint32_t lastPMFreq = 0;

static MusicNote pacManNotes[] = {{NOTE_B4},  {NOTE_B5}, {NOTE_Gb5}, {NOTE_Eb5},

                                  {NOTE_B5}, {0},       {NOTE_Eb5}, {0},

                                  {NOTE_C5},  {NOTE_C6}, {NOTE_G5},  {NOTE_E5},

                                  {NOTE_C6}, {0},       {NOTE_E5},  {0},

                                  {NOTE_B4},  {NOTE_B5}, {NOTE_G5}, {NOTE_E5},

                                  {NOTE_B5}, {0},       {NOTE_Eb5}, {0},

                                  {NOTE_Eb5}, {NOTE_Eb5}, {NOTE_F5},  {NOTE_F5},

                                  {NOTE_G5},  {NOTE_G5},  {NOTE_B5}, {0}};

static constexpr size_t PM_NOTE_COUNT = sizeof(pacManNotes) / sizeof(MusicNote);

void playPacMan(src::Drivers* drivers) {
    if (isPacManDone) return;
    if (lastPMTime == 0) lastPMTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastPMTime;

    if (timeSinceLast >= PM_MS_PER_16th) {
        lastPMTime = tap::arch::clock::getTimeMilliseconds();
        if (lastPMFreq != pacManNotes[currentPMNote].frequency) tap::buzzer::playNote(&drivers->pwm, pacManNotes[currentPMNote].frequency);
        lastPMFreq = pacManNotes[currentPMNote].frequency;
        currentPMNote++;
        isPacManDone = currentPMNote == PM_NOTE_COUNT;
    }
}

// Chainsaw Man theme

static bool isChainSawDone = false;

static constexpr uint32_t CHNSW_BPM = 350;
static constexpr uint32_t CHNSW_MS_PER_8th = (uint32_t)(((1.0f / CHNSW_BPM) * 60.0f * 1000.0f) / 2.0f); // halves the overall size of the note array

static uint32_t lastCHNSWTime = 0;
static uint32_t currentCHNSWNote = 0;
static uint32_t lastCHNSWFreq = 0;

static MusicNote chainSawNotes[72] = {
                    {NOTE_Ab5},{NOTE_Ab5},
                    {NOTE_Db5},{NOTE_Db5},{NOTE_Eb5},{NOTE_Eb5},{NOTE_E5},{NOTE_Gb5},{NOTE_Gb5},{NOTE_A4},
                    {NOTE_A4},{NOTE_E5},{NOTE_E5},{NOTE_E5},{NOTE_A4},{NOTE_A4},{NOTE_A4},{NOTE_A4},
                    {NOTE_Ab4},{NOTE_Ab4},{NOTE_Eb5},{NOTE_Eb5},{NOTE_Ab4},{NOTE_Gb4},{NOTE_Gb4},{NOTE_Ab4},
                    {NOTE_Ab4},{NOTE_Db5},{NOTE_Db5},{NOTE_Db5},{0},{0},{NOTE_Ab4},{NOTE_Ab4},
                    {NOTE_Gb4},{NOTE_E4},{NOTE_Gb4},{NOTE_Gb4},{NOTE_Ab4},{NOTE_Ab4},{NOTE_Bb4},
                    {NOTE_B4},{NOTE_B4},{NOTE_A4},{NOTE_Ab4},{NOTE_Gb4},{NOTE_Gb4},{NOTE_Db5},{NOTE_Db5},
                    {NOTE_Db5},{NOTE_Db5},{NOTE_B4},{NOTE_A4},{NOTE_Ab4},{NOTE_E5},{NOTE_E5},{NOTE_Eb5},
                    {NOTE_Eb5},{NOTE_Eb5},{NOTE_Eb5},{NOTE_Eb5},{0},{0},{NOTE_Ab5},{NOTE_Ab5},
                    {NOTE_Db5},{NOTE_Db5},{NOTE_Db5},{NOTE_Db5}

};

static constexpr size_t CHNSW_NOTE_COUNT = sizeof(chainSawNotes) / sizeof(MusicNote);

void playChainSawMan(src::Drivers* drivers) {
    if (isChainSawDone) return;
    if (lastCHNSWTime == 0) lastCHNSWTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastCHNSWTime;

    if (timeSinceLast >= CHNSW_MS_PER_8th) {
        lastCHNSWTime = tap::arch::clock::getTimeMilliseconds();
        if (lastCHNSWFreq != chainSawNotes[currentCHNSWNote].frequency) tap::buzzer::playNote(&drivers->pwm, chainSawNotes[currentCHNSWNote].frequency);
        lastCHNSWFreq = chainSawNotes[currentCHNSWNote].frequency;
        currentCHNSWNote++;
        isChainSawDone = currentCHNSWNote == CHNSW_NOTE_COUNT;
    }
}

}  // namespace utils::Music