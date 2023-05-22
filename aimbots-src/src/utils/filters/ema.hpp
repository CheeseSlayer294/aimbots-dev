#pragma once

class EMAFilter {
private:
    float alpha;
    float lastValue;

public:
    EMAFilter(float alpha) {
        this->alpha = alpha;
        this->lastValue = 0;
    }

    float update(float value) {
        float filteredValue = alpha * value + (1 - alpha) * lastValue;
        lastValue = filteredValue;
        return filteredValue;
    }
};