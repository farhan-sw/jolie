#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H

#include <Arduino.h>
#include <queue>

class MovingAverage {
private:
    std::queue<float> window;   // Queue to store recent values
    size_t maxSize;             // Maximum size of the moving average window
    float sum;                  // Sum of the values in the window

public:
    // Constructor
    MovingAverage(size_t _maxSize);

    // Add a new value and get the smoothed average
    float addValue(float newValue);

    // Reset the moving average
    void reset();

    // Get the current moving average
    float getAverage() const;

    // Check if the moving average has enough values to be valid
    bool isReady() const;
};

#endif // MOVINGAVERAGE_H
