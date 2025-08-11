#pragma once

// Initialize the camera peripheral with ESP32-S3 Eye pin mapping.
// Returns 1 on success, 0 on failure (after printing an error on Serial).
int cameraSetup(void);

// Initialize status LED, SD-MMC, and /camera folder, then run cameraSetup().
// Sets the WS2812 color to green on success, red on failure.
void setUpCamera(void);

// Capture one JPEG frame and save it to the given absolute path on SD.
// Example path: "/camera/123456.jpg".
// Returns true on success, false on failure.
bool saveJpegToSD(const char* path);
void captureImage(void);
