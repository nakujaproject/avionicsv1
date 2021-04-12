// We have three scenarios:
// 1. When we start decreasing altitude
// 2. When we reach 0 vertical velocity
// 3. When leaves - 9.8 

// With the acceleration close to 0g begin watching the altitude from the barometer. When it decreases, we’re at apogee. Also start a countdown so that if a decreasing altitude is never detected, an apogee event will still occur.
// With the acceleration somewhat close to 0g, start a safety countdown so that if the lower acceleration threshold is never hit, an apogee event will still occur.
// As a final fail safe: If the acceleration is greater than 1g again and we’re looking for apogee, fire the apogee event because we failed to detect apogee entirely.
float liftoffAltitude, prevAltitude, apogeeAltitude;
int measures = 5;
bool isApogee = false;

void detectApogee1(float altitude) {
     //detect apogee
    if (altitude > liftoffAltitude) {
        if (altitude < prevAltitude ) {
            measures -= 1;
            if (measures == 0) {
                apogeeAltitude = altitude;
                isApogee = true;
            }
            else {
                prevAltitude = altitude;    
            }
        }
    }
}

void detectApogee2(float velocity){
    if (velocity < 0){
        isApogee = true;
    }
}

void detectApogee3(float acceleration){
    bool isUnder = false;
    if (acceleration < 0){
        isUnder = true;
    }
    if (acceleration > 0) && (isUnder == true) {
        isApogee = true;
    }
}
