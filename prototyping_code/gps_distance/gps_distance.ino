/*
 * gps_distance.ino
 *
 * Prototype program to read in GPS coordinates from a Sparkfun SAM-M8Q board
 * (https://www.sparkfun.com/products/15210) and computes the distance from
 * two readings using the Haversine Distance Formula.
 */

// Trig functions
#include <math.h>

// Needed for I2C to GPS
#include <Wire.h>

// Sparkfun u-blox GNSS Library
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <MicroNMEA.h>

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// Pushbutton pin
constexpr byte BUTTON_PIN = 2;

// GPS object
SFE_UBLOX_GNSS myGNSS;

/*
 * A nautical mile is defined as one minute of arc of a great circle of the
 * earth.  The SI value for the length of a nautical mile in kilometers is 
 * 1.852 km.  This means that the circumference of the earth is:
 *               60 minutes    1.852 kilometers
 * 360 degrees * ---------- * ------------------ = 40,003.2 km
 *                1 degree         1 minute
 *
 * Since the circumference = 2 * pi * r, Earth's radius is:
 * 40,003.2 = 2 * pi * r -> r = 6,366.707 kilometers
 */ 
constexpr double EARTH_RADIUS_M = 6366707.0;

/*
 * Computes the half versed sine (haversine) of the given angle.  Angle must
 * be given in RADIANS.  Output is in RADIANS.
 */
double Hav(const double theta) {
  const double halfsine = sin(theta / 2);
  return halfsine * halfsine;
}

/*
 * Computes the arc distance between 2 GPS coordinates.  Coordinates are given
 * in DEGREES.  Arc distance given in METERS.
 */
double ComputeArcDist(const double lat1, const double long1,
                      const double lat2, const double long2) {
  // Need to convert from degrees to radians for trig functions
  const double lat1_rads = lat1 * M_PI/180.0;
  const double lat2_rads = lat2 * M_PI/180.0;
  const double long1_rads = long1 * M_PI/180.0;
  const double long2_rads = long2 * M_PI/180.0;

  // Compute differences in latitude/longitude
  const double delta_lat_rads = lat2_rads - lat1_rads;
  const double delta_long_rads = long2_rads - long1_rads;

  const double a = Hav(delta_lat_rads) + (cos(lat1_rads) * cos(lat2_rads) *
                                          Hav(delta_long_rads));
  const double c = 2 * asin(min(1, sqrt(a)));
  return EARTH_RADIUS_M * c;
}

/*
 * Acquires a GPS coordinate.  Returns true on success and the input pointers
 * will have the appropriate latitude/longitude updated.  On failure returns
 * false.  Latitude and longitude are in DEGREES.
 */
bool AcquireGPSCoord(double *latitude, double *longitude) {
  // nullptr checks
  if (latitude == nullptr) {
    Serial.println("ERROR: latitude is NULL");
    return false;
  }

  if (longitude == nullptr) {
    Serial.println("ERROR: longitude is NULL");
    return false;
  }

  // Keep trying to acquire the latest coordinate
  bool gotValidCoord = false;
  while (!gotValidCoord) {
    Serial.println("Checking for new valid GPS data");

    // Check for new data
    myGNSS.checkUblox();

    if (nmea.isValid()) {
      // Got a valid coordinate.  Store and exit.
      *latitude = nmea.getLatitude() / 1000000.;
      *longitude = nmea.getLongitude() / 1000000.;
      nmea.clear();
      gotValidCoord = true;
    } else {
      // Sleep for a bit to avoid spamming GPS requests
      delay(500);
    }
  }
  return true;
}

/*
 * Waits for a pushbutton press.  Also implements the debouncing logic.
 */
void WaitForButtonPress() {
  unsigned long timer = 0;
  unsigned long interval = 20;

  boolean buttonState = 0;
  boolean lastButtonState = 0;

  boolean buttonPressed = false;
  boolean buttonReleased = false;

  // Wait until a button press is detected
  while (!buttonPressed || !buttonReleased) {
    // Wait for debouncing timer to expire
    if ((millis() - timer) >= interval) {
      timer = millis();

      // Read the pushbutton
      buttonState = digitalRead(BUTTON_PIN);

      if (buttonState != lastButtonState) {
        // Button is HIGH if not pressed; LOW if pressed
        if (buttonState == LOW) {
          buttonPressed = true;
        } else {
          buttonReleased = true;
        }

        // Save current state for next iteration
        lastButtonState = buttonState;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Wait for Serial Monitor to be opened
  while (!Serial);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  pinMode(BUTTON_PIN, INPUT);

  // Connect to the GPS.  On failure hang in this if-block.
  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS module not detected at default I2C address.  Please check wiring.  Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL);
  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);
}

void loop() {
  // Acquire first GPS coordinate
  Serial.println("Please push button for first GPS coordinate");
  WaitForButtonPress();

  double lat1, long1;
  const bool gps_succ1 = AcquireGPSCoord(&lat1, &long1);
  if (gps_succ1) {
    Serial.println("First GPS coordinate: ");
    Serial.print("Latitude (degrees): ");
    Serial.println(lat1, 6);
    Serial.print("Longitude (degrees): ");
    Serial.println(long1, 6);

    // Acquire second GPS coordinate
    Serial.println("Please push button for second GPS coordinate");
    WaitForButtonPress();

    double lat2, long2;
    const bool gps_succ2 = AcquireGPSCoord(&lat2, &long2);
    if (gps_succ2) {
      Serial.println("Second GPS coordinate: ");
      Serial.print("Latitude (degrees): ");
      Serial.println(lat2, 6);
      Serial.print("Longitude (degrees): ");
      Serial.println(long2, 6);

      // Compute the arc distance between the 2 coordinates
      const double dist = ComputeArcDist(lat1, long1, lat2, long2);
      Serial.print("Distance between the 2 GPS coordinates (meters): ");
      Serial.println(dist, 6);
    } else {
      Serial.println("ERROR: Unable to acquire second GPS coordinate");
    }
  } else {
    Serial.println("ERROR: Unable to acquire first GPS coordinate");
  }
}

void SFE_UBLOX_GNSS::processNMEA(char incoming) {
  nmea.process(incoming);
}
