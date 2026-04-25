    #include "configuration.h"

    #if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
    #include "driver/gpio.h"
    #include "esp_sleep.h"
    #include "../mesh/generated/meshtastic/telemetry.pb.h"
    #include "Default.h"
    #include "EnvironmentTelemetry.h"
    #include "MeshService.h"
    #include "NodeDB.h"
    #include "PowerFSM.h"
    #include "RTC.h"
    #include "Router.h"
    #include "UnitConversions.h"
    #include "buzz.h"
    #include "graphics/SharedUIDisplay.h"
    #include "graphics/images.h"
    #include "main.h"
    #include "modules/ExternalNotificationModule.h"
    #include "power.h"
    #include "sleep.h"
    #include "target_specific.h"
    #include <OLEDDisplay.h>
    static bool scheduleInitDone = false;
    static uint32_t nextDueMs = 0;
    static const uint32_t PERIOD_MS = 300000UL; // Periode totale de 300s
    static const uint32_t OFFSET_MS = 305000UL; // Premier envoi
    static const uint32_t GUARD_MS  = 200;   // reveil 2ms avant pour éviter de rater le slot d'envoi

    #if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL

    // Sensors
    #include "Sensor/CGRadSensSensor.h"
    #include "Sensor/RCWL9620Sensor.h"
    #include "Sensor/nullSensor.h"

    namespace graphics
    {
    extern void drawCommonHeader(OLEDDisplay *display, int16_t x, int16_t y, const char *titleStr, bool force_no_invert,
                                bool show_date);
    }
    #if __has_include(<Adafruit_AHTX0.h>)
    #include "Sensor/AHT10.h"
    #endif

    #if __has_include(<Adafruit_BME280.h>)
    #include "Sensor/BME280Sensor.h"
    #endif

    #if __has_include(<Adafruit_BMP085.h>)
    #include "Sensor/BMP085Sensor.h"
    #endif

    #if __has_include(<Adafruit_BMP280.h>)
    #include "Sensor/BMP280Sensor.h"
    #endif

    #if __has_include(<Adafruit_LTR390.h>)
    #include "Sensor/LTR390UVSensor.h"
    #endif

    #if __has_include(<bsec2.h>) || __has_include(<Adafruit_BME680.h>)
    #include "Sensor/BME680Sensor.h"
    #endif

    #if __has_include(<Adafruit_DPS310.h>)
    #include "Sensor/DPS310Sensor.h"
    #endif

    #if __has_include(<Adafruit_MCP9808.h>)
    #include "Sensor/MCP9808Sensor.h"
    #endif

    #if __has_include(<Adafruit_SHT31.h>)
    #include "Sensor/SHT31Sensor.h"
    #endif

    #if __has_include(<Adafruit_LPS2X.h>)
    #include "Sensor/LPS22HBSensor.h"
    #endif

    #if __has_include(<Adafruit_SHTC3.h>)
    #include "Sensor/SHTC3Sensor.h"
    #endif

    #if __has_include("RAK12035_SoilMoisture.h") && defined(RAK_4631) && RAK_4631 == 1
    #include "Sensor/RAK12035Sensor.h"
    #endif

    #if __has_include(<Adafruit_VEML7700.h>)
    #include "Sensor/VEML7700Sensor.h"
    #endif

    #if __has_include(<Adafruit_TSL2591.h>)
    #include "Sensor/TSL2591Sensor.h"
    #endif

    #if __has_include(<ClosedCube_OPT3001.h>)
    #include "Sensor/OPT3001Sensor.h"
    #endif

    #if __has_include(<Adafruit_SHT4x.h>)
    #include "Sensor/SHT4XSensor.h"
    #endif

    #if __has_include(<SparkFun_MLX90632_Arduino_Library.h>)
    #include "Sensor/MLX90632Sensor.h"
    #endif

    #if __has_include(<DFRobot_LarkWeatherStation.h>)
    #include "Sensor/DFRobotLarkSensor.h"
    #endif

    #if __has_include(<DFRobot_RainfallSensor.h>)
    #include "Sensor/DFRobotGravitySensor.h"
    #endif

    #if __has_include(<SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>)
    #include "Sensor/NAU7802Sensor.h"
    #endif

    #if __has_include(<Adafruit_BMP3XX.h>)
    #include "Sensor/BMP3XXSensor.h"
    #endif

    #if __has_include(<Adafruit_PCT2075.h>)
    #include "Sensor/PCT2075Sensor.h"
    #endif

    #endif
    #ifdef T1000X_SENSOR_EN
    #include "Sensor/T1000xSensor.h"
    #endif

    #ifdef SENSECAP_INDICATOR
    #include "Sensor/IndicatorSensor.h"
    #endif

    #if __has_include(<Adafruit_TSL2561_U.h>)
    #include "Sensor/TSL2561Sensor.h"
    #endif

    #if __has_include(<BH1750_WE.h>)
    #include "Sensor/BH1750Sensor.h"
    #endif

    #define FAILED_STATE_SENSOR_READ_MULTIPLIER 10
    #define DISPLAY_RECEIVEID_MEASUREMENTS_ON_SCREEN true

    #include "Sensor/AddI2CSensorTemplate.h"
    #include "graphics/ScreenFonts.h"
    #include <Throttle.h>

    // =====================
    // Rain gauge (reed switch) on GPIO
    // =====================
    #define RAIN_PIN 4
    #define RAIN_RESOLUTION_MM 0.2794f
    #define RAIN_WINDOW_MS 300000UL       // 5 min
    #define RAIN_SCALE 100.0f

    #define RAIN_MIN_TIP_INTERVAL_US 150000UL   // 150 ms between 2 valid tips

    static volatile uint32_t rainTips = 0;
    static volatile uint32_t lastRainTipUs = 0;

    static bool rainInitDone = false;
    static uint32_t lastRainCalcMs = 0;
    static float lastRainRateMmph = 0.0f;

    static void IRAM_ATTR rainISR()
    {
        uint32_t now = micros();

        if ((uint32_t)(now - lastRainTipUs) >= RAIN_MIN_TIP_INTERVAL_US) {
            rainTips++;
            lastRainTipUs = now;
        }
    }

    void EnvironmentTelemetryModule::i2cScanFinished(ScanI2C *i2cScanner)
    {
        if (!moduleConfig.telemetry.environment_measurement_enabled && !ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE) {
            return;
        }
        LOG_INFO("Environment Telemetry adding I2C devices...");

        // order by priority of metrics/values (low top, high bottom)

    #if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
    #ifdef T1000X_SENSOR_EN
        // Not a real I2C device
        addSensor<T1000xSensor>(i2cScanner, ScanI2C::DeviceType::NONE);
    #else
    #ifdef SENSECAP_INDICATOR
        // Not a real I2C device, uses UART
        addSensor<IndicatorSensor>(i2cScanner, ScanI2C::DeviceType::NONE);
    #endif
        addSensor<RCWL9620Sensor>(i2cScanner, ScanI2C::DeviceType::RCWL9620);
        addSensor<CGRadSensSensor>(i2cScanner, ScanI2C::DeviceType::CGRADSENS);
    #endif
    #endif

    #if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL
    #if __has_include(<DFRobot_LarkWeatherStation.h>)
        addSensor<DFRobotLarkSensor>(i2cScanner, ScanI2C::DeviceType::DFROBOT_LARK);
    #endif
    //#if __has_include(<DFRobot_RainfallSensor.h>)
    //    addSensor<DFRobotGravitySensor>(i2cScanner, ScanI2C::DeviceType::DFROBOT_RAIN);
    //#endif
    #if __has_include(<Adafruit_AHTX0.h>)
        addSensor<AHT10Sensor>(i2cScanner, ScanI2C::DeviceType::AHT10);
    #endif
    #if __has_include(<Adafruit_BMP085.h>)
        addSensor<BMP085Sensor>(i2cScanner, ScanI2C::DeviceType::BMP_085);
    #endif
    #if __has_include(<Adafruit_BME280.h>)
        addSensor<BME280Sensor>(i2cScanner, ScanI2C::DeviceType::BME_280);
    #endif
    #if __has_include(<Adafruit_LTR390.h>)
        addSensor<LTR390UVSensor>(i2cScanner, ScanI2C::DeviceType::LTR390UV);
    #endif
    #if __has_include(<bsec2.h>) || __has_include(<Adafruit_BME680.h>)
        addSensor<BME680Sensor>(i2cScanner, ScanI2C::DeviceType::BME_680);
    #endif
    #if __has_include(<Adafruit_BMP280.h>)
        addSensor<BMP280Sensor>(i2cScanner, ScanI2C::DeviceType::BMP_280);
    #endif
    #if __has_include(<Adafruit_DPS310.h>)
        addSensor<DPS310Sensor>(i2cScanner, ScanI2C::DeviceType::DPS310);
    #endif
    #if __has_include(<Adafruit_MCP9808.h>)
        addSensor<MCP9808Sensor>(i2cScanner, ScanI2C::DeviceType::MCP9808);
    #endif
    #if __has_include(<Adafruit_SHT31.h>)
        addSensor<SHT31Sensor>(i2cScanner, ScanI2C::DeviceType::SHT31);
    #endif
    #if __has_include(<Adafruit_LPS2X.h>)
        addSensor<LPS22HBSensor>(i2cScanner, ScanI2C::DeviceType::LPS22HB);
    #endif
    #if __has_include(<Adafruit_SHTC3.h>)
        addSensor<SHTC3Sensor>(i2cScanner, ScanI2C::DeviceType::SHTC3);
    #endif
    #if __has_include("RAK12035_SoilMoisture.h") && defined(RAK_4631) && RAK_4631 == 1
        addSensor<RAK12035Sensor>(i2cScanner, ScanI2C::DeviceType::RAK12035);
    #endif
    #if __has_include(<Adafruit_VEML7700.h>)
        addSensor<VEML7700Sensor>(i2cScanner, ScanI2C::DeviceType::VEML7700);
    #endif
    #if __has_include(<Adafruit_TSL2591.h>)
        addSensor<TSL2591Sensor>(i2cScanner, ScanI2C::DeviceType::TSL2591);
    #endif
    #if __has_include(<ClosedCube_OPT3001.h>)
        addSensor<OPT3001Sensor>(i2cScanner, ScanI2C::DeviceType::OPT3001);
    #endif
    #if __has_include(<Adafruit_SHT4x.h>)
        addSensor<SHT4XSensor>(i2cScanner, ScanI2C::DeviceType::SHT4X);
    #endif
    #if __has_include(<SparkFun_MLX90632_Arduino_Library.h>)
        addSensor<MLX90632Sensor>(i2cScanner, ScanI2C::DeviceType::MLX90632);
    #endif

    #if __has_include(<Adafruit_BMP3XX.h>)
        addSensor<BMP3XXSensor>(i2cScanner, ScanI2C::DeviceType::BMP_3XX);
    #endif
    #if __has_include(<Adafruit_PCT2075.h>)
        addSensor<PCT2075Sensor>(i2cScanner, ScanI2C::DeviceType::PCT2075);
    #endif
    #if __has_include(<Adafruit_TSL2561_U.h>)
        addSensor<TSL2561Sensor>(i2cScanner, ScanI2C::DeviceType::TSL2561);
    #endif
    #if __has_include(<SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>)
        addSensor<NAU7802Sensor>(i2cScanner, ScanI2C::DeviceType::NAU7802);
    #endif
    #if __has_include(<BH1750_WE.h>)
        addSensor<BH1750Sensor>(i2cScanner, ScanI2C::DeviceType::BH1750);
    #endif

    #endif
    }

    int32_t EnvironmentTelemetryModule::runOnce()
{
    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
    if (wakeCause == ESP_SLEEP_WAKEUP_GPIO) {
    LOG_INFO("[WAKE] wakeup by GPIO, rain pin state=%d", digitalRead(RAIN_PIN));
    }

    uint32_t result = UINT32_MAX;

    if (!(moduleConfig.telemetry.environment_measurement_enabled ||
          moduleConfig.telemetry.environment_screen_enabled ||
          ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE)) {
        return disable();
    }

    if (firstTime) {
        firstTime = 0;

        if (!scheduleInitDone) {
            uint32_t now = millis();
            nextDueMs = now + OFFSET_MS;
            scheduleInitDone = true;
            LOG_INFO("Telemetry schedule init: nextDueMs=%u (now=%u)", nextDueMs, now);
        }

        if (!rainInitDone) {
            pinMode(RAIN_PIN, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, FALLING);

            gpio_wakeup_enable((gpio_num_t)RAIN_PIN, GPIO_INTR_LOW_LEVEL);
            esp_sleep_enable_gpio_wakeup();

            rainInitDone = true;
            lastRainCalcMs = millis();
            lastRainTipUs = 0;

            LOG_INFO("RAIN init: GPIO=%d, ISR ok, GPIO wakeup on LOW enabled", RAIN_PIN);
        }

        if (moduleConfig.telemetry.environment_measurement_enabled ||
            ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE) {
            LOG_INFO("Environment Telemetry: init");

            if (!sensors.empty()) {
                result = DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
            }

#ifndef T1000X_SENSOR_EN
#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL
            if (ina219Sensor.hasSensor())
                result = ina219Sensor.runOnce();
            if (ina260Sensor.hasSensor())
                result = ina260Sensor.runOnce();
            if (ina3221Sensor.hasSensor())
                result = ina3221Sensor.runOnce();
            if (max17048Sensor.hasSensor())
                result = max17048Sensor.runOnce();
#ifdef HAS_RAKPROT
            result = rak9154Sensor.runOnce();
#endif
#endif
#endif
        }

        return result == UINT32_MAX ? disable() : setStartDelay();
    }

    if (!moduleConfig.telemetry.environment_measurement_enabled &&
        !ENVIRONMENTAL_TELEMETRY_MODULE_ENABLE) {
        return disable();
    }

    for (TelemetrySensor *sensor : sensors) {
        uint32_t delay = sensor->runOnce();
        if (delay < result) {
            result = delay;
        }
    }

    uint32_t now = millis();
    if (now - lastRainCalcMs >= RAIN_WINDOW_MS) {
        uint32_t tips;

        noInterrupts();
        tips = rainTips;
        rainTips = 0;
        interrupts();

        float rain_mm = tips * RAIN_RESOLUTION_MM;
        lastRainRateMmph = rain_mm * (3600000.0f / (float)RAIN_WINDOW_MS);
        lastRainCalcMs = now;

        LOG_INFO("RAIN: tips=%u rain=%.2f mm/h", tips, lastRainRateMmph);
    }

    // Avant le slot -> attendre
   if ((int32_t)(now - nextDueMs) < 0) {
        uint32_t waitMs = nextDueMs - now;

        if (waitMs > GUARD_MS)
            waitMs -= GUARD_MS;
        else
            waitMs = 0;

        // Do not enter long sleep while rain contact is still LOW
        if (digitalRead(RAIN_PIN) == LOW) {
            waitMs = 20;
            LOG_INFO("[RAIN] pin still LOW, postpone sleep");
        }

        LOG_INFO("[SLEEP] waitMs=%lu", (unsigned long)waitMs);
        setIntervalFromNow(waitMs);
        return waitMs;
    }

    // Slot atteint
    LOG_INFO("[WAKE] now=%lu nextDue=%lu",
             (unsigned long)now,
             (unsigned long)nextDueMs);

    if (airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
        airTime->isTxAllowedAirUtil()) {

        static uint32_t slotIndex = 0;
        int32_t lateMs = (int32_t)now - (int32_t)nextDueMs;

        LOG_INFO("[SLOT] #%lu now=%lu nextDue=%lu late=%ldms (guard=%lu)",
                 (unsigned long)slotIndex,
                 (unsigned long)now,
                 (unsigned long)nextDueMs,
                 (long)lateMs,
                 (unsigned long)GUARD_MS);

        LOG_INFO("[TX] sending telemetry");
        sendTelemetry();
        slotIndex++;
        lastSentToMesh = now;

        do {
            nextDueMs += PERIOD_MS;
        } while ((int32_t)(now - nextDueMs) >= 0);

        uint32_t waitMs = nextDueMs - now;
        if (waitMs > GUARD_MS)
            waitMs -= GUARD_MS;
        else
            waitMs = 0;

        if (digitalRead(RAIN_PIN) == LOW) {
            waitMs = 20;
            LOG_INFO("[RAIN] pin still LOW after TX, postpone sleep");
        }

        LOG_INFO("[SLEEP] waitMs=%lu", (unsigned long)waitMs);
        setIntervalFromNow(waitMs);
        return waitMs;
    }
    return min(sendToPhoneIntervalMs, result);
}

    bool EnvironmentTelemetryModule::wantUIFrame()
    {
        return moduleConfig.telemetry.environment_screen_enabled;
    }

    #if HAS_SCREEN
    void EnvironmentTelemetryModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
    {
        // === Setup display ===
        display->clear();
        display->setFont(FONT_SMALL);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        int line = 1;

        // === Set Title
        const char *titleStr = (graphics::currentResolution == graphics::ScreenResolution::High) ? "Environment" : "Env.";

        // === Header ===
        graphics::drawCommonHeader(display, x, y, titleStr);

        // === Row spacing setup ===
        const int rowHeight = FONT_HEIGHT_SMALL - 4;
        int currentY = graphics::getTextPositions(display)[line++];

        // === Show "No Telemetry" if no data available ===
        if (!lastMeasurementPacket) {
            display->drawString(x, currentY, "No Telemetry");
            return;
        }

        // Decode the telemetry message from the latest received packet
        const meshtastic_Data &p = lastMeasurementPacket->decoded;
        meshtastic_Telemetry telemetry;
        if (!pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &telemetry)) {
            display->drawString(x, currentY, "No Telemetry");
            return;
        }

        const auto &m = telemetry.variant.environment_metrics;

        // Check if any telemetry field has valid data
        bool hasAny = m.has_temperature || m.has_relative_humidity || m.barometric_pressure != 0 || m.iaq != 0 || m.voltage != 0 ||
                    m.current != 0 || m.lux != 0 || m.white_lux != 0 || m.weight != 0 || m.distance != 0 || m.radiation != 0;

        if (!hasAny) {
            display->drawString(x, currentY, "No Telemetry");
            return;
        }

        // === First line: Show sender name + time since received (left), and first metric (right) ===
        const char *sender = getSenderShortName(*lastMeasurementPacket);
        uint32_t agoSecs = service->GetTimeSinceMeshPacket(lastMeasurementPacket);
        String agoStr = (agoSecs > 864000) ? "?"
                        : (agoSecs > 3600) ? String(agoSecs / 3600) + "h"
                        : (agoSecs > 60)   ? String(agoSecs / 60) + "m"
                                        : String(agoSecs) + "s";

        String leftStr = String(sender) + " (" + agoStr + ")";
        display->drawString(x, currentY, leftStr); // Left side: who and when

        // === Collect sensor readings as label strings (no icons) ===
        std::vector<String> entries;

        if (m.has_temperature) {
            String tempStr = moduleConfig.telemetry.environment_display_fahrenheit
                                ? "Tmp: " + String(UnitConversions::CelsiusToFahrenheit(m.temperature), 1) + "°F"
                                : "Tmp: " + String(m.temperature, 1) + "°C";
            entries.push_back(tempStr);
        }
        if (m.has_relative_humidity)
            entries.push_back("Hum: " + String(m.relative_humidity, 0) + "%");
        if (m.barometric_pressure != 0)
            entries.push_back("Prss: " + String(m.barometric_pressure, 0) + " hPa");
        if (m.iaq != 0) {
            float rain = ((float)m.iaq) / RAIN_SCALE;
            entries.push_back("Rain: " + String(rain, 1) + " mm/h");
        }
        if (m.voltage != 0 || m.current != 0)
            entries.push_back(String(m.voltage, 1) + "V / " + String(m.current, 0) + "mA");
        if (m.lux != 0)
            entries.push_back("Light: " + String(m.lux, 0) + "lx");
        if (m.white_lux != 0)
            entries.push_back("White: " + String(m.white_lux, 0) + "lx");
        if (m.weight != 0)
            entries.push_back("Weight: " + String(m.weight, 0) + "kg");
        if (m.radiation != 0)
            entries.push_back("Rad: " + String(m.radiation, 2) + " µR/h");

        // === Show first available metric on top-right of first line ===
        if (!entries.empty()) {
            String valueStr = entries.front();
            int rightX = SCREEN_WIDTH - display->getStringWidth(valueStr);
            display->drawString(rightX, currentY, valueStr);
            entries.erase(entries.begin()); // Remove from queue
        }

        // === Advance to next line for remaining telemetry entries ===
        currentY += rowHeight;

        // === Draw remaining entries in 2-column format (left and right) ===
        for (size_t i = 0; i < entries.size(); i += 2) {
            // Left column
            display->drawString(x, currentY, entries[i]);

            // Right column if it exists
            if (i + 1 < entries.size()) {
                int rightX = SCREEN_WIDTH / 2;
                display->drawString(rightX, currentY, entries[i + 1]);
            }

            currentY += rowHeight;
        }
        graphics::drawCommonFooter(display, x, y);
    }
    #endif

    bool EnvironmentTelemetryModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *t)
    {
        if (t->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
    #if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
            const char *sender = getSenderShortName(mp);

            LOG_INFO("(Received from %s): barometric_pressure=%f, current=%f, gas_resistance=%f, relative_humidity=%f, "
                    "temperature=%f",
                    sender, t->variant.environment_metrics.barometric_pressure, t->variant.environment_metrics.current,
                    t->variant.environment_metrics.gas_resistance, t->variant.environment_metrics.relative_humidity,
                    t->variant.environment_metrics.temperature);
            LOG_INFO("(Received from %s): voltage=%f, Rain=%d, distance=%f, lux=%f, white_lux=%f", sender,
                    t->variant.environment_metrics.voltage, t->variant.environment_metrics.iaq,
                    t->variant.environment_metrics.distance, t->variant.environment_metrics.lux,
                    t->variant.environment_metrics.white_lux);

            LOG_INFO("(Received from %s): wind speed=%fm/s, direction=%d degrees, weight=%fkg", sender,
                    t->variant.environment_metrics.wind_speed, t->variant.environment_metrics.wind_direction,
                    t->variant.environment_metrics.weight);

            LOG_INFO("(Received from %s): radiation=%fµR/h", sender, t->variant.environment_metrics.radiation);

    #endif
            // release previous packet before occupying a new spot
            if (lastMeasurementPacket != nullptr)
                packetPool.release(lastMeasurementPacket);

            lastMeasurementPacket = packetPool.allocCopy(mp);
        }

        return false; // Let others look at this message also if they want
    }

    bool EnvironmentTelemetryModule::getEnvironmentTelemetry(meshtastic_Telemetry *m)
    {
        bool valid = false;
        bool hasSensor = false;
        // getMetrics() doesn't always get evaluated because of
        // short-circuit evaluation rules in c++
        bool get_metrics;
        m->time = getTime();
        m->which_variant = meshtastic_Telemetry_environment_metrics_tag;
        m->variant.environment_metrics = meshtastic_EnvironmentMetrics_init_zero;

        uint32_t now = millis(); // Permet de sécuriser le calcul du taux de pluie même si getEnvironmentTelemetry est appelé plus fréquemment que RAIN_WINDOW_MS (desynchronisation)
        if (now - lastRainCalcMs >= RAIN_WINDOW_MS) {

                uint32_t tips;

                noInterrupts();
                tips = rainTips;
                rainTips = 0;
                interrupts();

                float rain_mm = tips * RAIN_RESOLUTION_MM;

                lastRainRateMmph = rain_mm * (3600000.0f / (float)RAIN_WINDOW_MS);

                lastRainCalcMs = now;

                LOG_INFO("RAIN (forced calc): tips=%u rain=%.2f mm/h", tips, lastRainRateMmph);
            }

        for (TelemetrySensor *sensor : sensors) {
            get_metrics = sensor->getMetrics(m); // avoid short-circuit evaluation rules
            valid = valid || get_metrics;
            hasSensor = true;
        }
        

    #ifndef T1000X_SENSOR_EN
        if (ina219Sensor.hasSensor()) {
            get_metrics = ina219Sensor.getMetrics(m);
            valid = valid || get_metrics;
            hasSensor = true;
        }
        if (ina260Sensor.hasSensor()) {
            get_metrics = ina260Sensor.getMetrics(m);
            valid = valid || get_metrics;
            hasSensor = true;
        }
        if (ina3221Sensor.hasSensor()) {
            get_metrics = ina3221Sensor.getMetrics(m);
            valid = valid || get_metrics;
            hasSensor = true;
        }
        if (max17048Sensor.hasSensor()) {
            get_metrics = max17048Sensor.getMetrics(m);
            valid = valid || get_metrics;
            hasSensor = true;
        }
    #endif
    #ifdef HAS_RAKPROT
        get_metrics = rak9154Sensor.getMetrics(m);
        valid = valid || get_metrics;
        hasSensor = true;
    #endif
        // OVERRIDE IAQ with rain intensity
        uint32_t precipBuck = (uint32_t)lroundf(lastRainRateMmph * RAIN_SCALE);
        m->variant.environment_metrics.has_iaq = true;
        m->variant.environment_metrics.iaq = precipBuck;
        LOG_INFO("RAIN->IAQ = %u (%.2f mm/h)", precipBuck, lastRainRateMmph);
        return valid && hasSensor;
    }

    meshtastic_MeshPacket *EnvironmentTelemetryModule::allocReply()
    {
        if (currentRequest) {
            auto req = *currentRequest;
            const auto &p = req.decoded;
            meshtastic_Telemetry scratch;
            meshtastic_Telemetry *decoded = NULL;
            memset(&scratch, 0, sizeof(scratch));
            if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &scratch)) {
                decoded = &scratch;
            } else {
                LOG_ERROR("Error decoding EnvironmentTelemetry module!");
                return NULL;
            }
            // Check for a request for environment metrics
            if (decoded->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
                meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
                if (getEnvironmentTelemetry(&m)) {
                    LOG_INFO("Environment telemetry reply to request");
                    return allocDataProtobuf(m);
                } else {
                    return NULL;
                }
            }
        }
        return NULL;
    }

    bool EnvironmentTelemetryModule::sendTelemetry(NodeNum dest, bool phoneOnly)
    {
        meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
        m.which_variant = meshtastic_Telemetry_environment_metrics_tag;
        m.time = getTime();

        if (getEnvironmentTelemetry(&m)) {
            LOG_INFO("Send: barometric_pressure=%f, current=%f, gas_resistance=%f, relative_humidity=%f, temperature=%f",
                    m.variant.environment_metrics.barometric_pressure, m.variant.environment_metrics.current,
                    m.variant.environment_metrics.gas_resistance, m.variant.environment_metrics.relative_humidity,
                    m.variant.environment_metrics.temperature);
            LOG_INFO("Send: voltage=%f, Rain=%d, distance=%f, lux=%f", m.variant.environment_metrics.voltage,
                    m.variant.environment_metrics.iaq, m.variant.environment_metrics.distance, m.variant.environment_metrics.lux);

            LOG_INFO("Send: wind speed=%fm/s, direction=%d degrees, weight=%fkg", m.variant.environment_metrics.wind_speed,
                    m.variant.environment_metrics.wind_direction, m.variant.environment_metrics.weight);

            LOG_INFO("Send: radiation=%fµR/h", m.variant.environment_metrics.radiation);

            LOG_INFO("Send: soil_temperature=%f, soil_moisture=%u", m.variant.environment_metrics.soil_temperature,
                    m.variant.environment_metrics.soil_moisture);

            meshtastic_MeshPacket *p = allocDataProtobuf(m);
            p->to = dest;
            p->decoded.want_response = false;
            if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
                p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
            else
                p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
            // release previous packet before occupying a new spot
            if (lastMeasurementPacket != nullptr)
                packetPool.release(lastMeasurementPacket);

            lastMeasurementPacket = packetPool.allocCopy(*p);
            if (phoneOnly) {
                LOG_INFO("Send packet to phone");
                service->sendToPhone(p);
            } else {
                LOG_INFO("Send packet to mesh");
                service->sendToMesh(p, RX_SRC_LOCAL, true);

                if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR && config.power.is_power_saving) {
                    meshtastic_ClientNotification *notification = clientNotificationPool.allocZeroed();
                    notification->level = meshtastic_LogRecord_Level_INFO;
                    notification->time = getValidTime(RTCQualityFromNet);
                    sprintf(notification->message, "Sending telemetry and sleeping for %us interval in a moment",
                            Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.environment_update_interval,
                                                            default_telemetry_broadcast_interval_secs) /
                                1000U);
                    service->sendClientNotification(notification);
                    LOG_DEBUG("Start next execution in 5s, then sleep");
                }
            }
            return true;
        }
        return false;
    }

    AdminMessageHandleResult EnvironmentTelemetryModule::handleAdminMessageForModule(const meshtastic_MeshPacket &mp,
                                                                                    meshtastic_AdminMessage *request,
                                                                                    meshtastic_AdminMessage *response)
    {
        AdminMessageHandleResult result = AdminMessageHandleResult::NOT_HANDLED;
    #if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR_EXTERNAL

        for (TelemetrySensor *sensor : sensors) {
            result = sensor->handleAdminMessage(mp, request, response);
            if (result != AdminMessageHandleResult::NOT_HANDLED)
                return result;
        }

        if (ina219Sensor.hasSensor()) {
            result = ina219Sensor.handleAdminMessage(mp, request, response);
            if (result != AdminMessageHandleResult::NOT_HANDLED)
                return result;
        }
        if (ina260Sensor.hasSensor()) {
            result = ina260Sensor.handleAdminMessage(mp, request, response);
            if (result != AdminMessageHandleResult::NOT_HANDLED)
                return result;
        }
        if (ina3221Sensor.hasSensor()) {
            result = ina3221Sensor.handleAdminMessage(mp, request, response);
            if (result != AdminMessageHandleResult::NOT_HANDLED)
                return result;
        }
        if (max17048Sensor.hasSensor()) {
            result = max17048Sensor.handleAdminMessage(mp, request, response);
            if (result != AdminMessageHandleResult::NOT_HANDLED)
                return result;
        }
    #endif
        return result;
    }

    #endif
