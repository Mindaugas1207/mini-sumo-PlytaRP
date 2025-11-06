
#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "one_wire_device.h"

class DistanceSensor : public OneWireDevice
{
public:
    enum RangeFlags
    {
        RANGE_FLAG_VALID = 0x01, // measurement valid
        RANGE_FLAG_WRAP = 0x02, // range wrapped around
        RANGE_FLAG_PHASE = 0x04, // out of phase
        RANGE_FLAG_SIGNAL_LOW = 0x08, // signal too low
        RANGE_FLAG_SIGMA_LOW = 0x10, // sigma too low
        RANGE_FLAG_SIGMA_HIGH = 0x20, // sigma too high
        RANGE_FLAG_MIN = 0x40, // below minimum range
        RANGE_FLAG_MAX = 0x80 // above maximum range
    };

    enum DetectionMode
    {
        DETECTION_MODE_ERROR = -1,
        DETECTION_MODE_ANY_VALID_RANGE = 0,
        DETECTION_MODE_THRESHOLD = 1,
        DETECTION_MODE_WINDOW = 2
    };

    DistanceSensor(OneWire &oneWire, uint index);

    /*! \brief Get the range status flags from the last measurement
     * \return RangeFlags bitmask, or 0 on error
     */
    RangeFlags getRangeFlags(void);

    /*! \brief Get the last measured distance
     * \return Distance in mm, or -1 on error
     */
    int getDistance_mm(void);

    /*! \brief Get the last measured distance
     * \return Distance in cm, or -1 on error
     */
    int getDistance_cm(void);

    /*! \brief Get the number of SPADs used in the last measurement
     * \return SPAD count, or -1 on error
    */
    int getSPADCount(void);

    /*! \brief Get the signal rate from the last measurement
     * \return Signal rate in MCPS (Mega Counts Per Second), or -1 on error
     */
    int getSignalRate_MCPS(void);

    /*! \brief Get the ambient rate from the last measurement
     * \return Ambient rate in MCPS (Mega Counts Per Second), or -1 on error
     */
    int getAmbientRate_MCPS(void);

    /*! \brief Get the signal sigma from the last measurement
     * \return Signal sigma in mm, or -1 on error
     */
    int getSignalSigma_mm(void);

    /*! \brief Get the range check flags
     * \return RangeFlags bitmask, or 0 on error
    */
    RangeFlags getRangeChecks(void);

    /*! \brief Get the maximum range distance
     * \return Maximum distance in mm, or -1 on error
    */
    int getRangeMax_mm(void);

    /*! \brief Get the minimum range distance
    * \return Minimum distance in mm, or -1 on error
    */
    int getRangeMin_mm(void);

    /*! \brief Get the range timing budget
     * \return Timing budget in ms, or -1 on error
    */
    int getRangeTiming_ms(void);

    int getRangeLinearCorrection(void);

    /*! \brief Get the range offset
     * \return Offset in mm, or -1 on error
    */
    int getRangeOffset(void);

    /*! \brief Get the crosstalk compensation value
     * \return Crosstalk in counts, or -1 on error
    */
    int getRangeXTALK(void);

    /*! \brief Get the signal limit used in the last measurement
     * \return Signal limit in MCPS (Mega Counts Per Second), or -1 on error
    */
    int getRangeSignalLimit_MCPS(void);

    /*! \brief Get the sigma limit used in the last measurement
     * \return Sigma limit in mm, or -1 on error
    */
    int getRangeSigmaLimit_mm(void);

    /*! \brief Set the range check flags
     * \param flags RangeFlags bitmask
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeChecks(RangeFlags flags);

    /*! \brief Set the maximum range distance
     * \param maxDistance Maximum distance in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeMax_mm(uint maxDistance);

    /*! \brief Set the minimum range distance
     * \param minDistance Minimum distance in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeMin_mm(uint minDistance);

    /*! \brief Set the range timing budget
     * \param timing Timing budget in ms (10 to 200)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeTiming_ms(uint timing);

    bool setRangeLinearCorrection(uint correction);

    /*! \brief Set the range offset
     * \param offset Offset in mm (-32768 to 32767)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeOffset(uint offset);

    /*! \brief Set the crosstalk compensation value
     * \param xtalk Crosstalk in counts (0 to 32767)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeXTALK(uint xtalk);

    /*! \brief Set the signal limit used in measurements
     * \param limit Signal limit in MCPS (Mega Counts Per Second)
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeSignalLimit_MCPS(uint limit);

    /*! \brief Set the sigma limit used in measurements
     * \param limit Sigma limit in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setRangeSigmaLimit_mm(uint limit);

    /*! \brief Set the detection mode
     * \param mode DetectionMode value
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionMode(DetectionMode mode);

    /*! \brief Get the detection mode
     * \return DetectionMode value, or DETECTION_MODE_ERROR on error
    */
    DetectionMode getDetectionMode(void);

    /*! \brief Set whether the detection output is inverted
     * \param inverted true to invert the output, false for normal operation
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionInverted(bool inverted);

    /*! \brief Get whether the detection output is inverted
     * \param inverted Reference to variable to receive the result
     * \return true if the command was acknowledged, false on error
    */
    bool getDetectionInverted(bool &inverted);

    /*! \brief Get the detection output state
     * \param detected Reference to variable to receive the result
     * \return true if the command was acknowledged, false on error
    */
    bool getDetectionState(bool &detected);

    /*! \brief Get the lower threshold for detection mode
     * \return Lower threshold in mm, or -1 on error
    */
    int getDetectionLowerThreshold_mm(void);

    /*! \brief Set the lower threshold for detection mode
     * \param threshold Lower threshold in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionLowerThreshold_mm(uint threshold);

    /*! \brief Get the upper threshold for detection mode
     * \return Upper threshold in mm, or -1 on error
    */
    int getDetectionUpperThreshold_mm(void);

    /*! \brief Set the upper threshold for detection mode
     * \param threshold Upper threshold in mm
     * \return true if the command was acknowledged, false on error
    */
    bool setDetectionUpperThreshold_mm(uint threshold);
};

#endif // DISTANCE_SENSOR_H
