#include "yei_core.h"

const TSS_Command simple_commands[260]={
/*000*/    {0x00,"getTaredOrientationAsQuaternion", 16, "ffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*001*/    {0x01,"getTaredOrientationAsEulerAngles", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*002*/    {0x02,"getTaredOrientationAsRotationMatrix", 36, "fffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*003*/    {0x03,"getTaredOrientationAsAxisAngle", 16, "ffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*004*/    {0x04,"getTaredOrientationAsTwoVector", 24, "ffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*005*/    {0x05,"getDifferenceQuaternion", 16, "ffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*006*/    {0x06,"getUntaredOrientationAsQuaternion", 16, "ffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*007*/    {0x07,"getUntaredOrientationAsEulerAngles", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*008*/    {0x08,"getUntaredOrientationAsRotationMatrix", 36, "fffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*009*/    {0x09,"getUntaredOrientationAsAxisAngle", 16, "ffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*010*/    {0x0a,"getUntaredOrientationAsTwoVector", 24, "ffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*011*/    {0x0b,"getTaredTwoVectorInSensorFrame", 24, "ffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*012*/    {0x0c,"getUntaredTwoVectorInSensorFrame", 24, "ffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*013*/    {0x0d,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*014*/    {0x0e,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*015*/    {0x0f,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*016*/    {0x10,"setEulerAngleDecompositionOrder", 0, "", 1, "B",  0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*017*/    {0x11,"setMagnetoresistiveThreshold", 0, "", 16, "fIff", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*018*/    {0x12,"setAccelerometerResistanceThreshold", 0, "", 8, "fI", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*019*/    {0x13,"offsetWithCurrentOrientation", 0, "", 0, "", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*020*/    {0x14,"resetBaseOffset", 0, "", 0, "", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*021*/    {0x15,"offsetWithQuaternion", 0, "", 16, "ffff", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*022*/    {0x16,"setBaseOffsetWithCurrentOrientation", 0, "", 0, "", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*023*/    {0x17,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*024*/    {0x18,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*025*/    {0x19,"checkLongCommands", 1, "B", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*026*/    {0x1a,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*027*/    {0x1b,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*028*/    {0x1c,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*029*/    {0x1d,"setPinMode", 0, "", 2, "BB", 0x8000000, TSS_FW_20R7_COMPATIBLE},
/*030*/    {0x1e,"getPinMode", 2, "BB", 0, "", 0x8000000, TSS_FW_20R7_COMPATIBLE},
/*031*/    {0x1f,"getInterruptStatus", 1, "B", 0, "", 0x8000000, TSS_FW_20R7_COMPATIBLE},
/*032*/    {0x20,"getAllNormalizedComponentSensorData", 36, "fffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*033*/    {0x21,"getNormalizedGyroRate", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*034*/    {0x22,"getNormalizedAccelerometerVector", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*035*/    {0x23,"getNormalizedCompassVector", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*036*/    {0x24,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*037*/    {0x25,"getAllCorrectedComponentSensorData", 36, "fffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*038*/    {0x26,"getCorrectedGyroRate", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*039*/    {0x27,"getCorrectedAccelerometerVector", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*040*/    {0x28,"getCorrectedCompassVector", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*041*/    {0x29,"getCorrectedLinearAccelerationInGlobalSpace", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*042*/    {0x2a,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*043*/    {0x2b,"getTemperatureC", 4, "f", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*044*/    {0x2c,"getTemperatureF", 4, "f", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*045*/    {0x2d,"getConfidenceFactor", 4, "f", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*046*/    {0x2e,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*047*/    {0x2f,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*048*/    {0x30,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*049*/    {0x31,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*050*/    {0x32,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*051*/    {0x33,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*052*/    {0x34,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*053*/    {0x35,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*054*/    {0x36,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*055*/    {0x37,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*056*/    {0x38,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*057*/    {0x39,"turnOnMassStorage", 0, "", 0, "",0x40000000, TSS_FW_20R7_COMPATIBLE},
/*058*/    {0x3a,"turnOffMassStorage", 0, "", 0, "",0x40000000, TSS_FW_20R7_COMPATIBLE},
/*059*/    {0x3b,"formatAndInitializeSDCard", 0, "", 0, "", 0x40000000, TSS_FW_20R7_COMPATIBLE},
/*060*/    {0x3c,"beginDataLoggingSession", 0, "", 0, "", 0x40000000, TSS_FW_20R7_COMPATIBLE},
/*061*/    {0x3d,"endDataLoggingSession", 0, "", 0, "", 0x40000000, TSS_FW_20R7_COMPATIBLE},
/*062*/    {0x3e,"setClockValues", 0, "", 6, "BBBBBB", 0x40000000, TSS_FW_20R7_COMPATIBLE},
/*063*/    {0x3f,"getClockValues", 6, "BBBBBB", 0, "", 0x40000000, TSS_FW_20R7_COMPATIBLE},
/*064*/    {0x40,"getAllRawComponentSensorData", 36, "fffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*065*/    {0x41,"getRawGyroscopeRate", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*066*/    {0x42,"getRawAccelerometerData", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*067*/    {0x43,"getRawCompassData", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*068*/    {0x44,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*069*/    {0x45,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*070*/    {0x46,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*071*/    {0x47,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*072*/    {0x48,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*073*/    {0x49,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*074*/    {0x4a,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*075*/    {0x4b,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*076*/    {0x4c,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*077*/    {0x4d,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*078*/    {0x4e,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*079*/    {0x4f,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*080*/    {0x50,"setStreamingSlots", 0, "", 8, "BBBBBBBB", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*081*/    {0x51,"getStreamingSlots", 8, "BBBBBBBB", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*082*/    {0x52,"setStreamingTiming", 0, "", 12, "III", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*083*/    {0x53,"getStreamingTiming", 12, "III", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*084*/    {0x54,"getStreamingBatch", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*085*/    {0x55,"startStreaming", 0, "", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*086*/    {0x56,"stopStreaming", 0, "", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*087*/    {0x57,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*088*/    {0x58,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*089*/    {0x59,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*090*/    {0x5a,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*091*/    {0x5b,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*092*/    {0x5c,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*093*/    {0x5d,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*094*/    {0x5e,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*095*/    {0x5f,"updateCurrentTimestamp", 0, "", 4, "I", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*096*/    {0x60,"tareWithCurrentOrientation", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*097*/    {0x61,"tareWithQuaternion", 0, "", 16, "ffff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*098*/    {0x62,"tareWithRotationMatrix", 0, "", 36, "fffffffff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*099*/    {0x63,"setStaticAccelerometerTrustValue", 0, "", 4, "f", 0xfc000000, TSS_FW_20R10_COMPATIBLE},
/*100*/    {0x64,"setConfidenceAccelerometerTrustValues", 0, "", 8, "ff", 0xfc000000, TSS_FW_20R10_COMPATIBLE},
/*101*/    {0x65,"setStaticCompassTrustValue", 0, "", 4, "f", 0xfc000000, TSS_FW_20R10_COMPATIBLE},
/*102*/    {0x66,"setConfidenceCompassTrustValues", 0, "", 8, "ff", 0xfc000000, TSS_FW_20R10_COMPATIBLE},
/*103*/    {0x67,"setDesiredUpdateRate", 0, "", 4, "I", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*104*/    {0x68,"setMultiReferenceVectorsWithCurrentOrientation", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*105*/    {0x69,"setReferenceVectorMode", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*106*/    {0x6a,"setOversampleRate", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*107*/    {0x6b,"setGyroscopeEnabled", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*108*/    {0x6c,"setAccelerometerEnabled", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*109*/    {0x6d,"setCompassEnabled", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*110*/    {0x6e,"resetMultiReferenceVectorsToZero", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*111*/    {0x6f,"setMultiReferenceTableResolution", 0, "", 2, "BB", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*112*/    {0x70,"setCompassMultiReferenceVector", 0, "", 13, "Bfff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*113*/    {0x71,"setCompassMultiReferenceCheckVector", 0, "", 13, "Bfff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*114*/    {0x72,"setAccelerometerMultiReferenceVector", 0, "", 13, "Bfff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*115*/    {0x73,"setAccelerometerMultiReferenceCheckVector", 0, "", 13, "Bfff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*116*/    {0x74,"setAxisDirections", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*117*/    {0x75,"setRunningAveragePercent", 0, "", 4, "f", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*118*/    {0x76,"setCompassReferenceVector", 0, "", 12, "fff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*119*/    {0x77,"setAccelerometerReferenceVector", 0, "", 12, "fff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*120*/    {0x78,"resetKalmanFilter", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*121*/    {0x79,"setAccelerometerRange", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*122*/    {0x7a,"setMultiReferenceWeightPower", 0, "", 4, "f", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*123*/    {0x7b,"setFilterMode", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*124*/    {0x7c,"setRunningAverageMode", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*125*/    {0x7d,"setGyroscopeRange", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*126*/    {0x7e,"setCompassRange", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*127*/    {0x7f,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*128*/    {0x80,"getTareAsQuaternion", 16, "ffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*129*/    {0x81,"getTareAsRotationMatrix", 36, "fffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*130*/    {0x82,"getAccelerometerTrustValues", 8, "ff", 0, "", 0xfc000000, TSS_FW_20R10_COMPATIBLE},
/*131*/    {0x83,"getCompassTrustValues", 8, "ff", 0, "", 0xfc000000, TSS_FW_20R10_COMPATIBLE},
/*132*/    {0x84,"getCurrentUpdateRate", 4, "I", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*133*/    {0x85,"getCompassReferenceVector", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*134*/    {0x86,"getAccelerometerReferenceVector", 12, "fff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*135*/    {0x87,"getReferenceVectorMode", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*136*/    {0x88,"getCompassMultiReferenceVector", 12, "fff", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*137*/    {0x89,"getCompassMultiReferenceCheckVector", 12, "fff", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*138*/    {0x8a,"getAccelerometerMultiReferenceVector", 12, "fff", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*139*/    {0x8b,"getAccelerometerMultiReferenceCheckVector", 12, "fff", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*140*/    {0x8c,"getGyroscopeEnabledState", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*141*/    {0x8d,"getAccelerometerEnabledState", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*142*/    {0x8e,"getCompassEnabledState", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*143*/    {0x8f,"getAxisDirections", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*144*/    {0x90,"getOversampleRate", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*145*/    {0x91,"getRunningAveragePercent", 4, "f", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*146*/    {0x92,"getDesiredUpdateRate", 4, "I", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*147*/    {0x93,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*148*/    {0x94,"getAccelerometerRange", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*149*/    {0x95,"getMultiReferenceModePowerWeight", 4, "f", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*150*/    {0x96,"getMultiReferenceResolution", 2, "BB", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*151*/    {0x97,"getNumberOfMultiReferenceCells", 4, "I", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*152*/    {0x98,"getFilterMode", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*153*/    {0x99,"getRunningAverageMode", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*154*/    {0x9a,"getGyroscopeRange", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*155*/    {0x9b,"getCompassRange", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*156*/    {0x9c,"getEulerAngleDecompositionOrder", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*157*/    {0x9d,"getMagnetoresistiveThreshold", 16, "fIff", 0, "", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*158*/    {0x9e,"getAccelerometerResistanceThreshold", 8, "fI", 0, "", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*159*/    {0x9f,"getOffsetOrientationAsQuaternion", 16, "ffff", 0, "", 0xfc000000, TSS_FW_20R13_COMPATIBLE},
/*160*/    {0xa0,"setCompassCalibrationCoefficients", 0, "", 48, "ffffffffffff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*161*/    {0xa1,"setAccelerometerCalibrationCoefficients", 0, "", 48, "ffffffffffff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*162*/    {0xa2,"getCompassCalibrationCoefficients", 48, "ffffffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*163*/    {0xa3,"getAccelerometerCalibrationCoefficients", 48, "ffffffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*164*/    {0xa4,"getGyroscopeCalibrationCoefficients", 48, "ffffffffffff", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*165*/    {0xa5,"beginGyroscopeAutoCalibration", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*166*/    {0xa6,"setGyroscopeCalibrationCoefficients", 0, "", 48, "ffffffffffff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*167*/    {0xa7,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*168*/    {0xa8,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*169*/    {0xa9,"setCalibrationMode", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*170*/    {0xaa,"getCalibrationMode", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*171*/    {0xab,"setOrthoCalibrationDataPointFromCurrentOrientation", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*172*/    {0xac,"setOrthoCalibrationDataPointFromVector", 0, "", 14, "BBfff", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*173*/    {0xad,"getOrthoCalibrationDataPoint", 12, "fff", 2, "BB", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*174*/    {0xae,"performOrthoCalibration", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*175*/    {0xaf,"clearOrthoCalibrationData", 0, "", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*176*/    {0xb0,"setWirelessStreamingAutoFlushMode", 0, "", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*177*/    {0xb1,"getWirelessStreamingAutoFlushMode", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*178*/    {0xb2,"setWirelessStreamingManualFlushBitfield", 0, "", 2, "H", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*179*/    {0xb3,"getWirelessStreamingManualFlushBitfield", 2, "H", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*180*/    {0xb4,"getManualFlushSingle", 0, "", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*181*/    {0xb5,"getManualFlushBulk", 2, "H", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE}, // Will need to discuss about later.
/*182*/    {0xb6,"broadcastSynchronizationPulse", 0, "", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*183*/    {0xb7,"getReceptionBitfield", 2, "H", 0, "",0x2000000, TSS_FW_20R7_COMPATIBLE},
/*184*/    {0xb8,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*185*/    {0xb9,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*186*/    {0xba,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*187*/    {0xbb,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*188*/    {0xbc,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*189*/    {0xbd,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*190*/    {0xbe,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*191*/    {0xbf,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*192*/    {0xc0,"getWirelessPanID", 2, "H", 0, "", 0x32000000, TSS_FW_20R7_COMPATIBLE},
/*193*/    {0xc1,"setWirelessPanID", 0, "", 2, "H", 0x12000000, TSS_FW_20R7_COMPATIBLE},
/*194*/    {0xc2,"getWirelessChannel", 1, "B", 0, "", 0x32000000, TSS_FW_20R7_COMPATIBLE},
/*195*/    {0xc3,"setWirelessChannel", 0, "", 1, "B", 0x12000000, TSS_FW_20R7_COMPATIBLE},
/*196*/    {0xc4,"setLEDMode", 0, "", 1, "B", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*197*/    {0xc5,"commitWirelessSettings", 0, "", 0, "", 0x32000000, TSS_FW_20R7_COMPATIBLE},
/*198*/    {0xc6,"getWirelessAddress", 2, "H", 0, "", 0x32000000, TSS_FW_20R7_COMPATIBLE},
/*199*/    {0xc7,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*200*/    {0xc8,"getLEDMode", 1, "B", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*201*/    {0xc9,"getBatteryVoltage", 4, "f", 0, "", 0xf0000000, TSS_FW_20R7_COMPATIBLE},
/*202*/    {0xca,"getBatteryPercentRemaining", 1, "B", 0, "", 0xf0000000, TSS_FW_20R7_COMPATIBLE},
/*203*/    {0xcb,"getBatteryStatus", 1, "B", 0, "", 0xf0000000, TSS_FW_20R7_COMPATIBLE},
/*204*/    {0xcc,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*205*/    {0xcd,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*206*/    {0xce,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*207*/    {0xcf,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*208*/    {0xd0,"getSerialNumberAtLogicalID", 4, "I", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*209*/    {0xd1,"setSerialNumberAtLogicalID", 0, "", 5, "BI", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*210*/    {0xd2,"getWirelessChannelNoiseLevels", 16, "BBBBBBBBBBBBBBBB", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*211*/    {0xd3,"setWirelessRetries", 0, "", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*212*/    {0xd4,"getWirelessRetries", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*213*/    {0xd5,"getWirelessSlotsOpen", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*214*/    {0xd6,"getSignalStrength", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*215*/    {0xd7,"setWirelessHIDUpdateRate", 0, "", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*216*/    {0xd8,"getWirelessHIDUpdateRate", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*217*/    {0xd9,"setWirelessHIDAsynchronousMode", 0, "", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*218*/    {0xda,"getWirelessHIDAsynchronousMode", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*219*/    {0xdb,"setWirelessResponseHeaderBitfield", 0, "", 4, "I", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*220*/    {0xdc,"getWirelessResponseHeaderBitfield", 4, "I", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*221*/    {0xdd,"setWiredResponseHeaderBitfield", 0, "", 4, "I", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*222*/    {0xde,"getWiredResponseHeaderBitfield", 4, "I", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*223*/    {0xdf,"getFirmwareVersionString", 12, "ssssssssssss", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*224*/    {0xe0,"restoreFactorySettings", 0, "", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*225*/    {0xe1,"commitSettings", 0, "", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*226*/    {0xe2,"softwareReset", 0, "", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*227*/    {0xe3,"setSleepMode", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*228*/    {0xe4,"getSleepMode", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*229*/    {0xe5,"enterBootloaderMode", 0, "", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*230*/    {0xe6,"getHardwareVersionString", 32, "ssssssssssssssssssssssssssssssss", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*231*/    {0xe7,"setUARTBaudRate", 0, "", 4, "I", 0x8c000000, TSS_FW_20R7_COMPATIBLE},
/*232*/    {0xe8,"getUARTBaudRate", 4, "I", 0, "", 0x8c000000, TSS_FW_20R7_COMPATIBLE},
/*233*/    {0xe9,"setUSBMode", 0, "", 1, "B", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*234*/    {0xea,"getUSBMode", 1, "B", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*235*/    {0xeb,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*236*/    {0xec,"null", 0, "", 0, "",0, TSS_FW_NOT_COMPATIBLE},
/*237*/    {0xed,"getSerialNumber", 4, "I", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*238*/    {0xee,"setLEDColor", 0, "", 12, "fff", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*239*/    {0xef,"getLEDColor", 12, "fff", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*240*/    {0xf0,"setJoystickLogicalID", 0, "", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*241*/    {0xf1,"setMouseLogicalID", 0, "", 1, "B", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*242*/    {0xf2,"getJoystickLogicalID", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*243*/    {0xf3,"getMouseLogicalID", 1, "B", 0, "", 0x2000000, TSS_FW_20R7_COMPATIBLE},
/*244*/    {0xf4,"setControlMode", 0, "", 3, "BBB", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*245*/    {0xf5,"setControlData", 0, "", 7, "BBBf", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*246*/    {0xf6,"getControlMode", 1, "B", 2, "BB", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*247*/    {0xf7,"getControlData", 4, "f", 3, "BBB", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*248*/    {0xf8,"setButtonGyroDisableLength", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*249*/    {0xf9,"getButtonGyroDisableLength", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*250*/    {0xfa,"getButtonState", 1, "B", 0, "", 0xf4000000, TSS_FW_20R7_COMPATIBLE},
/*251*/    {0xfb,"setMouseAbsoluteRelativeMode", 0, "", 1, "B", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*252*/    {0xfc,"getMouseAbsoluteRelativeMode", 1, "B", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*253*/    {0xfd,"setJoystickAndMousePresentRemoved", 0, "", 2, "BB", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*254*/    {0xfe,"getJoystickAndMousePresentRemoved", 2, "B", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*255*/    {0xff,"null", 0, "", 0, "", 0xfe000000, TSS_FW_20R7_COMPATIBLE},
/*256*/    {0xf0,"setJoystickEnabled", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*257*/    {0xf1,"setMouseEnabled", 0, "", 1, "B", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*258*/    {0xf2,"getJoystickEnabled", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE},
/*259*/    {0xf3,"getMouseEnabled", 1, "B", 0, "", 0xfc000000, TSS_FW_20R7_COMPATIBLE}
};

void endian_swap_16(unsigned short * x)
{
    *x = (*x>>8) |
        (*x<<8);
}

void endian_swap_32(unsigned int * x)
{
    *x = (*x>>24) |
        ((*x<<8) & 0x00FF0000) |
        ((*x>>8) & 0x0000FF00) |
         (*x<<24);
}

