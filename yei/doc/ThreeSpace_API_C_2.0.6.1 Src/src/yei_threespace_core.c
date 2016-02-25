/********************************************//**
 * \file yei_threespace_core.c
 * \brief  ThreeSpace API 2.0
 * \author Chris George
 * \author Daniel Morrison
 * \copyright Copyright 1998-2014, YEI Corporation.
 *
 * The YEI 3-Space C API is released under the YEI 3-Space Open Source License, which allows for both
 * non-commercial use and commercial use with certain restrictions.
 *
 * For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL v.3, subject to the YEI 3-Space Open
 * Source Licensing Overview and Definitions.
 *
 * For Commercial Use, a YEI Commercial/Redistribution License is required, pursuant to the YEI 3-Space Open Source
 * Licensing Overview and Definitions. Commercial Use, for the purposes of this License, means the use, reproduction
 * and/or Distribution, either directly or indirectly, of the Covered Works or any portion thereof, or a Compilation,
 * Improvement, or Modification, for Pecuniary Gain. A YEI Commercial/Redistribution License may or may not require
 * payment, depending upon the intended use.
 *
 * Full details of the YEI 3-Space Open Source License can be found in license.txt
 * License also available online at http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include "yei_threespace_api.h"
#include "yei_threespace_core.h"
#include "yei_enum_ports.h"

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


//definitions

int compareFirmwareVersion(const char * version_string1, const char * version_string2){
    char day1[3]= {0};
    char month1[4]= {0};
    char year1[5]= {0};
    char minute1[3]= {0};
    char day2[3]= {0};
    char month2[4]= {0};
    char year2[5]= {0};
    char minute2[3]= {0};
    int day_int1;
    int month_int1;
    int year_int1;
    int hour_int1;
    int minute_int1;
    int day_int2;
    int month_int2;
    int year_int2;
    int hour_int2;
    int minute_int2;
    int i;
    const char * month_list[12]={   "Jan",
                                    "Feb",
                                    "Mar",
                                    "Apr",
                                    "May",
                                    "Jun",
                                    "Jul",
                                    "Aug",
                                    "Sep",
                                    "Oct",
                                    "Nov",
                                    "Dec"};
    //check year
    strncpy(year1,version_string1+5,4);
    strncpy(year2,version_string2+5,4);
    year_int1 = atoi(year1);
    year_int2 = atoi(year2);
    if(year_int1<year_int2){
        return -1;
    }
    else if(year_int1>year_int2){
        return 1;
    }
    //check month
    month_int1=0;
    month_int2=0;
    strncpy(month1,version_string1+2,3);
    for(i=0; i<12; i++){
        if(!strcmp(month1,month_list[i])){
            month_int1=i+1;
        }
    }
    strncpy(month2,version_string2+2,3);
    for(i=0; i<12; i++){
        if(!strcmp(month2,month_list[i])){
            month_int2=i+1;
        }
    }
    if(month_int1<month_int2){
        return -1;
    }
    else if(month_int1>month_int2){
        return 1;
    }
    //check day
    strncpy(day1,version_string1,2);
    strncpy(day2,version_string2,2);
    day_int1 = atoi(day1);
    day_int2 = atoi(day2);
    if(day_int1<day_int2){
        return -1;
    }
    else if(day_int1>day_int2){
        return 1;
    }
    //check hour
    hour_int1 = *(version_string1+9);
    hour_int2 = *(version_string2+9);
    if(hour_int1<hour_int2){
        return -1;
    }
    else if(hour_int1>hour_int2){
        return 1;
    }
    //check minute
    strncpy(minute1,version_string1+10,2);
    strncpy(minute2,version_string2+10,2);
    minute_int1 = atoi(minute1);
    minute_int2 = atoi(minute2);
    if(minute_int1<minute_int2){
        return -1;
    }
    else if(minute_int1>minute_int2){
        return 1;
    }
    return 0;
}

TSS_Firmware_Compatibility getFirmwareCompatibility(const char * version_string){
    int i;
    int fw_version_sting_len;
    fw_version_sting_len  = sizeof(TSS_Firmware_Version_String)/ sizeof(const char* const);
    for(i=fw_version_sting_len-1; i > 0; i--){
        if(i==0){
            return TSS_FW_NOT_COMPATIBLE;
        }
        if(compareFirmwareVersion(version_string, TSS_Firmware_Version_String[i]) > 0){
          return i;
        }
    }
    return TSS_FW_NOT_COMPATIBLE;
}

int tss_initThreeSpaceAPI(){
    QueryPerformanceFrequency(&timer_frequency);
    #ifdef _HEXDUMP
    f_hex_dump = fopen("ThreeSpace_API.log","w");
    setbuf(stderr,NULL);
    #endif  
    if(sensor_list_len){
        printf("Error, init should only be called once/n %d", sizeof(sensor_list));
        return 1;
    }
    //Do somthing?
    return 0;
}

TSS_Device_Id _createTSSWirelessSensor(TSS_Device_Id device, char logical_id){
    char firmware_version[13] ={0};
//    char min_firmware_version[] = {"17Mar2013A00"}; //absolute min version
//    char min_firmware_version[] = {"25Apr2013A00"};
    unsigned int tss_idx=0;
    unsigned int device_serial=0;
    unsigned int i;
    TSS_Firmware_Compatibility fw_version;
    TSS_Dongle * ts_dongle = NULL;
    tss_idx = (device-TSS_DONGLE_ID);
    if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
        ts_dongle = dongle_list[tss_idx];
    }
    if(!ts_dongle){
        return TSS_NO_DEVICE_ID;
    }
    for( i=0; i < wireless_retries ; i++){
        if(!faWriteReadDongle(ts_dongle,logical_id,
                          &simple_commands[TSS_GET_SERIAL_NUMBER],NULL, (char*)&device_serial,NULL)){
            break;
        }
    }
    if(device_serial){
        TSS_Sensor * new_sensor;
        //printf("Device serial: %X\n",device_serial);
        for( i=0; i < wireless_retries ; i++){
            if(!faWriteReadDongle(ts_dongle,logical_id,
                                  &simple_commands[TSS_GET_FIRMWARE_VERSION_STRING], NULL, firmware_version, NULL)){
                break;
            }
        }
        #ifdef DEBUG_PRINT
        printf("Firmware Version=%s\n",firmware_version);
        #endif
//        if( compareFirmwareVersion(firmware_version,min_firmware_version) < 0){
//            printf("incompatible version!!\n");
//            return TSS_NO_DEVICE_ID;
//        }
        fw_version = getFirmwareCompatibility(firmware_version);
        #ifdef DEBUG_PRINT
        printf("Creating a Wireless Sensor: %x\n",device_serial);
        #endif
        new_sensor = (TSS_Sensor*)malloc(sizeof(TSS_Sensor));
        if( new_sensor == 0){printf("ERROR: Out of memory\n"); return 0;}
        new_sensor->device_type = TSS_WL_W;
        new_sensor->fw_compatibility = fw_version;
        new_sensor->device_serial = device_serial;
        new_sensor->baudrate = 0;
        new_sensor->logical_id = logical_id;
        new_sensor->dongle = device;
        memset(new_sensor->protocol_header, 0, sizeof(new_sensor->protocol_header));
        new_sensor->stream_slots[0] = TSS_NULL;
        new_sensor->stream_slots[1] = TSS_NULL;
        new_sensor->stream_slots[2] = TSS_NULL;
        new_sensor->stream_slots[3] = TSS_NULL;
        new_sensor->stream_slots[4] = TSS_NULL;
        new_sensor->stream_slots[5] = TSS_NULL;
        new_sensor->stream_slots[6] = TSS_NULL;
        new_sensor->stream_slots[7] = TSS_NULL;
        memset(new_sensor->stream_parse_str, 0, sizeof(new_sensor->stream_parse_str));
        new_sensor->stream_byte_len = 0;
        new_sensor->stream_enabled = 0;
        new_sensor->last_stream_data = NULL;
        //Thread stuff
        new_sensor->is_active = 0;
        InitializeCriticalSection(&new_sensor->stream_lock);
        new_sensor->reader_event = NULL;
        new_sensor->writer_event = NULL;
        new_sensor->timestamp_mode = ts_dongle->timestamp_mode;
        new_sensor->new_data_event = CreateEvent(NULL, TRUE, FALSE, NULL);
//        new_sensor->reader_lock = ts_dongle->reader_lock;
//        new_sensor->stream_lock = ts_dongle->stream_lock;
//        new_sensor->reader_event = CreateEvent(NULL,FALSE,FALSE,NULL);
//        new_sensor->writer_event = CreateEvent(NULL,FALSE,FALSE,NULL);
//        InitializeCriticalSection(&new_sensor->reader_lock);
//        InitializeCriticalSection(&new_sensor->stream_lock);
        new_sensor->serial_port = NULL;
        new_sensor->callback = NULL;

        if( sensor_list_len < sensor_list_counter+1){
            //sensor_list = realloc(sensor_list,sizeof(TSS_Sensor *)*(sensor_list_counter+1));
            sensor_list_len= sensor_list_counter+1;
        }
        sensor_list[sensor_list_counter]= new_sensor;
        sensor_list_counter++;

        new_sensor->device_id = (TSS_WIRELESS_W_ID + (sensor_list_counter - 1));

        return new_sensor->device_id;
    }
    return TSS_NO_DEVICE_ID;
}

int tss_delThreeSpaceAPI(){
    unsigned int i;
    for(i=0; i < sensor_list_len; i++){
        if( sensor_list[i] != NULL){
            sensor_list[i]->is_active= 0;
            if(sensor_list[i]->serial_port){
                f9Write(sensor_list[i]->serial_port,&simple_commands[TSS_STOP_STREAMING],NULL);
                SetEvent(sensor_list[i]->reader_event);
            }
        }
    }
    for(i=0; i < dongle_list_len; i++){
        if( dongle_list[i] != NULL){
            dongle_list[i]->is_active= 0;
            f9Write(dongle_list[i]->serial_port,&simple_commands[TSS_STOP_STREAMING],NULL);
            SetEvent(dongle_list[i]->reader_event);
        }
    }
    for(i=0; i < dongle_list_len; i++){
        if( dongle_list[i] != NULL){
            unsigned short reception_bitfield=0;
            char k,j;

            #ifdef DEBUG_PRINT
            LARGE_INTEGER frequency;        // ticks per second
            LARGE_INTEGER t1, t2;           // ticks
            QueryPerformanceFrequency(&frequency);
            QueryPerformanceCounter(&t1);
            #endif

            //dongle_list[i]->is_active= 0;
            WaitForSingleObject(dongle_list[i]->reader_thread,510);
            //f7WriteRead(dongle_list[i]->serial_port, &simple_commands[TSS_STOP_STREAMING],NULL, NULL);
            f7WriteRead(dongle_list[i]->serial_port, &simple_commands[TSS_STOP_STREAMING],NULL, NULL);
            Sleep(60);
            PurgeComm(dongle_list[i]->serial_port,PURGE_RXCLEAR|PURGE_TXCLEAR);

            f7WriteRead(dongle_list[i]->serial_port,&simple_commands[TSS_GET_RECEPTION_BITFIELD],NULL, (char*)&reception_bitfield );
            #ifdef DEBUG_PRINT
            printf("reception_bitfield %X\n",reception_bitfield);
            #endif
            for( k = 0; k<15; k++){
                if( (reception_bitfield >> k) & 1){
                    #ifdef DEBUG_PRINT
                    printf("Logical_id: %d\n", k);
                    #endif
                    TSS_Error error;
                    for( j=0; j < 15 ; j++){
                        #ifdef DEBUG_PRINT
                        printf("tries:%d\n", j);
                        #endif
                        error = f8WriteRead(dongle_list[i]->serial_port, k, &simple_commands[TSS_STOP_STREAMING], NULL, NULL);
                        if(error == TSS_NO_ERROR){
                            #ifdef DEBUG_PRINT
                            printf("stopped sensor, %d\n",i);
                            #endif
                            break;
                        }
                    }
                }
            }
            #ifdef DEBUG_PRINT
            QueryPerformanceCounter(&t2);
            printf("Time=%f", (t2.QuadPart-t1.QuadPart)*1.0f/frequency.QuadPart);
            #endif

            CloseHandle(dongle_list[i]->serial_port);
            free(dongle_list[i]->last_stream_data);
            // Stop the threads TODO
            //Free the windows handles TODO overlapped
            #ifdef DEBUG_PRINT
            printf("closing handle dongle\n");
            #endif
            CloseHandle(dongle_list[i]->reader_event);
            CloseHandle(dongle_list[i]->writer_event);
            DeleteCriticalSection(&dongle_list[i]->reader_lock);
            DeleteCriticalSection(&dongle_list[i]->stream_lock);

            //Free memory
            free(dongle_list[i]);
            dongle_list[i] = NULL;
        }
    }
    for(i=0; i < sensor_list_len; i++){
        if( sensor_list[i] != NULL){
            if(sensor_list[i]->stream_enabled){
                if(sensor_list[i]->device_type != TSS_WL_W){
                    f7WriteRead(sensor_list[i]->serial_port, &simple_commands[TSS_STOP_STREAMING],NULL, NULL);
                }
//                int j;
//                TSS_Device_Id device = TSS_NO_DEVICE_ID;;
//                switch(sensor_list[i]->device_type){
//                case TSS_USB:
//                    device = (TSS_USB_ID+ i);
//                    break;
//                case TSS_WL:
//                    device = (TSS_WIRELESS_ID+ i);
//                    break;
//                case TSS_EM:
//                    device = (TSS_EMBEDDED_ID+ i);
//                    break;
//                case TSS_DL:
//                    device = (TSS_DATALOGGER_ID+ i);
//                    break;
//                case TSS_BT:
//                    device = (TSS_BLUETOOTH_ID+ i);
//                    break;
//                case TSS_WL_W:
//                case TSS_UNKNOWN:
//                case TSS_BTL:
//                case TSS_DNG:
//                    printf("Somehow a device without a known type was made?");
//                    device = TSS_NO_DEVICE_ID;
//                }
//                for(j=0 ; j < 5; j++){
//                    if(!tss_stopStreaming(device,NULL)){
//                        break;
//                    }
//                }
            }
            CloseHandle(sensor_list[i]->serial_port);
            //sensor_list[i]->is_active= 0;
            WaitForSingleObject(sensor_list[i]->reader_thread,550);
            free(sensor_list[i]->last_stream_data);
            // Stop the threads TODO
            //Free the windows handles TODO overlapped
            #ifdef DEBUG_PRINT
            printf("closing handle\n");
            #endif
            CloseHandle(sensor_list[i]->new_data_event);
            if(sensor_list[i]->device_type != TSS_WL_W){
                CloseHandle(sensor_list[i]->reader_event);
                CloseHandle(sensor_list[i]->writer_event);
                DeleteCriticalSection(&sensor_list[i]->reader_lock);
            }
            DeleteCriticalSection(&sensor_list[i]->stream_lock);

            //Free memory
            free(sensor_list[i]);
            sensor_list[i] = NULL;
        }
    }
    sensor_list_len = 0;
    sensor_list_counter = 0;

    dongle_list_len = 0;
    dongle_list_counter = 0;

    wireless_retries = TSS_DEFAULT_WIRLESSS_RETRIES;
    default_baud_rate = TSS_DEFAULT_BAUD_RATE;
    //free(sensor_list);
    //free(dongle_list);
    return 0;
}

TSS_Error _generateProtocolHeader(TSS_Protocol_Header_Setup protocol,
                            char * parser_str8){
    int index = 0;
    if(protocol.protocol_bits.success_failure){
        parser_str8[index]='B';
        index++;
    }
    if(protocol.protocol_bits.timestamp){
        parser_str8[index]='I';
        index++;
    }
    if(protocol.protocol_bits.command_echo){
        parser_str8[index]='B';
        index++;
    }
    if(protocol.protocol_bits.checksum){
        parser_str8[index]='B';
        index++;
    }
   if(protocol.protocol_bits.logical_id){
        parser_str8[index]='B';
        index++;
    }
    if(protocol.protocol_bits.serial_number){
        parser_str8[index]='I';
        index++;
    }
    if(protocol.protocol_bits.data_length){
        parser_str8[index]='B';
        index++;
    }
    return index;
}

int parseData( char* data, int len, const char* parser_str){
    int parser_index=0;
    int data_index=0;
    while(parser_str[parser_index]!= 0 ){
        switch(parser_str[parser_index]){
        case 'i': //int
        case 'I': //unsigned int
        case 'f': //float
            endian_swap_32((unsigned int *)&data[data_index]);
            data_index+=4;
            break;
        case 'h': //short
        case 'H': //unsigned short
            endian_swap_16((unsigned short *)&data[data_index]);
            data_index+=2;
            break;
        case 'b': //signed char
        case 'B': //unsigned char
        case '?': //bool
        case 's': //char[]
        case 'x': //pad byte
            //No swaps needed on single bytes
            data_index+=1;
            break;
        default:
            //should not get here
            printf("should not get here: %c\n",parser_str[parser_index]);
            break;
        }
        parser_index+=1;
    }

    if(data_index+1 == len){
        return 1;
    }
    return 0;
}

int calculateParseDataSize(const char* parser_str){
    int parser_index=0;
    int data_index=0;
    while(parser_str[parser_index]!= 0 ){
        switch(parser_str[parser_index]){
        case 'i': //int
        case 'I': //unsigned int
        case 'f': //float
            data_index+=4;
            break;
        case 'h': //short
        case 'H': //unsigned short
            data_index+=2;
            break;
        case 'b': //signed char
        case 'B': //unsigned char
        case '?': //bool
        case 's': //char[]
        case 'x': //pad byte
            //No swaps needed on single bytes
            data_index+=1;
            break;
        default:
            //should not get here
            printf("should not get here: %c\n",parser_str[parser_index]);
            break;
        }
        parser_index+=1;
    }
    return data_index;
}

unsigned char createChecksum(const char *command_bytes,
                             const unsigned int num_bytes){
    unsigned int chkSum = 0;
    unsigned int i;
    for (i = 0; i < num_bytes; i++){
        chkSum += command_bytes[i];
    }
    return (unsigned char)(chkSum % 256);
}

#ifdef _HEXDUMP
void _hexDump(const char * buffer, int buffer_size, const char * header_str){
    int i;
    int current_index = 0;
    char hex_buffer[766];
    //hex_buffer = (char *)malloc((buffer_size*3)+1);
    for(i = 0; i < buffer_size; i++){
        unsigned char buffer_byte = buffer[i];
        current_index+= sprintf( &hex_buffer[current_index], " %02hhX",(unsigned char) buffer_byte);
    }
    fprintf(f_hex_dump,"%s%s\n", header_str, hex_buffer);
    //free(hex_buffer)
}
#endif

TSS_Error f9Write(HANDLE com_handle,const TSS_Command * cmd_info, const char * input_data){
    unsigned char start_byte = 0xf9;
    unsigned int write_size = cmd_info->in_data_len+3; //3 ={Start byte, Command byte, Checksum byte}
    char write_array[256];
    DWORD num_bytes_written;
    OVERLAPPED osWrite= {0};
//    HANDLE overlapped_reader_event;
//  HANDLE overlapped_writer_event;
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    //write_array = malloc(write_size);
    if( write_array == 0){printf("ERROR: Out of memory\n"); return 0;}
    write_array[0]=start_byte;
    write_array[1]=cmd_info->command;
    if(cmd_info->in_data_len){
        memcpy(write_array+2,input_data, cmd_info->in_data_len);
        parseData(write_array+2,cmd_info->in_data_len, cmd_info->in_data_detail);
    }
    write_array[write_size-1] = createChecksum(write_array+1, write_size-2);
    #ifdef _HEXDUMP
    _hexDump(write_array,write_size, "<<");
    #endif
    if(!WriteFile(  com_handle,write_array,write_size,
                    &num_bytes_written,
                    &osWrite)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());
            #endif
            //free(write_array);
            CloseHandle(osWrite.hEvent);
            return TSS_ERROR_WRITE;
        }
    }
    if (!GetOverlappedResult(com_handle, &osWrite, &num_bytes_written, TRUE)){
        #ifdef DEBUG_PRINT
        printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());
        #endif
        CloseHandle(osWrite.hEvent);
        return TSS_ERROR_WRITE;
    }
    CloseHandle(osWrite.hEvent);
    return TSS_NO_ERROR;
}

TSS_Error f7WriteRead(HANDLE com_handle,const TSS_Command * cmd_info, const char * input_data, char * output_data){
    unsigned char start_byte = 0xf7;
    unsigned int write_size = cmd_info->in_data_len+3; //3 ={Start byte, Command byte, Checksum byte}
    unsigned int read_size =  cmd_info->rtn_data_len;
    char write_array[256];
    DWORD num_bytes_written;
    DWORD num_bytes_read;
    OVERLAPPED osWrite= {0};
    OVERLAPPED osReader= {0};
//    HANDLE overlapped_reader_event;
//  HANDLE overlapped_writer_event;
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    //write_array = malloc(write_size);
    if( write_array == 0){printf("ERROR: Out of memory\n"); return 0;}
    write_array[0]=start_byte;
    write_array[1]=cmd_info->command;
    if(cmd_info->in_data_len){
        memcpy(write_array+2,input_data, cmd_info->in_data_len);
        parseData(write_array+2,cmd_info->in_data_len, cmd_info->in_data_detail);
    }
    write_array[write_size-1] = createChecksum(write_array+1, write_size-2);
    #ifdef _HEXDUMP
    _hexDump(write_array,write_size, "<<");
    #endif
    if(!WriteFile(  com_handle,write_array,write_size,
                    &num_bytes_written,
                    &osWrite)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());
            #endif
            //free(write_array);
            CloseHandle(osWrite.hEvent);
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_WRITE;
        }
    }
    //free(write_array);
    if (!GetOverlappedResult(com_handle, &osWrite, &num_bytes_written, TRUE)){
        #ifdef DEBUG_PRINT
        printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());
        #endif
        CloseHandle(osWrite.hEvent);
        CloseHandle(osReader.hEvent);
        return TSS_ERROR_WRITE;
    }
    if(!cmd_info->rtn_data_len){
        CloseHandle(osWrite.hEvent);
        CloseHandle(osReader.hEvent);
        return TSS_NO_ERROR;
    }
    // Reading the response
    if(!ReadFile(   com_handle,
                    output_data,
                    read_size,
                    &num_bytes_read,
                    &osReader)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error reading from port\n");
            #endif
            CloseHandle(osWrite.hEvent);
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
    }
    if (!GetOverlappedResult(com_handle, &osReader, &num_bytes_read, TRUE)){
        #ifdef DEBUG_PRINT
         printf ("GetOverlappedResult Error reading from port\n");
        #endif
        CloseHandle(osWrite.hEvent);
        CloseHandle(osReader.hEvent);
        return TSS_ERROR_READ;
    }
    #ifdef _HEXDUMP
    _hexDump(output_data,num_bytes_read, ">>");
    #endif
    parseData(output_data,cmd_info->rtn_data_len, cmd_info->rtn_data_detail);
    CloseHandle(osWrite.hEvent);
    CloseHandle(osReader.hEvent);
    return TSS_NO_ERROR;
}

TSS_Error f8WriteRead(HANDLE com_handle, char logical_id, const TSS_Command * cmd_info, const char * input_data, char * output_data){
    unsigned char start_byte = 0xf8;
    unsigned int write_size = cmd_info->in_data_len + 4; //4 ={Start byte, Logical id, Command byte, Checksum byte}
    unsigned int read_size =  cmd_info->rtn_data_len;
    char write_array[256];
    char f8_header[3];
    DWORD num_bytes_written;
    DWORD num_bytes_read;
    OVERLAPPED osWrite = {0};
    OVERLAPPED osReader = {0};
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    //write_array = malloc(write_size);
    if(write_array == 0){printf("ERROR: Out of memory\n"); return 0;}
    write_array[0] = start_byte;
    write_array[1] = logical_id;
    write_array[2] = cmd_info->command;
    if(cmd_info->in_data_len){
        memcpy(write_array + 3, input_data, cmd_info->in_data_len);
        parseData(write_array + 3, cmd_info->in_data_len, cmd_info->in_data_detail);
    }
    write_array[write_size - 1] = createChecksum(write_array + 1, write_size - 2);
    #ifdef _HEXDUMP
    _hexDump(write_array,write_size, "<<");
    #endif
    if(!WriteFile(com_handle, write_array, write_size, &num_bytes_written, &osWrite)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());
            #endif
            //free(write_array);
            CloseHandle(osWrite.hEvent);
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_WRITE;
        }
    }
    //free(write_array);
    if (!GetOverlappedResult(com_handle, &osWrite, &num_bytes_written, TRUE)){
        #ifdef DEBUG_PRINT
        printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());
        #endif
        CloseHandle(osWrite.hEvent);
        CloseHandle(osReader.hEvent);
        return TSS_ERROR_WRITE;
    }
//    if(!cmd_info->rtn_data_len){
//        CloseHandle(osWrite.hEvent);
//        CloseHandle(osReader.hEvent);
//        return TSS_NO_ERROR;
//    }
    // Reading the response
    if(!ReadFile(com_handle, f8_header, 2, &num_bytes_read, &osReader)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error reading from port\n");
            #endif
            CloseHandle(osWrite.hEvent);
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
    }
    if(!GetOverlappedResult(com_handle, &osReader, &num_bytes_read, TRUE)){
        #ifdef DEBUG_PRINT
         printf ("GetOverlappedResult Error reading from port\n");
        #endif
        CloseHandle(osWrite.hEvent);
        CloseHandle(osReader.hEvent);
        return TSS_ERROR_READ;
    }
    if(f8_header[1] != logical_id){
        PurgeComm(com_handle,PURGE_RXCLEAR|PURGE_TXCLEAR);
        return TSS_ERROR_READ;
    }

    if(f8_header[0] == 0){
        if(!ReadFile(com_handle, f8_header + 2, 1, &num_bytes_read, &osReader)){
            if (GetLastError() != ERROR_IO_PENDING){
                #ifdef DEBUG_PRINT
                printf ("Error reading from port\n");
                #endif
                CloseHandle(osWrite.hEvent);
                CloseHandle(osReader.hEvent);
                return TSS_ERROR_READ;
            }
        }
        if(!GetOverlappedResult(com_handle, &osReader, &num_bytes_read, TRUE)){
            #ifdef DEBUG_PRINT
             printf ("GetOverlappedResult Error reading from port\n");
            #endif
            CloseHandle(osWrite.hEvent);
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
    }
    else{
        return TSS_ERROR_COMMAND_FAIL;
    }
    if(!ReadFile(com_handle, output_data, read_size, &num_bytes_read, &osReader)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error reading from port\n");
            #endif
            CloseHandle(osWrite.hEvent);
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
    }
    if(!GetOverlappedResult(com_handle, &osReader, &num_bytes_read, TRUE)){
        #ifdef DEBUG_PRINT
         printf ("GetOverlappedResult Error reading from port\n");
        #endif
        CloseHandle(osWrite.hEvent);
        CloseHandle(osReader.hEvent);
        return TSS_ERROR_READ;
    }
    parseData(output_data, cmd_info->rtn_data_len, cmd_info->rtn_data_detail);
    CloseHandle(osWrite.hEvent);
    CloseHandle(osReader.hEvent);
    return TSS_NO_ERROR;
}

TSS_Error f9WriteRead(TSS_Sensor * sensor, const TSS_Command * cmd_info,
                const char * input_data, char * output_data,
                unsigned int * timestamp){
    unsigned char start_byte = 0xf9;
    unsigned int write_size = cmd_info->in_data_len+3; //3 ={Start byte, Command byte, Checksum byte}
//    unsigned int read_size =  cmd_info->rtn_data_len;
    char * write_array;
    DWORD num_bytes_written;
//    DWORD num_bytes_read;
    //TSS_Header_69 header_packet;
    OVERLAPPED osWrite= {0};
    DWORD dwWaitResult;
    write_array = malloc(write_size);
    if( write_array == 0){printf("ERROR: Out of memory\n"); return TSS_ERROR_MEMORY;}
    write_array[0]=start_byte;
    write_array[1]=cmd_info->command;
    if(cmd_info->in_data_len){
        memcpy(write_array+2,input_data, cmd_info->in_data_len);
        parseData(write_array+2,cmd_info->in_data_len, cmd_info->in_data_detail);
    }
    write_array[write_size-1] = createChecksum(write_array+1, write_size-2);
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    EnterCriticalSection(&sensor->reader_lock);
    #ifdef _HEXDUMP
    _hexDump(write_array,write_size, "<<");
    #endif
    if(!WriteFile(  sensor->serial_port,write_array,write_size,
                    &num_bytes_written,
                    &osWrite)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error writing to port\n");
            #endif
            free(write_array);
            CloseHandle(osWrite.hEvent);
            LeaveCriticalSection(&sensor->reader_lock);
            return TSS_ERROR_WRITE;
        }
    }
    #ifdef DEBUG_PRINT
    printf("wrote data\n");
    #endif
    free(write_array);
    if (!GetOverlappedResult(sensor->serial_port, &osWrite, &num_bytes_written, TRUE)){
        #ifdef DEBUG_PRINT
        printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());
        #endif
        CloseHandle(osWrite.hEvent);
        LeaveCriticalSection(&sensor->reader_lock);
        return TSS_ERROR_WRITE;
    }
    CloseHandle(osWrite.hEvent);
    dwWaitResult=WaitForSingleObject(sensor->writer_event,1000);
    if(dwWaitResult ==WAIT_OBJECT_0){
        TSS_Header_71 * header_packet = (TSS_Header_71 *)sensor->last_header_data;
//        printf("header_packet %u, %X, %u\n",header_packet->success_failure,
//                                            header_packet->command_echo,
//                                            header_packet->data_length);
        if(timestamp){
            *timestamp = header_packet->timestamp;
        }
        if(header_packet->success_failure){
            SetEvent(sensor->reader_event);
            LeaveCriticalSection(&sensor->reader_lock);
            return TSS_ERROR_COMMAND_FAIL;
        }
        if(cmd_info->rtn_data_len){
            memcpy(output_data,sensor->last_out_data, cmd_info->rtn_data_len);
        }
        SetEvent(sensor->reader_event);
        LeaveCriticalSection(&sensor->reader_lock);
        parseData(output_data,cmd_info->rtn_data_len, cmd_info->rtn_data_detail);
        return TSS_NO_ERROR;
    }
    // An error occurred
    printf("Wait error f9WriteRead(%d)\n", (unsigned int)GetLastError());
    LeaveCriticalSection(&sensor->reader_lock);
    return TSS_ERROR_TIMEOUT;
}

TSS_Error f9WriteReadDongle(TSS_Dongle * dongle, const TSS_Command * cmd_info,
                const char * input_data, char * output_data,
                unsigned int * timestamp){
    unsigned char start_byte = 0xf9;
    unsigned int write_size = cmd_info->in_data_len+3; //3 ={Start byte, Command byte, Checksum byte}
//    unsigned int read_size =  cmd_info->rtn_data_len;
    char write_array[256];
    DWORD num_bytes_written;
//    DWORD num_bytes_read;
    //TSS_Header_85 header_packet;
    OVERLAPPED osWrite= {0};
    //write_array = malloc(write_size);
    //if( write_array == 0){printf("ERROR: Out of memory\n"); return TSS_ERROR_MEMORY;}
    DWORD dwWaitResult;
    write_array[0]=start_byte;
    write_array[1]=cmd_info->command;
    if(cmd_info->in_data_len){
        memcpy(write_array+2,input_data, cmd_info->in_data_len);
        parseData(write_array+2,cmd_info->in_data_len, cmd_info->in_data_detail);
    }
    write_array[write_size-1] = createChecksum(write_array+1, write_size-2);
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    EnterCriticalSection(&dongle->reader_lock);
    #ifdef _HEXDUMP
    _hexDump(write_array,write_size, "<<");
    #endif
    if(!WriteFile(  dongle->serial_port,write_array,write_size,
                    &num_bytes_written,
                    &osWrite)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error writing to port\n");
            #endif
            //free(write_array);
            CloseHandle(osWrite.hEvent);
            LeaveCriticalSection(&dongle->reader_lock);
            return TSS_ERROR_WRITE;
        }
    }
    //free(write_array);
    if (!GetOverlappedResult(dongle->serial_port, &osWrite, &num_bytes_written, TRUE)){
        #ifdef DEBUG_PRINT
        printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());;
        #endif
        CloseHandle(osWrite.hEvent);
        LeaveCriticalSection(&dongle->reader_lock);
        return TSS_ERROR_WRITE;
    }
    CloseHandle(osWrite.hEvent);

    dwWaitResult=WaitForSingleObject(dongle->writer_event,1000);
    if(dwWaitResult ==WAIT_OBJECT_0){
        TSS_Header_87 * header_packet = (TSS_Header_87 *)dongle->last_header_data;
        if(timestamp){
            *timestamp = header_packet->timestamp;
        }
//        printf("header_packet %u, %u, %X, %u\n",header_packet->success_failure,
//                            header_packet->logical_id,
//                              header_packet->command_echo,
//                              header_packet->data_length);
       if(header_packet->success_failure){
            SetEvent(dongle->reader_event);
            LeaveCriticalSection(&dongle->reader_lock);
            return TSS_ERROR_COMMAND_FAIL;
        }
        if(cmd_info->rtn_data_len){
            memcpy(output_data,dongle->last_out_data,cmd_info->rtn_data_len);
        }
        SetEvent(dongle->reader_event);
        LeaveCriticalSection(&dongle->reader_lock);
        parseData(output_data,cmd_info->rtn_data_len, cmd_info->rtn_data_detail);
        return TSS_NO_ERROR;
    }
    printf("Wait error f9WriteRead(%d)\n", (unsigned int)GetLastError());
    LeaveCriticalSection(&dongle->reader_lock);
    return TSS_ERROR_TIMEOUT;
}

TSS_Error faWriteReadDongle(TSS_Dongle * dongle, char logical_id, const TSS_Command * cmd_info,
                const char * input_data, char * output_data,
                unsigned int * timestamp){
    unsigned char start_byte = 0xfa;
    unsigned int write_size = cmd_info->in_data_len+4; //3 ={Start byte, logical_id, Command byte, Checksum byte}
//    unsigned int read_size =  cmd_info->rtn_data_len;
    char write_array[256];
    DWORD num_bytes_written;
//    DWORD num_bytes_read;
    //TSS_Header_85 header_packet;
    OVERLAPPED osWrite= {0};
    DWORD dwWaitResult;
    //write_array = malloc(write_size);
    //if( write_array == 0){printf("ERROR: Out of memory\n"); return TSS_ERROR_MEMORY;}
    write_array[0]=start_byte;
    write_array[1]=logical_id;
    write_array[2]=cmd_info->command;
    if(cmd_info->in_data_len){
        memcpy(write_array+3,input_data, cmd_info->in_data_len);
        parseData(write_array+3,cmd_info->in_data_len, cmd_info->in_data_detail);
    }
    write_array[write_size-1] = createChecksum(write_array+1, write_size-2);
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    EnterCriticalSection(&dongle->reader_lock);
    #ifdef _HEXDUMP
    _hexDump(write_array,write_size, "<<");
    #endif
    if(!WriteFile(  dongle->serial_port,write_array,write_size,
                    &num_bytes_written,
                    &osWrite)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error writing to port\n");
            #endif
            //free(write_array);
            CloseHandle(osWrite.hEvent);
            LeaveCriticalSection(&dongle->reader_lock);
            return TSS_ERROR_WRITE;
        }
    }
    //free(write_array);
    if (!GetOverlappedResult(dongle->serial_port, &osWrite, &num_bytes_written, TRUE)){
        #ifdef DEBUG_PRINT
        printf ("Error writing to port(%d)\n", (unsigned int)GetLastError());;
        #endif
        CloseHandle(osWrite.hEvent);
        LeaveCriticalSection(&dongle->reader_lock);
        return TSS_ERROR_WRITE;
    }
    CloseHandle(osWrite.hEvent);

    dwWaitResult=WaitForSingleObject(dongle->writer_event,1000);
    if(dwWaitResult ==WAIT_OBJECT_0){
        TSS_Header_87 * header_packet = (TSS_Header_87 *)dongle->last_header_data;
        if(timestamp){
            *timestamp = header_packet->timestamp;
        }
//        printf("header_packet %u, %u, %X, %u\n",header_packet->success_failure,
//                            header_packet->logical_id,
//                              header_packet->command_echo,
//                              header_packet->data_length);
       if(header_packet->success_failure){
            SetEvent(dongle->reader_event);
            LeaveCriticalSection(&dongle->reader_lock);
            return TSS_ERROR_COMMAND_FAIL;
        }
        if(cmd_info->rtn_data_len){
            memcpy(output_data,dongle->last_out_data,cmd_info->rtn_data_len);
        }
        SetEvent(dongle->reader_event);
        LeaveCriticalSection(&dongle->reader_lock);
        parseData(output_data,cmd_info->rtn_data_len, cmd_info->rtn_data_detail);
        return TSS_NO_ERROR;
    }
    printf("Wait error faWriteRead(%d)\n", (unsigned int)GetLastError());
    LeaveCriticalSection(&dongle->reader_lock);
    return TSS_ERROR_TIMEOUT;
}

TSS_Error writeRead(TSS_Device_Id device, const TSS_Command * cmd_info,   //TODO add typecheck here
              const char * input_data, char * output_data,
              unsigned int *timestamp){
//    printf("serial device: %X\n",device);
    unsigned int tss_idx;
    unsigned int tss_d_idx;
    unsigned int i;
    int result = TSS_INVALID_ID;
    if(!(device & cmd_info->compatibility_mask)){
        printf("Invalid command for that sensor type!!!\n");
        return TSS_INVALID_COMMAND;
    }
    #ifdef DEBUG_PRINT
    printf("device & TSS_ALL_SENSORS_ID = %x\n", (device & TSS_ALL_SENSORS_ID));
    printf("device  = %x\n", (device ));
    #endif
    switch(device & TSS_ALL_SENSORS_ID){
    case(TSS_DONGLE_ID):
        #ifdef DEBUG_PRINT
        printf("dong write.....\n");
        #endif
        tss_idx = (device-TSS_DONGLE_ID);
        if(tss_idx < dongle_list_len && dongle_list[tss_idx]){
            if( dongle_list[tss_idx]->fw_compatibility < cmd_info->fw_compatibility){
                return TSS_ERROR_FIRMWARE_INCOMPATIBLE;
            }
            #ifdef DEBUG_PRINT
            printf("Dongle description_str: %s\n",cmd_info->description_str);
            #endif
            return f9WriteReadDongle(dongle_list[tss_idx],
                                cmd_info, input_data, output_data, timestamp);
        }
        break;
    case(TSS_WIRELESS_W_ID):
        tss_idx = (device-TSS_WIRELESS_W_ID);
        if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
            tss_d_idx = (sensor_list[tss_idx]->dongle-TSS_DONGLE_ID);
            if(tss_d_idx < dongle_list_len && dongle_list[tss_d_idx]){
                if( dongle_list[tss_d_idx]->fw_compatibility < cmd_info->fw_compatibility){
                    return TSS_ERROR_FIRMWARE_INCOMPATIBLE;
                }
                if( sensor_list[tss_idx]->fw_compatibility < cmd_info->fw_compatibility){
                    return TSS_ERROR_FIRMWARE_INCOMPATIBLE;
                }
                #ifdef DEBUG_PRINT
                printf("Wireless description_str: %s\n",cmd_info->description_str);
                #endif
                for(i=0 ; i < wireless_retries ; i++){
                    result= faWriteReadDongle( dongle_list[tss_d_idx],
                                            sensor_list[tss_idx]->logical_id,
                                            cmd_info, input_data, output_data,
                                            timestamp);
                    if(result == TSS_NO_ERROR){
                        break;
                    }
                }
                return result;
            }
        }
        break;
    case(TSS_USB_ID):
    case(TSS_EMBEDDED_ID):
    case(TSS_WIRELESS_ID):
    case(TSS_DATALOGGER_ID):
    case(TSS_BLUETOOTH_ID):
        tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    //    printf("serial idx: %X\n",tss_idx);
        if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
            if( sensor_list[tss_idx]->fw_compatibility < cmd_info->fw_compatibility){
                return TSS_ERROR_FIRMWARE_INCOMPATIBLE;
            }
            #ifdef DEBUG_PRINT
            printf("description_str: %s\n",cmd_info->description_str);
            #endif
            return f9WriteRead(sensor_list[tss_idx],
                               cmd_info, input_data, output_data, timestamp);
        }
    }
    return TSS_INVALID_ID;
}

TSS_Error _parseWiredStreamData(TSS_Device_Id device, char * output_data,
                          unsigned int output_data_len,  unsigned int * timestamp){
    unsigned int tss_idx = ~(~device|TSS_NO_DONGLE_ID);
    if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
        TSS_Sensor * sensor = sensor_list[tss_idx];
        DWORD num_bytes_read;
        TSS_Header_69 header_packet;
        OVERLAPPED osReader= {0};

        if(!sensor->stream_byte_len){
            printf("if(!sensor->stream_byte_len){\n");
            return TSS_ERROR_STREAM_CONFIG;
        }

        if(output_data_len != sensor->stream_byte_len){
            printf("if(output_data_len != sensor->stream_byte_len){\n");
            return TSS_ERROR_PARAMETER;
        }

        osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        // Reading the response
        if(!ReadFile(   sensor->serial_port,
                        &header_packet,
                        sizeof(header_packet),
                        &num_bytes_read,
                        &osReader)){
                if (GetLastError() != ERROR_IO_PENDING){
                    printf ("Error reading from port Header packet\n");
                    CloseHandle(osReader.hEvent);
                    return TSS_ERROR_READ;
                }
        }
        if (!GetOverlappedResult(sensor->serial_port, &osReader, &num_bytes_read, TRUE)){
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
        if(num_bytes_read !=sizeof(header_packet)){
            printf("_parseWiredStreamDatanum_bytes_read !=sizeof(header_packet)){\n");
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
        if(sensor->stream_byte_len != header_packet.data_length){
            printf("if(sensor->stream_byte_len != header_packet.data_length){\n");
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
        if(!header_packet.success_failure){
            // Reading the response
            if(!ReadFile(   sensor->serial_port,
                            output_data,
                            sensor->stream_byte_len,
                            &num_bytes_read,
                            &osReader)){
                if (GetLastError() != ERROR_IO_PENDING){
                    #ifdef DEBUG_PRINT
                    printf ("Error reading from port\n");
                    #endif
                    CloseHandle(osReader.hEvent);
                    return TSS_ERROR_COMMAND_FAIL;
                }
            }
            if (!GetOverlappedResult(sensor->serial_port, &osReader, &num_bytes_read, TRUE)){
                CloseHandle(osReader.hEvent);
                return TSS_ERROR_READ;
            }
            parseData(output_data,sensor->stream_byte_len, sensor->stream_parse_str);
            CloseHandle(osReader.hEvent);
            return TSS_NO_ERROR;
        }
        CloseHandle(osReader.hEvent);
    }
    return TSS_ERROR_READ;
}

unsigned int __stdcall _serialReadLoop(void * vsensor){
    TSS_Sensor * sensor = (TSS_Sensor *)vsensor;
    #ifdef DEBUG_PRINT
    printf("run loop\n");
    #endif
    while(sensor->is_active){
        int error = _parseWiredStreamDataThreaded(sensor);
        if(error == TSS_ERROR_READ){
            //printf("Brreaking \n");
            //break;
        }
    }
    #ifdef DEBUG_PRINT
    printf("Finished loop\n");
    #endif
    return 0;
}

TSS_Error _parseWiredStreamDataThreaded(TSS_Sensor * sensor){
    DWORD num_bytes_read;
    TSS_Header_69 header_packet69;
    TSS_Header_71 header_packet71;
    void * header_data;
    char success_failure;
    unsigned int timestamp=0;
    unsigned char command_echo;
    //unsigned char logical_id;
    unsigned char data_length;
    int header_size;
    OVERLAPPED osReader= {0};
    osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    // Reading the response
    if(sensor->timestamp_mode == TSS_TIMESTAMP_SENSOR){
        header_size= sizeof(header_packet71);
        header_data = &header_packet71;
    }
    else{
        header_size= sizeof(header_packet69);
        header_data = &header_packet69;
    }
    if(!ReadFile(   sensor->serial_port,
                    header_data,
                    header_size,
                    &num_bytes_read,
                    &osReader)){
        if (GetLastError() != ERROR_IO_PENDING){
            #ifdef DEBUG_PRINT
            printf ("Error reading from port\n");
            #endif
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
    }
    if (!GetOverlappedResult(sensor->serial_port, &osReader, &num_bytes_read, TRUE)){
        CloseHandle(osReader.hEvent);
        #ifdef DEBUG_PRINT
        printf ("Error reading from port Serial operation failed\n");
        #endif
        return TSS_ERROR_READ;
    }
    if(num_bytes_read !=header_size){
        CloseHandle(osReader.hEvent);
        return TSS_ERROR_READ;
    }
    #ifdef _HEXDUMP
    _hexDump(header_data, num_bytes_read, ">>");
    #endif
    if(sensor->timestamp_mode == TSS_TIMESTAMP_SENSOR){
        success_failure= header_packet71.success_failure;
        timestamp= header_packet71.timestamp;
        endian_swap_32(&timestamp);
        command_echo= header_packet71.command_echo;
        data_length= header_packet71.data_length;
        #ifdef DEBUG_PRINT
        if(timestamp == l_timestamp){
            printf("DUP Packet!!!!!!\n");
        }
        l_timestamp = timestamp;
        #endif
    }
    else{
        success_failure= header_packet69.success_failure;
        command_echo= header_packet69.command_echo;
        data_length= header_packet69.data_length;
    }
    if(sensor->timestamp_mode == TSS_TIMESTAMP_SYSTEM){
        LARGE_INTEGER timer_counter;
        QueryPerformanceCounter(&timer_counter);
        timestamp = (unsigned int)(timer_counter.QuadPart /(timer_frequency.QuadPart /1000000));
    }

    if(command_echo == 0xff){
        char * stream_data = sensor->last_stream_data;
        if(sensor->stream_byte_len != data_length){
            stream_data = sensor->last_out_data;
        }
        EnterCriticalSection(&sensor->stream_lock);
        // Reading the response
        if(!ReadFile(   sensor->serial_port,
                        stream_data,
                        data_length,
                        &num_bytes_read,
                        &osReader)){
            if (GetLastError() != ERROR_IO_PENDING){
                printf ("Error reading from port ff\n");
                CloseHandle(osReader.hEvent);
                LeaveCriticalSection(&sensor->stream_lock);
                return TSS_ERROR_READ;
            }
        }
        if (!GetOverlappedResult(sensor->serial_port, &osReader, &num_bytes_read, TRUE)){
            CloseHandle(osReader.hEvent);
            LeaveCriticalSection(&sensor->stream_lock);
            return TSS_ERROR_READ;
        }
        #ifdef _HEXDUMP
        _hexDump(stream_data, num_bytes_read, ">>");
        #endif
        parseData(sensor->last_stream_data,data_length, sensor->stream_parse_str);
        sensor->last_stream_timestamp = timestamp;
        if(sensor->callback){
            sensor->callback(sensor->device_id,sensor->last_stream_data,data_length, &timestamp);
        }
        SetEvent(sensor->new_data_event);
        LeaveCriticalSection(&sensor->stream_lock);

        CloseHandle(osReader.hEvent);
        return TSS_NO_ERROR;
    }
    else{
        header_packet71.success_failure=success_failure;
        header_packet71.timestamp=timestamp;
        header_packet71.command_echo=command_echo;
        header_packet71.data_length= data_length;
        sensor->last_header_data = &header_packet71;
        if(data_length){
            if(!ReadFile(   sensor->serial_port,
                            sensor->last_out_data,
                            data_length,
                            &num_bytes_read,
                            &osReader)){
                if (GetLastError() != ERROR_IO_PENDING){
                    printf ("Error reading from data from port\n");
                    CloseHandle(osReader.hEvent);
                    return TSS_ERROR_READ;
                }
            }
            if (!GetOverlappedResult(sensor->serial_port, &osReader, &num_bytes_read, TRUE)){
                CloseHandle(osReader.hEvent);
                return TSS_ERROR_READ;
            }
            #ifdef _HEXDUMP
             _hexDump(sensor->last_out_data,num_bytes_read, ">>");
            #endif
        }

        SetEvent(sensor->writer_event);
        WaitForSingleObject(sensor->reader_event, 1000); // check later for unhandled packet
        CloseHandle(osReader.hEvent);
        return TSS_NO_ERROR;
    }
    return TSS_ERROR_READ;
}

unsigned int __stdcall _serialReadLoopDongle(void * vdongle){
    TSS_Dongle * dongle = (TSS_Dongle *)vdongle;
    #ifdef DEBUG_PRINT
     printf("run loop\n");
     #endif
    while(dongle->is_active){
        int error = _parseWiredStreamDataThreadedDongle(dongle);
        if(error == TSS_ERROR_READ){
            //printf("Brreaking \n");
            //break;
        }
    }
    #ifdef DEBUG_PRINT
    printf("Finished loop\n");
    #endif
    return 0;
}

TSS_Error _parseWiredStreamDataThreadedDongle(TSS_Dongle * dongle){
    DWORD num_bytes_read;
    TSS_Header_85 header_packet85;
    TSS_Header_87 header_packet87;
    void * header_data;
    char success_failure;
    unsigned int timestamp=0;
    unsigned char command_echo;
    unsigned char logical_id;
    unsigned char data_length;
    int header_size;
    OVERLAPPED osReader= {0};
    osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    // Reading the response
    if(dongle->timestamp_mode == TSS_TIMESTAMP_SENSOR){
        header_size= sizeof(header_packet87);
        header_data = &header_packet87;
    }
    else{
        header_size= sizeof(header_packet85);
        header_data = &header_packet85;
    }
    if(!ReadFile(   dongle->serial_port,
                    header_data,
                    header_size,
                    &num_bytes_read,
                    &osReader)){
        if (GetLastError() != ERROR_IO_PENDING){
            printf ("Error reading from port\n");
            CloseHandle(osReader.hEvent);
            return TSS_ERROR_READ;
        }
    }
    if (!GetOverlappedResult(dongle->serial_port, &osReader, &num_bytes_read, TRUE)){
        CloseHandle(osReader.hEvent);
        #ifdef DEBUG_PRINT
        printf ("Error reading from port Serial operation failed\n");
        #endif
        return TSS_ERROR_READ;
    }
    if(num_bytes_read !=header_size){
        CloseHandle(osReader.hEvent);
        return TSS_ERROR_READ;
    }
    #ifdef _HEXDUMP
    _hexDump(header_data,num_bytes_read, ">>");
    #endif

    if(dongle->timestamp_mode == TSS_TIMESTAMP_SENSOR){
        success_failure= header_packet87.success_failure;
        timestamp= header_packet87.timestamp;
        endian_swap_32(&timestamp);
        command_echo= header_packet87.command_echo;
        logical_id= header_packet87.logical_id;
        data_length= header_packet87.data_length;
        #ifdef DEBUG_PRINT
        if(timestamp == l_timestamp){
            printf("DUP Packet!!!!!!\n");
        }
        l_timestamp = timestamp;
        #endif
    }
    else{
        success_failure= header_packet85.success_failure;
        command_echo= header_packet85.command_echo;
        logical_id= header_packet85.logical_id;
        data_length= header_packet85.data_length;
    }
    if(dongle->timestamp_mode == TSS_TIMESTAMP_SYSTEM){
        LARGE_INTEGER timer_counter;
        QueryPerformanceCounter(&timer_counter);
        timestamp = (unsigned int)(timer_counter.QuadPart /(timer_frequency.QuadPart /1000000));
    }
    if(command_echo == 0xff){
        TSS_Sensor * ts_sensor;
        char * stream_data;
        unsigned int tss_idx;
        TSS_Device_Id device = dongle->w_sensors[logical_id];

        tss_idx = (device-TSS_WIRELESS_W_ID);
        if(tss_idx < sensor_list_len && sensor_list[tss_idx]){
            ts_sensor = sensor_list[tss_idx];
            if(ts_sensor->stream_byte_len != data_length){
                stream_data = ts_sensor->last_out_data;
                printf("if(ts_sensor->stream_byte_len %u != data_length %u){\n",ts_sensor->stream_byte_len , data_length);
            }
            else{
                stream_data = ts_sensor->last_stream_data;
            }
            EnterCriticalSection(&ts_sensor->stream_lock);
            // Reading the response
            if(!ReadFile(   dongle->serial_port,
                            stream_data,
                            data_length,
                            &num_bytes_read,
                            &osReader)){
                if (GetLastError() != ERROR_IO_PENDING){
                    printf ("Error reading from port\n");
                    CloseHandle(osReader.hEvent);
                    LeaveCriticalSection(&ts_sensor->stream_lock);
                    return TSS_ERROR_READ;
                }
            }
            if (!GetOverlappedResult(dongle->serial_port, &osReader, &num_bytes_read, TRUE)){
                CloseHandle(osReader.hEvent);
                LeaveCriticalSection(&ts_sensor->stream_lock);
                return TSS_ERROR_READ;
            }
            #ifdef _HEXDUMP
             _hexDump(stream_data,num_bytes_read, ">>");
            #endif
            parseData(ts_sensor->last_stream_data,data_length, ts_sensor->stream_parse_str);
            ts_sensor->last_stream_timestamp = timestamp;
            if(ts_sensor->callback){
                ts_sensor->callback(ts_sensor->device_id,ts_sensor->last_stream_data,data_length, &timestamp);
            }
            SetEvent(ts_sensor->new_data_event);
            LeaveCriticalSection(&ts_sensor->stream_lock);
            CloseHandle(osReader.hEvent);
            ts_sensor->record_count++;
            return TSS_NO_ERROR;
        }
        else{
            char safety_buffer[256];
            if(!ReadFile(   dongle->serial_port,
                            safety_buffer,
                            data_length,
                            &num_bytes_read,
                            &osReader)){
                if (GetLastError() != ERROR_IO_PENDING){
                    printf ("Error reading from port\n");
                    CloseHandle(osReader.hEvent);
                    return TSS_ERROR_READ;
                }
            }
            if (!GetOverlappedResult(dongle->serial_port, &osReader, &num_bytes_read, TRUE)){
                CloseHandle(osReader.hEvent);
                return TSS_ERROR_READ;
            }
            #ifdef _HEXDUMP
             _hexDump(safety_buffer,num_bytes_read, ">>");
            #endif
            CloseHandle(osReader.hEvent);

        }
    }
    else{
        #ifdef DEBUG_PRINT
        printf("command_echo %X\n", command_echo);
        #endif
        header_packet87.success_failure=success_failure;
        header_packet87.timestamp=timestamp;
        header_packet87.command_echo=command_echo;
        header_packet87.logical_id=logical_id;
        header_packet87.data_length= data_length;
        dongle->last_header_data = &header_packet87;
        #ifdef DEBUG_PRINT
        if(logical_id != 0xfe){
            printf("Wireless logical_id: %d\n",logical_id);
        }
        #endif
        if(data_length){
            if(!ReadFile(   dongle->serial_port,
                            dongle->last_out_data,
                            data_length,
                            &num_bytes_read,
                            &osReader)){
                if (GetLastError() != ERROR_IO_PENDING){
                    printf ("Error reading from port event(%d)\n", (unsigned int)GetLastError());
                    CloseHandle(osReader.hEvent);
                    return TSS_ERROR_READ;
                }
            }
            if (!GetOverlappedResult(dongle->serial_port, &osReader, &num_bytes_read, TRUE)){
                CloseHandle(osReader.hEvent);
                return TSS_ERROR_READ;
            }
            #ifdef _HEXDUMP
             _hexDump(dongle->last_out_data,num_bytes_read, ">>");
            #endif

        }
        SetEvent(dongle->writer_event);
        WaitForSingleObject(dongle->reader_event, 10000); // check later for unhandled packet
        CloseHandle(osReader.hEvent);
        return TSS_NO_ERROR;
    }
    return TSS_ERROR_READ;
}


















