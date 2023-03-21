*********************************************************************************************
* This application showcases the basic radar features of AWR2243 ES 1.0/ES1.1 mmWave device *
*  and mmWaveLink APIs usage on External Host environment for the same.                     *
*********************************************************************************************

Note:
    This application supports both SPI and I2C mode of operation.
    Refer to mmwaveconfig.txt on choosing the mode of operation.
 
How to run:
    1. Connect AWR2243 ES 1.0/ES 1.1 boosterpack and DCA1000 EVM to PC.
    2. Erase sFlash before running this application.
    3. Run mmwavelink_example.exe.
    
Execution flow of the application:
    1. Application sets the device in SOP4 mode (in the case of SPI) or SOP7 mode (in the case of I2C).
    2. Downloads the meta image over SPI/I2C (based on the mode chosen).
    3. API parameters for all the commands are read from mmwaveconfig.txt.

Note:
    1. To modify and re-run the application, use Visual Studio based project provided in the same directory.
    2. "trace.txt" file is created which logs all the SPI/I2C communication commands.
    3. "CalibrationData.txt" file is created which stores the calibration data. When Calibration restore is issued,
       it makes use of the data present in this file.
    4. "PhShiftCalibrationData.txt" file is created which stores the phase shifter calibration data. 
       When phase shifter Calibration restore is issued, it makes use of the data present in this file.
    5. "AdvChirpLUTData.txt" file is created which stores the locally programmed LUT data that is sent to RadarSS
       to populate the LUT at the device end.
