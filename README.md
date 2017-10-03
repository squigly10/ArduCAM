# ArduCAM
## Lightweight version of the ArduCAM Library for use with ArduTom.  This library works only with the OV5642 ArduCAM model.
<br>
<br>

#### This project will be completed in a series of phases:
 * __Phase 1:__ _[COMPLETE]_ Removing anything not related to the OV5642
 * __Phase 2:__ _[COMPLETE*]_ Removing functions _for_ the OV5642 that we're not using
 * __Phase 3:__ Simplifying any remaining code to contain only what we need
 * __Phase 4:__ Updating header files and removing unused files
<br>
<br>

#### Settings Functions:
```c++
void ArduCAM::OV5642_set_Compress_quality(uint8_t);
void ArduCAM::OV5642_set_Sharpness(uint8_t);
void ArduCAM::OV5642_set_Exposure_level(uint8_t);
void ArduCAM::OV5642_set_Contrast(uint8_t);
void ArduCAM::OV5642_set_Color_Saturation(uint8_t);
void ArduCAM::set_format(byte);
void ArduCAM::OV5642_set_JPEG_size(uint8_t);
void ArduCAM::OV5642_set_RAW_size(uint8_t);
void ArduCAM::set_mode(uint8_t);
void ArduCAM::InitCAM();
void ArduCAM::ArduCAM(int);
void ArduCAM::ArduCAM(byte, int);
```
<br>
Other functions exist, mostly for registry stuff (will update later)
<br>
<br>
[*] Complete, but may need to revisit
