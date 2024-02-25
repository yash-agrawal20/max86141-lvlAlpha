# MAX86141 Device Drivers

## Tasks completed:

1. **Device Driver Code**: The device driver code to extract signal information from MAXM86141 Optical Biomedical Sensor can be found in the `src` folder. It is `max86141.h` and `max86141.cpp` files.

2. **Algorithms for HR and SPO2**: The algorithms to find out the Heart Rate (HR) and Oxygen Saturation Level (SPO2) from the biosignals are provided in the `src` folder. It includes `algorithm_by_RF.h` and `algorithm_by_RF.cpp` files. It is referenced from `jonasgitt` github page.

3. **Functions for HRV and RR**: Functions to calculate Heart Rate Variability (HRV) and Respiration Rate (RR) using the HR and SPO2 values are implemented in the `max86141.ino` file in the `Examples` folder.

## To be completed:

1. **Noise Suppression Method**: A method to suppress volatile or spurious noise in the signal due to motion artifacts is yet to be completed.

## References:

1. [MAX86140 Datasheet](Docs/MAX86140.pdf)
2. [Algorithm by RF](https://github.com/jonasgitt/Patient24---Remote-Patient-Monitoring/blob/master/src/MAX86141)
3. [MAX86141 Arduino Library](https://github.com/joshbrew/MAX86141_Arduino/blob/master/libraries/MAX86141)
4. [Max86141 Example](https://github.com/MakerLabCRI/Max86141/blob/main/examples/Max86141_Init)
