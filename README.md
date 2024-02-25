# MAX86141 Optical Biomedical Sensor

## Tasks completed:

1. **Device Driver Code**: The device driver code to extract signal information from MAXM86141 Optical Biomedical Sensor can be found in the `src` folder. It includes `.h` and `.cpp` files.

2. **Algorithms for HR and SPO2**: The algorithms to find out the Heart Rate (HR) and Oxygen Saturation Level (SPO2) from the biosignals are provided in the `src` folder. It includes `.h` and `.cpp` files. 

3. **Functions for HRV and RR**: Functions to calculate Heart Rate Variability (HRV) and Respiration Rate (RR) using the HR and SPO2 values are implemented in the `max86141.ino` file in the `examples` folder.

## To be completed:

1. **Noise Suppression Method**: A method to suppress volatile or spurious noise in the signal due to motion artifacts is yet to be completed.

## References:

1. [MAX86140 Datasheet](Docs/MAX86140.pdf)
2. [Algorithm by RF](https://github.com/jonasgitt/Patient24---Remote-Patient-Monitoring/blob/master/src/MAX86141/algorithm_by_RF.cpp)
3. [MAX86141 Arduino Library](https://github.com/joshbrew/MAX86141_Arduino/blob/master/libraries/MAX86141/MAX86141.cpp)
4. [Max86141 Example](https://github.com/MakerLabCRI/Max86141/blob/main/examples/Max86141_Init/Max86141_Init.ino)
