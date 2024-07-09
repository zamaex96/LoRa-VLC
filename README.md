# Hybrid LoRa-VLC Communication using Ra-02 LoRa Module and STM32 Microcontroller

This project implements a diversity gain engine for an embedded system using STM32 microcontrollers. It involves initializing and configuring several peripherals, such as ADC, UART, SPI, and LoRa modules, to implement a diversity gain engine algorithm with three different combining techniques: Equal Gain Combining (EGC), Maximum Likelihood Combining (MLC), and Selection Combining (SC).

## Main Components and Functions

### Peripheral Initialization

1. **System Clock Configuration (`SystemClock_Config`)**:
   - Configures the system clock for the microcontroller.

2. **Peripheral Initialization**:
   - **ADC Initialization (`MX_ADC1_Init`)**: Configures the ADC1 to read three channels using DMA.
   - **SPI Initialization (`MX_SPI3_Init`)**: Configures the SPI3 interface for communication with the LoRa module.
   - **UART Initialization (`MX_USART1_UART_Init`)**: Configures USART1 for serial communication.
   - **DMA Initialization (`MX_DMA_Init`)**: Configures the DMA for handling ADC data transfers.
   - **GPIO Initialization (`MX_GPIO_Init`)**: Configures the general-purpose I/O pins.

### Main Function

1. **Initialization**:
   - `HAL_Init()`: Initializes the HAL library.
   - `SystemClock_Config()`: Configures the system clock.
   - Peripheral initialization functions.
   - `LoRa_Module_Setting()`: Configures the LoRa module.

2. **ADC Start**:
   - `HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 3)`: Starts the ADC in DMA mode to read values into `rawValues`.

3. **Main Loop**:
   - Continuously calls `DiversityGainEngineInit()` to execute the diversity gain engine logic.

### Diversity Gain Engine

1. **DiversityGainEngineInit()**:
   - Collects ADC readings and estimates threshold voltage.
   - Selects the combining technique based on the estimated threshold voltage.

2. **Combining Techniques**:
   - `ThresholdVoltageEstimator()`: Estimates the threshold voltage.
   - `CombiningTechniqueSelection()`: Selects the appropriate combining technique (EGC, MLC, SC) and performs data transmission using the selected technique.
   - `EGC_Engine()`: Implements Equal Gain Combining.
   - `MLC_Engine()`: Implements Maximum Likelihood Combining.
   - `SC_Engine()`: Implements Selection Combining.
   - `SC_EngineReception()`: Handles reception for the SC technique.

### LoRa Module Configuration

1. **LoRa_Module_Setting()**:
   - Initializes the LoRa module with specified parameters.
   - Starts continuous receiving mode.

2. **LoRa_Transceiver_Init()**:
   - Demonstrates sending and receiving data using the LoRa module.

### Callback and Error Handling

1. **HAL_ADC_ConvCpltCallback()**:
   - Sets a flag `convCompleted` when ADC conversion is complete.

2. **Error_Handler()**:
   - Infinite loop to indicate an error state.

### Debug and Assertion

1. **USE_FULL_ASSERT**:
   - Provides assertion handling for debugging purposes.

## Summary

This code provides a complete implementation of a diversity gain engine for an embedded system using STM32 microcontrollers. It includes the initialization and configuration of several peripherals such as ADC, UART, SPI, and LoRa modules. The diversity gain engine algorithm implements three combining techniques (EGC, MLC, SC) and continuously processes and transmits sensor data. Each function can be customized based on specific requirements or hardware specifications.
