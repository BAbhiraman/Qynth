/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>   // For fminf, fmaxf

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256
#define MIDI_PITCHES_COUNT 128
#define FS 48095.0f //see .ioc see for 48 kHz inaccuracy WAS 48000

typedef struct {
    uint8_t  vel[MIDI_PITCHES_COUNT];            // MIDI standard velocity (0-127) for each pitch
    uint8_t  released[MIDI_PITCHES_COUNT];       // True if the note has been released, false if pressed for each pitch
    uint32_t ticks_pressed[MIDI_PITCHES_COUNT];  // Ticks elapsed since the note was pressed for each pitch
    uint32_t ticks_released[MIDI_PITCHES_COUNT]; // Ticks elapsed since the note was released for each pitch
    float    env[MIDI_PITCHES_COUNT];            // Current envelope level (e.g., 0.0 to 1.0) for each pitch
} Notes; // Renamed to Notes (plural) as it now holds data for multiple notes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t RxUART[RX_BUFFER_SIZE];
volatile uint16_t OldPos = 0;
typedef enum {
    LAST_READ_STATUS,
    LAST_READ_PITCH,
    LAST_READ_VELOCITY,
    LAST_READ_PEDAL,
    LAST_READ_NONE // Good to have an initial/default state
} lastRead_t; // Using a typedef for easier use, and _t suffix is common for enums/structs
lastRead_t currentLastRead = LAST_READ_NONE;
uint8_t inNoteEvent = 0;
uint16_t currentMIDIPitch = 255; //actual values are 0-127
uint16_t MIDIptr = 253;
const int N_voices = 8;  // The number of voices to process

Notes my_midi_notes; // Declare an instance of the Notes struct

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Notes_init(Notes* notes_system) {
    // Loop through all possible MIDI pitches and set their initial state
    for (int i = 0; i < MIDI_PITCHES_COUNT; i++) {
        notes_system->vel[i] = 0;              // No velocity (note off)
        notes_system->released[i] = 0;         //
        notes_system->ticks_pressed[i] = 0;    // No ticks yet
        notes_system->ticks_released[i] = 0;   // No ticks yet
        notes_system->env[i] = 0.0f;           // Envelope at zero
    }
}

void clear_note(Notes* notes_system, uint8_t note_index) {
    // Basic validation to ensure the index is within bounds
    if (note_index < MIDI_PITCHES_COUNT) {
        notes_system->vel[note_index] = 0;              // Set velocity to 0 (note off)
        notes_system->released[note_index] = 0;         //
        notes_system->ticks_pressed[note_index] = 0;    // Reset ticks since pressed
        notes_system->ticks_released[note_index] = 0;   // Reset ticks since released
        notes_system->env[note_index] = 0.0f;           // Reset envelope level to 0.0
    } else {

        // Optional: Add an error handling or logging mechanism if an invalid index is provided
        // For embedded systems, you might have a dedicated error LED, a serial print, or nothing at all
        // depending on your error handling strategy.
        printf(stderr, "Error: clear_note called with invalid note_index: %u\n", note_index);
    }
}

/**
 * @brief Adds (activates) a note in the Notes tracking system with a given pitch and velocity.
 *
 * This function simulates a MIDI "Note On" event. It sets the velocity,
 * marks the note as pressed, and resets relevant envelope/timing parameters
 * for the specified pitch.
 *
 * @param notes_system A pointer to the Notes struct holding the state of all MIDI pitches.
 * @param pitch The MIDI pitch number (0-127) of the note to activate.
 * @param velocity The velocity value (0-127) for the note.
 */
void add_note(Notes* notes_system, uint8_t pitch, uint8_t velocity) {
    // Basic validation for the pitch
    if (notes_system == NULL) {
        printf("Error: NULL Notes system pointer passed to add_note.\r\n");
        return;
    }
    if (pitch >= MIDI_PITCHES_COUNT) {
        printf("Error: add_note called with invalid pitch: %u (max %u)\r\n", pitch, MIDI_PITCHES_COUNT - 1);
        return;
    }

    // A velocity of 0 is typically interpreted as a "Note Off" event.
    // While this function is named "add_note", it's good practice to handle
    // velocity 0 either by calling clear_note or just returning.
    if (velocity == 0) {
        // If velocity is 0, treat it as a note off (set released flag)
        // or clear the note entirely. For simplicity here, we'll mark it released
        // and let updateNoteEnvelope handle the decay.
        notes_system->released[pitch] = 1;
        // Don't actually set the vel[pitch] to 0! That variable tracks what was :')
        return;
    }

    // Set the velocity for the specified pitch
    notes_system->vel[pitch] = velocity;
    // Mark the note as pressed (not released)
    notes_system->released[pitch] = 0;
    // Reset ticks related to pressing/releasing for a new press
    notes_system->ticks_pressed[pitch] = 0;
    notes_system->ticks_released[pitch] = 0; // Not applicable for a pressed note
    // Reset envelope to 0.0f, as it will start its attack phase
    notes_system->env[pitch] = 0.0f;
}

/**
 * @brief Updates the envelope level for active notes within the Notes tracking system.
 *
 * This function iterates over the `Notes` arrays from highest to lowest pitch.
 * For the first N_voices (where velocity is not 0), it applies the ADSR envelope logic
 * (simplified to AD-Release here) to update the 'env' level for that specific note.
 *
 * @param notes_system A pointer to the Notes struct holding the state of all MIDI pitches.
 * @param attack_time_sec The duration of the attack phase in seconds.
 * @param decay_k The exponential decay factor for the decay phase (0.0 to 1.0, typically < 1.0).
 * @param release_k The exponential release factor for the release phase (0.0 to 1.0, typically < 1.0).
 */
void updateNoteEnvelope(Notes* notes_system, float attack_time_sec, float decay_k, float release_k) {
    // Ensure notes_system is not NULL
    if (notes_system == NULL) {
        printf("Error: NULL Notes system pointer passed to updateNoteEnvelope.\r\n");
        return;
    }

    // Calculate attack duration in ticks (common for all notes)
    // Ensure FS is not zero to prevent division by zero in attack_increment_per_tick
    float attack_duration_ticks = (FS > 0.0f) ? (attack_time_sec * FS) : 1.0f; // Prevent div by zero if FS is 0

    int voices_processed = 0; // Counter for active voices processed

    // Iterate over the arrays in descending pitch order
    for (int pitch = MIDI_PITCHES_COUNT - 1; pitch >= 0; pitch--) {
        // Only process notes that have velocity (are "on") and we haven't hit N_voices limit
        if (notes_system->vel[pitch] != 0) {
            // We only care about the first N_voices with non-zero velocity
            if (voices_processed >= N_voices) {
                // If this note is still active (not released and velocity > 0)
                // but we've already hit our voice limit, we might want to
                // consider stopping its envelope or giving it a very fast release.
                // For this implementation, we'll just skip it for envelope processing
                // if it's beyond N_voices and has velocity.
                // However, if it's already in release phase, we *should* continue processing it
                // until its envelope decays to 0. So, we adjust the condition here.
//                if (notes_system->released[pitch] == 0) { // If it's a *pressed* note beyond N_voices
//                    continue; // Skip processing its envelope as it's not a primary voice
//                }
                // If it's a released note (vel!=0, but released==1), we still process it
                // to allow its envelope to decay, regardless of N_voices.
                // The N_voices limit primarily applies to "active, pressed" notes.
            }

            // Check if the note is currently pressed (not released)
            if (notes_system->released[pitch] == 0) {
                notes_system->ticks_pressed[pitch]++; // Increment ticks since press

                // Attack Phase: if ticks_pressed is within the attack duration
                if (notes_system->ticks_pressed[pitch] <= attack_duration_ticks) {
                    // Calculate the linear increment per tick
                    if (attack_duration_ticks > 0.0f) {
                        float attack_increment_per_tick = 1.0f / attack_duration_ticks;
                        notes_system->env[pitch] += attack_increment_per_tick;
                    } else { // Instant attack
                        notes_system->env[pitch] = 1.0f;
                    }
                    // Clip the envelope level at 1.0 (maximum)
                    notes_system->env[pitch] = fminf(notes_system->env[pitch], 1.0f);
                }
                // Decay Phase: if attack is finished and note is still pressed
                else {
                    notes_system->env[pitch] *= decay_k;
                    // Ensure envelope doesn't go below 0 due to floating point inaccuracies
                    notes_system->env[pitch] = fmaxf(notes_system->env[pitch], 0.0f);
                }

                // Only increment voices_processed for currently pressed notes
                voices_processed++;

            } else { // If the note has been released (notes_system->released[pitch] == 1)
                notes_system->ticks_released[pitch]++; // Increment ticks since release
                // Release Phase: apply release scalar
                notes_system->env[pitch] *= release_k;
                // Ensure envelope doesn't go below 0
                notes_system->env[pitch] = fmaxf(notes_system->env[pitch], 0.0f);
            }

            // After envelope processing, if the envelope has decayed to near zero,
            // and the note is released, we can "turn off" the note entirely
            // by setting its velocity to 0 and clearing its state.
            // This is crucial for managing polyphony and avoiding processing
            // completely silent, released notes indefinitely.
            if (notes_system->env[pitch] < 0.001f && notes_system->released[pitch] == 1) {
                // Call your clear_note function here if you have one
                // clear_note(notes_system, pitch); // Assuming clear_note sets vel to 0, released to 1, etc.
                // If you don't have clear_note, you'd do:
                notes_system->vel[pitch] = 0;
                notes_system->ticks_pressed[pitch] = 0;
                notes_system->ticks_released[pitch] = 0;
                notes_system->env[pitch] = 0.0f; // Ensure it's exactly zero
                notes_system->released[pitch] = 0.0f; // Set released to 0 so that the next instance is ready.
            }
        }
    }
}

void ParseMIDI(uint8_t* data, uint16_t length) {
	int did_something = 0;
	uint8_t byte;
	for (uint16_t i = 0; i < length; i++) {
		byte = data[i];
		if (byte != 254) {

			did_something = 1;
			// Status byte if MSB = 1
			if ((byte >> 7) & 0x01) {
				// if MS Nybble is 0x9, Note On command
				currentLastRead = LAST_READ_STATUS;
				if ((byte >> 4) == 0x9) {
					//inNoteEvent = 1;
					printf("ON ");
				}
				else if ((byte >> 4) == 0x8) {
					//inNoteEvent = 0;
					printf("OFF ");
				}
			}
			// Data byte if MSB = 0
			else {
				if ((currentLastRead == LAST_READ_STATUS)) {
					currentLastRead = LAST_READ_PITCH;
					MIDIptr = byte;
					printf("note %02X ", byte);
				}
				else if (currentLastRead == LAST_READ_PITCH) {
					currentLastRead = LAST_READ_VELOCITY;
					if (byte == 0) {
						inNoteEvent = 0;
						clear_note(&my_midi_notes, currentMIDIPitch);
					}
					else {
						currentMIDIPitch = MIDIptr;
						add_note(&my_midi_notes, currentMIDIPitch, byte);
					}
					printf("vel %02X ", byte);
				}
				else if (currentLastRead == LAST_READ_VELOCITY) {
					currentLastRead = LAST_READ_PITCH;
					MIDIptr = byte;
					printf("note %02X ", byte);
				}
			}
		}
	}
	if (did_something) {
		HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
		printf("\r\n");
	}

}

void ReadRxBuffer() {
	uint16_t pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	if (pos != OldPos) {
		if (pos > OldPos) {
			// Data is contiguous
			ParseMIDI(&RxUART[OldPos], pos - OldPos);
		}
		else {
			// Data loops around
			ParseMIDI(&RxUART[OldPos], RX_BUFFER_SIZE - OldPos);
			ParseMIDI(&RxUART[0], pos);
		}
		OldPos = pos;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Hello world!\r\n");

  Notes_init(&my_midi_notes); // Initialize the entire system

  HAL_UART_Receive_DMA(&huart2, RxUART, RX_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_Delay(2);
	  HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
	  ReadRxBuffer();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 31250;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx =0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
