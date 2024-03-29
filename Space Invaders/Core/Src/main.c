/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Sprites.h"
#include "Sound.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */

enum State {
	INTRO_SCREEN,
	RESET_LEVEL1,
	LEVEL_1,
	WIN,
	LOSE
};

enum State gameStatus;

int highScore = 0;
int playerScore = 0;

volatile uint8_t refresh = 0;
volatile uint8_t player_fire = 0;
volatile uint8_t continue_input = 0;

int ship_x, ship_y = 0;
int ship_px, ship_py = 0;
int ship_health = 1;

int player_missile_xpos[3];
int player_missile_ypos[3];
int player_missile_prev_xpos[3];
int player_missile_prev_ypos[3];
int player_missile_status[3] = {0};

int barrier_x_pos[3] = {0};
int barrier_y_pos[3] = {0};
int barrier_health[3] = {0};


int invader1_xpos[4][6] = {0};
int invader1_ypos[4][6] = {0};
int invader1_prev_xpos[4][6] = {0};
int invader1_prev_ypos[4][6] = {0};
int invader1_status[4][6] = {0};
int row_to_move = 3;
int inv1_direction = 1;
int all_invader1_alive = 1; // 1: all alive; 0: all dead
int total_inv1_left = 4*6;


int invader1_fire_sequence[] = {
20, 4, 10, 9, 23, 6, 16, 22, 12,
11, 3, 6, 17, 8, 23, 7, 22, 12, 16,
22, 17, 6, 22, 22, 4, 2, 17, 13, 3,
12, 12, 10, 3, 6, 14, 19, 22, 5, 8,
7, 16, 23, 5, 0, 8, 23, 9, 15, 19, 9,
5, 21, 8, 4, 15, 3, 10, 21, 5, 3, 1,
19, 13, 10, 18, 22, 21, 15, 16, 14, 21,
5, 10, 4, 15, 4, 4, 20, 17, 17, 10, 10,
17, 7, 1, 3, 6, 4, 15, 0, 14, 17, 8, 9,
15, 21
};
int invader1_fire_index = 0;
int invader1_missile_xpos[6];
int invader1_missile_ypos[6];
int invader1_missile_prev_xpos[6];
int invader1_missile_prev_ypos[6];
int invader1_missile_status[6] = {0};

uint32_t ctr_lastcheck = 0;
uint32_t ctr_invader1_lastcheck = 0;
uint32_t ctr_invader1_fire = 0;

int inv1_x_pos_to_explode[3] = {0};
int inv1_y_pos_to_explode[3] = {0};
int inv1_explode_phase[3] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
void displayIntroScreen();
void displayGameOver();
void displayint(uint16_t val, uint32_t* screenptr);
void clearscreen();
void updatescreen();
void updatePlayerMissileStatus(int missile_index);
void updateEnemyMissileStatus(int missile_index);
void explodeInvader1();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void displayIntroScreen()
{

	// Intro Screen
	/*
	uint8_t* title_screen = (uint8_t*) Title_Screen;
	uint8_t* screenptr = (uint8_t*) (0x20020000 + 40*320 + 37);
	for (int i = 0; i < 31; ++i) {
		for (int j = 0; j < 245; ++j) {
			*screenptr++ = *title_screen++;
		}
		screenptr += 75;
	}
	*/

	// Intro Screen
	uint8_t* title_screen = (uint8_t*) Title_Screen;
	uint8_t* screenptr = (uint8_t*) (0x20020000 + 40*320 + 37);
	for (int i = 0; i < 31; ++i) {
		for (int j = 0; j < 245; ++j) {
			*screenptr++ = *title_screen++;
		}
		screenptr += 75;
	}

	uint8_t* title_instuction = (uint8_t*) Title_Instruction;
	screenptr = (uint8_t*) (0x20020000 + 100*320 + 64);
	for (int i = 0; i < 16; ++i) {
		for (int j = 0; j < 191; ++j) {
			*screenptr++ = *title_instuction++;
		}
		screenptr += 129;
	}

	while (continue_input != 1);
	continue_input = 0;

	gameStatus = RESET_LEVEL1;
	clearscreen();

	return;
}

void displayGameOver()
{
	clearscreen();

	uint8_t* screenptr = (uint8_t*) (0x20020000 + 40*320 + 38);
	uint8_t* game_over = (uint8_t*) Game_Over;


	for (int i = 0; i < 46; ++i) {
		for (int j = 0; j < 244; ++j) {
			*screenptr++ = *game_over++;
		}
		screenptr += 76;
	}

	uint8_t* title_instuction = (uint8_t*) Title_Instruction;
	screenptr = (uint8_t*) (0x20020000 + 150*320 + 64);
	for (int i = 0; i < 16; ++i) {
		for (int j = 0; j < 191; ++j) {
			*screenptr++ = *title_instuction++;
		}
		screenptr += 129;
	}

	while (continue_input != 1);
	continue_input = 0;

	gameStatus = RESET_LEVEL1;
	clearscreen();

	return;
}

void clearscreen()
{
	uint32_t* ptrscreen = (uint32_t*)0x20020000;
	for (int i = 0; i < 16000; ++i) {
		*ptrscreen++ = 0;
	}


	// copy title sprite
	uint32_t* titleptr = (uint32_t*)title;
	ptrscreen = (uint32_t*)0x20020000;
	for (int i = 0; i < 1600; i++)
	{
		*ptrscreen++ = *titleptr++;
	}

	displayint(highScore, (uint32_t*)(0x200204E0));
	displayint(playerScore, (uint32_t*)(0x20020418));

	return;
}

void displayint(uint16_t val, uint32_t* screenptr)
{
	uint8_t digit = 0;
	uint32_t* digitptr;
	uint32_t* scrcopyptr;
	for (int i = 0; i < 5; i++)
	{
		digit = val % 10;
		val /= 10;

		scrcopyptr = screenptr;
		digitptr = (uint32_t*)(digits + (digit << 3));
		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				*scrcopyptr++ = *digitptr++;
			}
			digitptr += 18;
			scrcopyptr += 78;

		}
		screenptr -= 2;
	}
}

void updatescreen()
{
	uint8_t* ptrscreen;

	// Erase Previous Ship
	ptrscreen = (uint8_t*)(0x20020000 + ship_py*320 + ship_px);
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 12; j++) {
			*ptrscreen++ = 0;
		}
		ptrscreen += 308;
	}

	// Draw New Ship
	ptrscreen = (uint8_t*)(0x20020000 + ship_y*320 + ship_x);
	uint8_t* ptrsrc = (uint8_t*) Ship;
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 12; j++) {
			*ptrscreen++ = *ptrsrc++;
		}
		ptrscreen += 308;
	}

	// Erase broken barriers
	for (int barrier_index = 0; barrier_index < 3; ++barrier_index) {
		if (barrier_health[barrier_index] == 0) {
			ptrscreen = (uint8_t*) (0x20020000 + barrier_y_pos[barrier_index]*320 + barrier_x_pos[barrier_index]);
			for (int i = 0; i < 5; ++i) {
				for (int j = 0; j < 22; ++j) {
					*ptrscreen++ = 0;
				}
				ptrscreen += 298;
			}
		}

	}

	// Draw Barriers
	for (int barrier_index = 0; barrier_index < 3; ++barrier_index) {
		if (barrier_health[barrier_index] > 0) {
			ptrscreen = (uint8_t*) (0x20020000 + barrier_y_pos[barrier_index]*320 + barrier_x_pos[barrier_index]);
			ptrsrc = (uint8_t*) Barrier;
			for (int i = 0; i < 5; ++i) {
				for (int j = 0; j < 22; ++j) {
					*ptrscreen++ = *ptrsrc++;
				}
				ptrscreen += 298;
			}
		}

	}


	// Erase Previous Invader 1 Positions
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 6; col++) {
				ptrscreen = (uint8_t*)(0x20020000 +invader1_prev_ypos[row][col]*320 + invader1_prev_xpos[row][col]);
				for (int i = 0; i < 8; i++) {
					for (int j = 0; j < 22; j++) {
						*ptrscreen++ = 0;
					}
					ptrscreen += 298;
				}

		}
	}


	// Draw New Invader 1 Positions
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 6; col++) {
			if (invader1_status[row][col] == 1) {
				ptrscreen = (uint8_t*)(0x20020000 +invader1_ypos[row][col]*320 + invader1_xpos[row][col]);
				ptrsrc = (uint8_t*) Invader1;
				for (int i = 0; i < 8; i++) {
					for (int j = 0; j < 12; j++) {
						*ptrscreen++ = *ptrsrc++;
					}
					ptrscreen += 308;
				}
			}
		}
	}

	// Erase Previous Player Missile Positions
	for (int missile_index = 0; missile_index < 3; ++missile_index) {
		ptrscreen = (uint8_t*)(0x20020000 + player_missile_prev_ypos[missile_index]*320 + player_missile_prev_xpos[missile_index]);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 1; j++) {
				*ptrscreen++ = 0;
			}
			ptrscreen += 319;
		}
	}

	// Display Player Missile
	for (int m = 0; m < 3; ++m) {
		if (player_missile_status[m] == 1) {
			ptrscreen = (uint8_t*)(0x20020000 + player_missile_ypos[m]*320 + player_missile_xpos[m]);
			ptrsrc = (uint8_t*)  Missile_Green;
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 1; ++j) {
					*ptrscreen++ = *ptrsrc++;
				}
				ptrscreen += 319;
			}
		}
	}

	// Erase Previous Invader 1 Missile Positions
	for (int missile_index = 0; missile_index < 6; ++missile_index) {
		if (invader1_missile_status[missile_index] == 1) {
			ptrscreen = (uint8_t*)(0x20020000 + invader1_missile_prev_ypos[missile_index]*320 + invader1_missile_prev_xpos[missile_index]);
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 1; ++j) {
					*ptrscreen++ = 0;
				}
				ptrscreen += 319;
			}
		}
	}

	// Display Invader 1 Missile
	for (int missile_index = 0; missile_index < 6; ++missile_index) {
		if (invader1_missile_status[missile_index] == 1) {
			ptrscreen = (uint8_t*)(0x20020000 + invader1_missile_ypos[missile_index]*320 + invader1_missile_xpos[missile_index]);
			ptrsrc = (uint8_t*) Missile_Red;
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 1; ++j) {
					*ptrscreen++ = *ptrsrc++;
				}
				ptrscreen += 319;
			}
		}
	}

	// Display explosions
	explodeInvader1();


	displayint(playerScore, (uint32_t*)(0x20020418));
}

void updatePlayerMissileStatus(int missile_index)
{
	int pos_x = player_missile_prev_xpos[missile_index];
	int pos_y = player_missile_prev_ypos[missile_index];

	// Check if missile is out of bounds
	if (player_missile_ypos[missile_index] <= 15) {
		player_missile_status[missile_index] = 0;
		uint8_t* ptrscreen = (uint8_t*)(0x20020000 + pos_y*320 + pos_x);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 1; j++) {
				*ptrscreen++ = 0;
			}
			ptrscreen += 319;
		}
		return;
	}

	/// Player Missile and Barrier Collision
	for (int barrier_index = 0; barrier_index < 3; ++barrier_index) {
		if (barrier_health[barrier_index] > 0) {
			if ( (barrier_x_pos[barrier_index] + 22 >= pos_x) && (barrier_x_pos[barrier_index] <= pos_x + 2) && (barrier_y_pos[barrier_index] + 3 >= pos_y) && (barrier_y_pos[barrier_index] <= pos_y) ) {
				--barrier_health[barrier_index];
				player_missile_status[missile_index] = 0;
				return;
			}
		}
	}



	/// Check for player missile and enemy collision
	// Loop through enemy positions to check if one is hit
	int inv_pos_x;
	int inv_pos_y;
	int inv_w = 12;
	int inv_h = 8;
	int inv_r, inv_c, inv_hit = 0;
	for (int row = 0; row < 4; ++row) {
		for (int col = 0; col < 6; ++col) {
			if (invader1_status[row][col] == 0) {
				continue;
			}
			inv_pos_x = invader1_prev_xpos[row][col];
			inv_pos_y = invader1_prev_ypos[row][col];

			if ( ((inv_pos_x + inv_w) >= pos_x) && (inv_pos_x <= pos_x + 2) && ((inv_pos_y + inv_h) >= pos_y) && (inv_pos_y <= pos_y) ) {
				inv_hit = 1;
				inv_r = row;
				inv_c = col;
				break;
			}


		}
	}

	if (inv_hit == 1) {
		--total_inv1_left;
		if (total_inv1_left > 0) {
			HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*) audio_explode, AUDIOLEN_EXPLODE);
		}

		playerScore += 10;
		player_missile_status[missile_index] = 0;
		invader1_status[inv_r][inv_c] = 0;

		inv_pos_x = invader1_prev_xpos[inv_r][inv_c];
		inv_pos_y = invader1_prev_ypos[inv_r][inv_c];

		// Erase missile and enemy
		uint8_t* ptrscreen = (uint8_t*)(0x20020000 + pos_y*320 + pos_x);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 1; j++) {
				*ptrscreen++ = 0;
			}
			ptrscreen += 319;
		}

		ptrscreen = (uint8_t*)(0x20020000 + inv_pos_y*320 + inv_pos_x);
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 12; j++) {
				*ptrscreen++ = 0;
			}
			ptrscreen += 308;
		}

		// Find an open explosion index and assign explode coordinates
		for (int explode_index = 0; explode_index < 3; ++explode_index) {
			if (inv1_explode_phase[explode_index] == 0){
				inv1_explode_phase[explode_index] = 1;

				int explode_x_pos = inv_pos_x - 10;
				int explode_y_pos = inv_pos_y - 12;

				// Cap explosion coordinates to the screen
				if (explode_x_pos <= 0) explode_x_pos = 0;
				if (explode_x_pos >= 288) explode_x_pos = 288;
				if (explode_y_pos >= 167) explode_y_pos = 167;
				if (explode_y_pos <= 0) explode_y_pos = 0;

				inv1_x_pos_to_explode[explode_index] = explode_x_pos;
				inv1_y_pos_to_explode[explode_index] = explode_y_pos;
				break;
			}
		}

	}


	return;
}


void updateEnemyMissileStatus(int missile_index)
{

	int pos_x = invader1_missile_xpos[missile_index];
	int pos_y = invader1_missile_ypos[missile_index];

	if (pos_y >= 200) { // out of bounds and erase
		invader1_missile_status[missile_index] = 0;
		uint8_t* ptrscreen = (uint8_t*)(0x20020000 + invader1_missile_prev_ypos[missile_index]*320 + invader1_missile_prev_xpos[missile_index]);
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 1; ++j) {
				*ptrscreen++ = 0;
			}
			ptrscreen += 319;
		}
		return;
	}

	/// Enemy Missile and Barrier Collision
	for (int barrier_index = 0; barrier_index < 3; ++barrier_index) {
		if (barrier_health[barrier_index] > 0) {
			if ( (barrier_x_pos[barrier_index] + 22 >= pos_x) && (barrier_x_pos[barrier_index] <= pos_x + 2) && (barrier_y_pos[barrier_index] + 3 >= pos_y) && (barrier_y_pos[barrier_index] <= pos_y) ) {
				--barrier_health[barrier_index];
				invader1_missile_status[missile_index] = 0;
				return;
			}
		}
	}

	/// Check if missile hit player ship
	if ((ship_x + 12 >= pos_x) && (ship_x <= pos_x + 1) && (ship_y + 8 >= pos_y) && (ship_y <= pos_y + 1)) { // Player Ship Hit
		--ship_health;
		if (ship_health <= 0) {
			gameStatus = LOSE;
			HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*) boop_2, AUDIOLEN_BOOP_2);
		}
		return;
	}


	return;
}

void explodeInvader1()
{
	for (int explode_index = 0; explode_index < 3; ++explode_index) {


			uint8_t* screenptr = (uint8_t*) (0x20020000 + inv1_y_pos_to_explode[explode_index]*320 + inv1_x_pos_to_explode[explode_index]);
			uint8_t* explode_sprite = 0;

			// Erase explosion if sequence was complete and set explode_phase to 0
			if (inv1_explode_phase[explode_index] == 8) {

				for (int i = 0; i < 32; ++i) {
					for (int j = 0; j < 32; ++j) {
						*screenptr++ = 0;
					}
					screenptr += 288;
				}
				inv1_explode_phase[explode_index] = 0;
				inv1_x_pos_to_explode[explode_index] = 0;
				inv1_y_pos_to_explode[explode_index] = 0;
			}

			// Find the right explode sprite
			switch (inv1_explode_phase[explode_index]) {
				case 1:
					explode_sprite = (uint8_t*) explode1;
					break;
				case 2:
					explode_sprite = (uint8_t*) explode2;
					break;
				case 3:
					explode_sprite = (uint8_t*) explode3;
					break;
				case 4:
					explode_sprite = (uint8_t*) explode4;
					break;
				case 5:
					explode_sprite = (uint8_t*) explode5;
					break;
				case 6:
					explode_sprite = (uint8_t*) explode6;
					break;
				case 7:
					explode_sprite = (uint8_t*) explode7;
					break;
			}
			if (inv1_explode_phase[explode_index] != 0) {
				for (int i = 0; i < 32; ++i) {
					for (int j = 0; j < 32; ++j) {
						*screenptr++ = *explode_sprite++;
					}
					screenptr += 288;
				}
				inv1_explode_phase[explode_index]++;
			}

	}
	return;
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
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  ship_x = 154;
  ship_y = 192;
  ship_px = ship_x;
  ship_py = ship_y;

  // Initialize Invader 1 position and status
  for (int r = 0; r < 4; r++) {
	  for (int c = 0; c < 6; c++) {
		  invader1_status[r][c] = 1;
		  invader1_xpos[r][c] = 80 + c*30;
		  invader1_ypos[r][c] = 50 + r*20;
		  invader1_prev_xpos[r][c] = invader1_xpos[r][c];
		  invader1_prev_ypos[r][c] = invader1_ypos[r][c];
	  }
  }

  gameStatus = INTRO_SCREEN;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /// Display Intro Screen
	  if (gameStatus == INTRO_SCREEN) {
		  displayIntroScreen();
	  }

	  /// Lost Game
	  if (gameStatus == LOSE) {
		  if (playerScore >= highScore) {
			  highScore = playerScore;
		  }
		  playerScore = 0;

		  displayGameOver();
		  gameStatus = RESET_LEVEL1;
	  }

	  /// Reset Game
	  if (gameStatus == RESET_LEVEL1) {

		  total_inv1_left = 4*6;

		  // Initialize Invader 1 position and status
		  all_invader1_alive = 1;
		  for (int r = 0; r < 4; r++) {
			  for (int c = 0; c < 6; c++) {
				  invader1_status[r][c] = 1;
				  invader1_xpos[r][c] = 80 + c*30;
				  invader1_ypos[r][c] = 50 + r*20;
				  invader1_prev_xpos[r][c] = invader1_xpos[r][c];
				  invader1_prev_ypos[r][c] = invader1_ypos[r][c];
			  }
		  }

		  // Reset Player Missiles
		  for (int missile_index = 0; missile_index < 3; ++missile_index) {
			player_missile_status[missile_index] = 0;
		  }
		  // Reset Invader 1 Missiles
		  for (int missile_index = 0; missile_index < 6; ++missile_index) {
			invader1_missile_status[missile_index] = 0;
		  }

		  // Reset and initialize barriers
		  for (int barrier_index = 0; barrier_index < 3; ++barrier_index) {
			barrier_health[barrier_index] = 4;
			barrier_x_pos[barrier_index] = 60 + barrier_index*80;
			barrier_y_pos[barrier_index] = 160;
		  }

		  gameStatus = LEVEL_1;
		  clearscreen();
	  }

	  /// Level 1
	  if (gameStatus == LEVEL_1) {
		if (continue_input != 0) continue_input = 0; // In case continue input was accidentally set.

		if (refresh==1) { // refresh interrupt was triggered
		  updatescreen();

		  // Save player previous position
		  ship_px = ship_x;
		  ship_py = ship_y;

		  // Save enemy invader 1 previous position
		  for (int r = 0; r < 4; r++) {
			  for (int c = 0; c < 6; c++) {
				  invader1_prev_xpos[r][c] = invader1_xpos[r][c];
				  invader1_prev_ypos[r][c] = invader1_ypos[r][c];
			  }
		  }

		  // Save player missiles previous position
		  for (int missile_index = 0; missile_index < 3; ++missile_index) {
			if (player_missile_status[missile_index] == 1) {
				player_missile_prev_xpos[missile_index] = player_missile_xpos[missile_index];
				player_missile_prev_ypos[missile_index] = player_missile_ypos[missile_index];
			}
		  }

		  // Save enemy missiles previous position
		  for (int missile_index = 0; missile_index < 6; ++missile_index) {
			if (invader1_missile_status[missile_index] == 1) {
				invader1_missile_prev_xpos[missile_index] = invader1_missile_xpos[missile_index];
				invader1_missile_prev_ypos[missile_index] = invader1_missile_ypos[missile_index];
			}
		  }

		  refresh = 0;
		} // Refresh triggered


		// Move player ship, player missiles and enemy missiles
		if (HAL_GetTick() - ctr_lastcheck >= 1) {
			if ((ship_x < 308) && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)) { // Right
				ship_x++;
			}
			else if ((ship_x > 0) && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)) { // Left
				ship_x--;
			}

			// Move the fired player missiles
			for (int missile_index = 0; missile_index < 3; ++missile_index) {
				if (player_missile_status[missile_index] == 1) {
					player_missile_ypos[missile_index]--;
					updatePlayerMissileStatus(missile_index);
				}
			}

			// Move fired enemy missiles
			for (int missile_index = 0; missile_index < 6; ++missile_index) {
				if (invader1_missile_status[missile_index] == 1) {
					invader1_missile_ypos[missile_index]++;
					updateEnemyMissileStatus(missile_index);
				}
			}

			ctr_lastcheck = HAL_GetTick();
		}

		// Move Invader 1
		if (HAL_GetTick() - ctr_invader1_lastcheck >= 16) {

			if (row_to_move < 0) {
				row_to_move = 3;
			}
			for (int col = 0; col < 6; col++) {
				if (inv1_direction == 1) {
					invader1_xpos[row_to_move][col] += 4;
				}
				else if (inv1_direction == 0) {
					invader1_xpos[row_to_move][col] -= 4;
				}
			}
			row_to_move--;

			if (invader1_xpos[0][5] > 304) { // move down at right side
				for (int row = 0; row < 4; row++) {
					for (int col = 0; col < 6; col++) {
						invader1_ypos[row][col] += 2;
					}
				}
				inv1_direction = 0;
			}
			else if (invader1_xpos[0][0] < 4) { // down at left side
				for (int row = 0; row < 4; ++row) {
					for (int col = 0; col < 6; ++col) {
						invader1_ypos[row][col] += 2;
					}
				}
				inv1_direction = 1;
			}

			ctr_invader1_lastcheck = HAL_GetTick();
		}


		// Player ship fired missile
		if (player_fire == 1) {
			// loop to find available missile
			for (int missile_index = 0; missile_index < 3; ++missile_index) {
				if (player_missile_status[missile_index] == 0) {
					HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*) pew, AUDIOLEN_PEW);
					player_missile_status[missile_index] = 1;
					player_missile_xpos[missile_index] = ship_x + 5;
					player_missile_ypos[missile_index] = ship_y - 4;
					break;
				}
			}

			player_fire = 0;
		}

		// Fire invader 1 missile
		if (HAL_GetTick() - ctr_invader1_fire >= 2) {
			for (int missile_index = 0; missile_index < 6; ++missile_index) {

				if (invader1_missile_status[missile_index] == 0) {
					if (invader1_fire_index > 95) {
						invader1_fire_index = 0;
					}
					int row = invader1_fire_sequence[invader1_fire_index] / 6;
					int col = invader1_fire_sequence[invader1_fire_index] % 6;
					invader1_fire_index++;
					if (invader1_status[row][col] == 1) {
						invader1_missile_status[missile_index] = 1;
						invader1_missile_xpos[missile_index] = invader1_xpos[row][col];
						invader1_missile_ypos[missile_index] = invader1_ypos[row][col];
					}
				}
				break;
			}
			ctr_invader1_fire = HAL_GetTick();
		}

		// Check if all Invader1 are dead
		all_invader1_alive = 0;
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 6; ++col) {
				if (invader1_status[row][col] == 1) {
					all_invader1_alive = 1;
				}
			}
			if (all_invader1_alive) {
				break;
			}
		}
		if (all_invader1_alive == 0) {
			HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*) great_success, AUDIOLEN_GREAT_SUCCESS);
			gameStatus = RESET_LEVEL1;
		}

	  } // end Level_1




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor 
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
