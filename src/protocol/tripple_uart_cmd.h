#ifndef TRIPPLE_UART_CMD_H
#define TRIPPLE_UART_CMD_H

#include <stdint.h>
#include <stdbool.h>

// Command buffer size
#define CMD_BUFFER_SIZE 3

// Command types
typedef enum
{
    CMD_NONE = 0,
    CMD_AAA,
    CMD_BBB,
    CMD_CCC,
    CMD_DDD,
    CMD_EEE,
    CMD_FFF,
    CMD_GGG,
    CMD_HHH,
    CMD_III, // Bind 1
    CMD_JJJ, // Bind 2
    CMD_KKK, // Bind 3
    CMD_LLL, // Bind 4
    CMD_MMM,
    CMD_NNN,
    CMD_OOO,
    CMD_PPP,
    CMD_QQQ,  // Balance override cycle test (open-wire bench)

    CMD_RRR,  // Single debug report
    CMD_SSS,
    CMD_TTT,  // Toggle debug report (100ms)
    CMD_UUU,  // BQ sleep/wake test
    CMD_VVV,  // binary protocol protocol self-test
    CMD_WWW,
    CMD_XXX,
    CMD_YYY,  // LPUART1 debug dump
} CommandType;

// Initialize the command detector
void CMD_Init(void);

// Function to be called from UART IRQ handler
void CMD_ProcessChar(char rx_char);

// Function to be called from main loop to check and execute commands
CommandType CMD_Execute(void);

// Clear the current command
void CMD_Clear(void);

// Check if a new command is available
bool CMD_IsAvailable(void);

#endif // TRIPPLE_UART_CMD_H
