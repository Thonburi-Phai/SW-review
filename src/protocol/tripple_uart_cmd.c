
#include "tripple_uart_cmd.h"

static char cmd_buffer[CMD_BUFFER_SIZE];
static bool cmd_available = false;
static CommandType current_cmd = CMD_NONE;

void CMD_Init(void)
{
    cmd_available = false;
    current_cmd = CMD_NONE;
    for (int i = 0; i < CMD_BUFFER_SIZE; i++)
    {
        cmd_buffer[i] = 0;
    }
}

void CMD_ProcessChar(char rx_char)
{
    // Shift existing characters left
    cmd_buffer[0] = cmd_buffer[1];
    cmd_buffer[1] = cmd_buffer[2];
    cmd_buffer[2] = rx_char;

    // Check if all characters are identical
    if (cmd_buffer[0] == cmd_buffer[1] &&
        cmd_buffer[1] == cmd_buffer[2])
    {

        // Determine command type
        switch (cmd_buffer[0])
        {
        case 'A':
            current_cmd = CMD_AAA;
            cmd_available = true;
            break;
        case 'B':
            current_cmd = CMD_BBB;
            cmd_available = true;
            break;
        case 'C':
            current_cmd = CMD_CCC;
            cmd_available = true;
            break;
        case 'D':
            current_cmd = CMD_DDD;
            cmd_available = true;
            break;
        case 'E':
            current_cmd = CMD_EEE;
            cmd_available = true;
            break;
        case 'F':
            current_cmd = CMD_FFF;
            cmd_available = true;
            break;
        case 'G':
            current_cmd = CMD_GGG;
            cmd_available = true;
            break;
        case 'H':
            current_cmd = CMD_HHH;
            cmd_available = true;
            break;
        case 'I':
            current_cmd = CMD_III;
            cmd_available = true;
            break;
        case 'J':
            current_cmd = CMD_JJJ;
            cmd_available = true;
            break;
        case 'K':
            current_cmd = CMD_KKK;
            cmd_available = true;
            break;
        case 'L':
            current_cmd = CMD_LLL;
            cmd_available = true;
            break;
        case 'M':
            current_cmd = CMD_MMM;
            cmd_available = true;
            break;
        case 'N':
            current_cmd = CMD_NNN;
            cmd_available = true;
            break;
        case 'O':
            current_cmd = CMD_OOO;
            cmd_available = true;
            break;
        case 'P':
            current_cmd = CMD_PPP;
            cmd_available = true;
            break;
        case 'Q':
            current_cmd = CMD_QQQ;
            cmd_available = true;
            break;
        case 'R':
            current_cmd = CMD_RRR;
            cmd_available = true;
            break;
        case 'S':
            current_cmd = CMD_SSS;
            cmd_available = true;
            break;
        case 'T':
            current_cmd = CMD_TTT;
            cmd_available = true;
            break;
        case 'U':
            current_cmd = CMD_UUU;
            cmd_available = true;
            break;
        case 'V':
            current_cmd = CMD_VVV;
            cmd_available = true;
            break;
        case 'W':
            current_cmd = CMD_WWW;
            cmd_available = true;
            break;
        case 'X':
            current_cmd = CMD_XXX;
            cmd_available = true;
            break;
        case 'Y':
            current_cmd = CMD_YYY;
            cmd_available = true;
            break;
        default:
            /* Not a recognized command sequence */
            break;
        }
    }
}

CommandType CMD_Execute(void)
{
    CommandType cmd = CMD_NONE;

    if (cmd_available)
    {
        cmd = current_cmd;
        CMD_Clear();
    }

    return cmd;
}

void CMD_Clear(void)
{
    cmd_available = false;
    current_cmd = CMD_NONE;
    for (int i = 0; i < CMD_BUFFER_SIZE; i++)
    {
        cmd_buffer[i] = 0;
    }
}

bool CMD_IsAvailable(void)
{
    return cmd_available;
}