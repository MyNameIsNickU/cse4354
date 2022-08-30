/*
 * cmd.c
 *
 *  Created on: Mar 25, 2022
 *      Author: Nicholas
 */


#include "cmd.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "rtos.h"

void int_tostr(uint32_t input, char * result)
{
    uint32_t input_temp = input;
    uint8_t places = 0;

    char int_map[11] = "0123456789";

    while(input_temp > 0)
    {
        input_temp /= 10;
        places++;
    }

    input_temp = input;

    result[places--] = '\0';

    do
    {
        result[places--] = int_map[input_temp % 10];
        input_temp /= 10;
    } while(input_temp > 0);
}

void getsUart0(USER_DATA* data)
{
    int count = 0;
    char c;

    while( 1 )
    {
        c = getcUart0();

        // If char c is a backspace (8 or 127), allows overriding of buffer
        if( c == 8 || c == 127 && count > 0)
            count--;
        // If the char c is readable (space, num, alpha), read to buffer
        else if( c >= 32 )
            data->buffer[count++] = c;

        // If return is hit or the MAX_CHARS limit reached, add the null and return
        if( c == 13 || count == MAX_CHARS)
        {
            data->buffer[count] = '\0';
            return;
        }
    }
}

// stores
void parseFields(USER_DATA* data)
{
    char c = '!';
    data->fieldCount = 0;
    bool isPrevDelim = true;
    uint8_t i;
    // Loop until we reach the end of the buffer
    for(i = 0; i < MAX_CHARS; i++)
    {
        c = data->buffer[i]; // Gets the char from the buffer
        if( c == '\0')
            return;

        if(isPrevDelim)
        {
            // if char c is alpha a-z LOWERCASE
            if( c >= 'a' && c <= 'z' )
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }

            // if char c is alpha A-Z UPPERCASE
            if ( c >= 'A' && c <= 'Z' )
            {
                data->fieldType[data->fieldCount] = 'A';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }

            // if char c is numeric
            if( c >= '0' && c <= '9')
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }

            // if c is float
            if( c == '.' || c == '-' )
            {
                data->fieldType[data->fieldCount] = 'f';
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
                isPrevDelim = false;
            }
        }
        else
        {
            // only delimeters are SPACES and COMMAS
            if( c == ' ' || c == ',')
            {
                isPrevDelim = true;
                data->buffer[i] = '\0';
            }
            else if( data->fieldType[data->fieldCount-1] == 'n' && c == '.' )
                data->fieldType[data->fieldCount-1] = 'f';
        }

    }

}

// data holding command info...
// ...field# is which spot the command we want is
// 0 1 2 3...
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
        return &data->buffer[ data->fieldPosition[fieldNumber] ];
    else
        return '\0';
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    char* strValue;
    int32_t returnVal = 0;
    uint8_t i;

    if(fieldNumber <= data->fieldCount && data->fieldType[fieldNumber] == 'n')
    {
        strValue = &data->buffer[ data->fieldPosition[fieldNumber] ];

        for(i = 0; strValue[i] != '\0'; ++i)
        {
            returnVal = returnVal * 10 + (strValue[i] - '0');
        }

        return returnVal;
    }

    else
        return -1;
}

// can't use sscanf for RTOS so not currently working
float getFieldFloat(USER_DATA *data, uint8_t fieldNumber)
{
    char *strValue;
    float returnVal = 0;
	double digit_position = 10;
	uint8_t i = 0;


    if(fieldNumber <= data->fieldCount && (data->fieldType[fieldNumber] == 'f' || data->fieldType[fieldNumber] == 'n') )
    {
        strValue = &data->buffer[ data->fieldPosition[fieldNumber] ];
		

        //sscanf(strValue, "%f", &returnVal);
		for(; strValue[i] != '\0'; i++)
        {
            returnVal = returnVal * 10 + (strValue[i] - '0');
        }
		//TODO check if i is the correct offset here
		i++;
		while(strValue[i++] != '\0')
		{
			returnVal += (strValue[i] - '0') / digit_position;
			digit_position *= 10;
		}
		
    }
    return returnVal;
}

bool strcomp(char * a, char * b)
{
    int8_t i = 0;
    char c1 = a[i];
    char c2 = b[i];

    do
    {
        if(c1 != c2)
            return false;
        else
        {
            i++;
            c1 = a[i];
            c2 = b[i];
        }
    } while(c1 != NULL || c2 != NULL);
    return true;
}

bool isCommand(USER_DATA* data, char strCommand[], uint8_t minArguments)
{
    char* strCompare = &data->buffer[ data->fieldPosition[0] ];
	
	if( strcomp(strCompare, strCommand) )
	{
		if( minArguments <= data->fieldCount-1 )
			return true;
		else
		{
			/* putsUart0("ERROR: Not enough arguments for '");
			putsUart0(strCompare);
			putsUart0("'.\n"); */
			return false;
		}
	}
	else
		return false;
}

void data_flush(USER_DATA * clear)
{
    int8_t i;
    for(i = 0; i < MAX_CHARS; i++)
        clear->buffer[i] = '\0';
    clear->fieldCount = 0;
    for(i = 0; i < MAX_FIELDS; i++)
    {
        clear->fieldPosition[i] = 0;
        clear->fieldType[i] = '\0';
    }

}

bool handleCommand(USER_DATA* data)
{
    uint8_t i;
    uint32_t pid;

    /*  ======================= *
     *  ||||||| H E L P ||||||| *
     *  ======================= */
    if( isCommand(data, "help", 0) ) // always pass data, "what string do you want to check", how many args
    {
        //putsUart0("No commands implemented yet...\r");
        putsUart0("Possible commands:\n");
        putsUart0("'reboot'\r");
        putsUart0("'clear'\r");
        return true;
    }

    /*  ======================= *
     *  ||||| R E B O O T ||||| *
     *  ======================= */
    else if( isCommand(data, "reboot", 0) )
    {
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        return true; // won't really do anything
    }

    /*  ======================= *
     *  |||||| C L E A R |||||| *
     *  ======================= */
    else if( isCommand(data, "clear", 0) )
    {
        for(i = 0; i < 50; i++)
        putcUart0('\n');
        return true;
    }

    /*  ======================= *
     *  ||||||||| P S ||||||||| *
     *  ======================= */
    else if( isCommand(data, "ps", 0) )
    {
        ps();
        return true;
    }

    /*  ======================= *
     *  ||||||| I P C S ||||||| *
     *  ======================= */
    else if( isCommand(data, "ipcs", 0) )
    {
        ipcs();
        return true;
    }

    /*  ======================= *
     *  ||||||| K I L L ||||||| *
     *  ======================= */
    else if( isCommand(data, "kill", 1) )
    {
        pid = getFieldInteger(data, 1);
        kill(pid);
        return true;
    }

    /*  ======================= *
     *  ||||||| P M A P ||||||| *
     *  ======================= */
    else if( isCommand(data, "pmap", 1) )
    {
        pid = getFieldInteger(data, 1);
        pmap(pid);
        return true;
    }

    /*  ======================= *
     *  |||| P R E E M P T |||| *
     *  ======================= */
    else if( isCommand(data, "preempt", 1) )
    {
        //pid = getFieldInteger(data, 1);
        //preempt();
        return true;
    }

    return false;
}

void shell()
{
	USER_DATA data;
		
	while(1)
	{
		putcUart0('>');
		getsUart0(&data);
		parseFields(&data);

		if( handleCommand(&data) )
		{

		}
		else
		{
			putsUart0("Invalid input.\n");
		}

		putcUart0('\n');
	}
}