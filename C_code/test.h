/*This is the test functions' head.
File name :test.h
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-06-21 15:14:9 Tom.xiao@st.com
*/

#ifndef __TEST_H
#define __TEST_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define Buffer_Byte_Len       10

/*Global declare.
*/
extern uint8_t Send_Byte_Buffer[96],
               Receive_Byte_Buffer[96];

void Test_Init(void);
void Test_Receive_Buffer(void);

#endif

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

