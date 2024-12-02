 /**
 ********************************************************************************
 * @file asmDump.s
 * @author University of Waterloo MTE 241, edited and improved by Daniel Luchian
 * @brief This assembly file was originally written by Professor Jeff Zarnett
 * for the labratory section of UWaterloo's MTE 241 course, since then
 * it has been edited to suit the requirments of this project
 ********************************************************************************
 */

  .syntax unified //Allows for C like comments
  .cpu cortex-m4 //Set CPU
  .thumb //Ensures the correct types of instructions get included

  .global SVC_Handler //Indicate to the linker that this function exists
  .thumb_func //Ensure that the address of our interrupt function is properly aligned or we hard fault
  SVC_Handler: //Function name
	TST LR, 4 //Test the 3rd bit in LR (4 is 0b1000, so its 3rd bit is 1)
	ITE EQ //If Then Equal
	MRSEQ R0, MSP //If the third bit is set, we are using MSP. Set us up to use that
	MRSNE R0, PSP //Otherwise, use PSP
	B SVCHandlerMain //Go to the C function


  .global runFirstThread //Running the first thread requires some special consideration, so it is its own function
  .thumb_func
  runFirstThread:
  	//Restore MSP since we have two things on there that won't go away
  	POP {R7}
  	POP {R7}

  	//Get ready for PSP
  	MRS R0, PSP
  	MOV LR, #0xFFFFFFFD
  	LDMIA R0!,{R4-R11}
  	MSR PSP, R0
  	BX LR

   .global PendSV_Handler //In general, we want to use PendSV for the actual context switching
   .thumb_func
   PendSV_Handler:
	//Restore MSP since we have two things on there that won't go away
	//POP {R7}
	//POP {R7}

	//Perform the switch
	MRS R0, PSP
	STMDB R0!,{R4-R11}
	BL Sched
	MRS R0, PSP
	MOV LR, #0xFFFFFFFD
	LDMIA R0!,{R4-R11}
	MSR PSP, R0
	BX LR
