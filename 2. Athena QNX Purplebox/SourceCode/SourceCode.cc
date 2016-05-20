/******************************************************************************
 * Simulated Multi-Thread PWM Controlled Servo Operation
 *
 * Description:
 *
 * A standalone program exhibiting multitasking characteristics by simultaneously
 *
 * controlling a pair of servo motors using a custom interpreted control language.
 *
 * The system will be responsive to simultaneous independent, externally provided
 *
 * commands. The servo positions are controlled with pulse-width modulation (PWM)
 *
 * Author:
 *  		Siddharth Ramkrishnan (sxr4316@rit.edu)	: 05.16.2016
 *
 *****************************************************************************/

#include <sys/neutrino.h>
#include <sys/syspage.h>
#include <sys/netmgr.h>
#include <sys/mman.h>
#include <hw/inout.h>
#include <pthread.h>
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>
#include <queue>

#define CLOCKPERIOD	200000

// OPCODE Definition

#define RECIPE_END  0x00

#define MOV         0x20

#define WAIT        0x40

//-----------------------------------------------

#define	FLIP		0x60     // GRAD Extension

//-----------------------------------------------

#define LOOP        0x80

#define END_LOOP    0xA0

//-----------------------------------------------

#define	SKIP		0xC0     // GRAD Extension

//-----------------------------------------------

#define UNUSED      0xE0



//Servo Position Selector Macros

#define POS0  125

#define POS1  126

#define POS2  127

#define POS3  128

#define POS4  129

#define POS5  130

// User Defined Macros & Static variables for use in model

#define RANGECHECK(POS)    ((POS<125)?125:((POS>130)?130:POS))

typedef struct
{
	unsigned char OpCodeError, MovError, LoopError, End_Recipe;
	unsigned char LEDStatus, LEDErrDisp;
	unsigned char Start_LoopInstruction;
	unsigned char Instruction_Counter;
	unsigned char Finish_Instruction;
	unsigned char InstructionSet[30];
	unsigned char Pause_Instruction;
	unsigned char Start_Instruction;
	unsigned char Current_Position;
	unsigned char PositionTemp;
	unsigned char Loop_Counter;
	unsigned char WaitCounter;
	unsigned char MoveCounter;
}ServoCtrlSet;

ServoCtrlSet A,B;


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

static unsigned char InstructionSet1[] ={MOV+0,MOV+5,MOV+0,MOV+3,LOOP+4,MOV+1,MOV+4,END_LOOP,MOV+0,MOV+2,WAIT+0,MOV+3,WAIT+0,MOV+2,MOV+3,WAIT+31,WAIT+31,WAIT+31,MOV+4,RECIPE_END};

static unsigned char InstructionSet2[] ={MOV+5,MOV+0,MOV+5,MOV+2,LOOP+4,MOV+4,MOV+1,END_LOOP,MOV+5,MOV+3,WAIT+0,MOV+2,WAIT+0,MOV+3,MOV+2,WAIT+31,WAIT+31,WAIT+31,MOV+1,RECIPE_END};

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define CLKPERIODNSPERCENT		20000000

static unsigned char 			EvalFlag = 0;

static unsigned char			PwmDtyCycA = 7,	PwmDtyCycB = 7;

static unsigned char			StrdPortA ;

static	pthread_t				RoutineEvaluation, PwmControlA, PwmControlB;

static	timespec				PwmAOnTime, PwmAOffTime, PwmBOnTime, PwmBOffTime;

static	uintptr_t				PortAHandle, PortBHandle, PortCHandle, DIOCtlHandle;

void hardwareinit()
{
	// Grant thread access to hardware registers

	(void)ThreadCtl( _NTO_TCTL_IO, NULL );

	PortAHandle			=	mmap_device_io(1,0x288);

	PortBHandle			=	mmap_device_io(1,0x289);

	PortCHandle			=	mmap_device_io(1,0x28A);

	DIOCtlHandle		=	mmap_device_io(1,0x28B);

	// Indicate only Channel 0 has to be Analog to Digital Converted

	(void)out8(mmap_device_io(1,0x282),	0x00);

	// Indicate all DIO ports except Port C are output ports

	(void)out8(DIOCtlHandle,	0x00);

	(void)out8(PortAHandle,		0x00);

	(void)out8(PortBHandle,		0x00);

	// Reset ADC Trigger Port

	(void)out8(PortCHandle,		0x00);
}

//
// Actuate PWM0 Singal to generate the duty cycle for selected motor position
//
//---------------------------------------------------------------------------

void Pwm0PulsCtrl()
{
   switch (A.Current_Position)
   {
    case POS0  : PwmDtyCycA	=	2;
                          break;
    case POS1  : PwmDtyCycA	=	4;
                          break;
    case POS2  : PwmDtyCycA	=	6;
                          break;
    case POS3  : PwmDtyCycA	=	8;
                          break;
    case POS4  : PwmDtyCycA	=	10;
                          break;
    case POS5  : PwmDtyCycA	=	12;
                          break;
    default   : PwmDtyCycA	=	7;
   }
}

//
// Actuate PWM1 Singal to generate the duty cycle for selected motor position
//
//---------------------------------------------------------------------------

void Pwm1PulsCtrl()
{
    switch (B.Current_Position)
    {
      case POS0: PwmDtyCycB	=	2;
                          break;
      case POS1: PwmDtyCycB	=	4;
                          break;
      case POS2: PwmDtyCycB	=	6;
                          break;
      case POS3: PwmDtyCycB	=	8;
                          break;
      case POS4: PwmDtyCycB	=	10;
                          break;
      case POS5: PwmDtyCycB	=	12;
                          break;
      default : PwmDtyCycB	=	7;
                          break;
   }
}

//
// Permanent ligthup of LEDs based on error status, toggle other LEDs every function call
//
//---------------------------------------------------------------------------------------

void LED_Action()
{
  StrdPortA = ~StrdPortA;

  if(A.OpCodeError)
    StrdPortA = StrdPortA|0x08;

  if(A.LoopError)
    StrdPortA = StrdPortA|0x04;

  if(A.End_Recipe)
    StrdPortA = StrdPortA|0x02;

  if(A.Pause_Instruction)
    StrdPortA = StrdPortA|0x01;

  if(B.OpCodeError)
    StrdPortA = StrdPortA|0x80;

  if(B.LoopError)
    StrdPortA = StrdPortA|0x40;

  if(B.End_Recipe)
    StrdPortA = StrdPortA|0x20;

  if(B.Pause_Instruction)
    StrdPortA = StrdPortA|0x10;

  StrdPortA = StrdPortA;
}


//
// Compare the target position to current Servo Position and delay number
//  of cycles corresponding to difference between current position
//  and target position
//
//---------------------------------------------------------------------------

void MOV_Action(ServoCtrlSet *ptr, unsigned char target_position)
{

  // Set signal out of range the first time , an out of range target position is detected.

 if((target_position<POS0 ||target_position>POS5)&&(ptr->MovError ==0))
 {
  (void)printf("\n\r Move Argument Out of Range \r\n") ;

  ptr->MovError = 1;
 }

 ptr->PositionTemp        = ptr->Current_Position;

 ptr->Current_Position    = RANGECHECK(target_position);

 // At first evaluation, set the current position to static variable to monitor the difference

 if(ptr->Finish_Instruction ==  1)
 {

  ptr->MoveCounter      	  = ptr->PositionTemp;

  ptr->Finish_Instruction 	= 0;

 }

 // Increment / Decrement the static counter until target position is reached

 if(ptr->MoveCounter > target_position)
 {

	ptr->MoveCounter--;

 }
 else if (ptr->MoveCounter<target_position)
 {

	ptr->MoveCounter++;

 }
 else
 {

  ptr->Finish_Instruction = 1;

 }

}


//
// System stays idle , without change in output for the number of function calls
//  based on argument value passed
//---------------------------------------------------------------------------

void WAIT_Action(ServoCtrlSet *ptr,unsigned char Op_Counter)
{
  ptr->WaitCounter            = (ptr->WaitCounter==0)?0:(ptr->WaitCounter - 1);

  if(ptr->Finish_Instruction == 1)
  {
    ptr->WaitCounter          = Op_Counter ;

    ptr->Finish_Instruction   = 0;
  }

  if(ptr->WaitCounter==0)
  {
    ptr->Finish_Instruction   = 1;
  }
  else
  {
    ptr->Finish_Instruction   = 0;
  }
}


//--------------------------------------------------------------------------------
void FLIP_Action(ServoCtrlSet *ptr)
{

  switch(ptr->Current_Position)
  {
    case  POS0:(void)MOV_Action(ptr,POS5);
                    break;
    case  POS1:(void)MOV_Action(ptr,POS4);
                    break;
    case  POS2:(void)MOV_Action(ptr,POS3);
                    break;
    case  POS3:(void)MOV_Action(ptr,POS2);
                    break;
    case  POS4:(void)MOV_Action(ptr,POS1);
                    break;
    case  POS5:(void)MOV_Action(ptr,POS0);
                    break;

  }
}
//--------------------------------------------------------------------------------

void LoopStart(ServoCtrlSet *ptr, unsigned char loop_counter)
{

  if(ptr->Loop_Counter  ==  0)
  {
    ptr->Start_LoopInstruction  = ptr->Instruction_Counter + 1;

    ptr->Loop_Counter           = loop_counter;

    ptr->Finish_Instruction     = 1;

    ptr->LoopError =0;
  }
  else if((ptr->OpCodeError !=1)&&(ptr->LoopError !=1)&&(ptr->MovError !=1))
  {
	  ptr->LoopError =1;
  }
}

void LoopEnd(ServoCtrlSet *ptr)
{

 if(ptr->Loop_Counter !=  0)
  {
    ptr->Instruction_Counter    = ptr->Start_LoopInstruction  ;

    ptr->Loop_Counter           = ptr->Loop_Counter - 1 ;
  }

 ptr->Finish_Instruction        = 1;
}


//--------------------------------------------------------------------------------
void SKIP_Action(ServoCtrlSet *ptr,unsigned char Op_Counter)
{
  ptr->Instruction_Counter    = ptr->Instruction_Counter +  Op_Counter ;

}
//--------------------------------------------------------------------------------

void ServoRoutineEval(ServoCtrlSet *ptr)
{

  if(ptr->Start_Instruction == 1)
  {
    ptr->Finish_Instruction = 1;

    ptr->Start_Instruction = 0;

    ptr->Instruction_Counter = 0;

  }else if (ptr->Pause_Instruction == 0)
  {

   if((ptr->OpCodeError !=1)&&(ptr->LoopError !=1)&&(ptr->MovError !=1))
   {
    switch(ptr->InstructionSet[ptr->Instruction_Counter]&0xE0)
    {
      case  RECIPE_END  : //Recipe End OpCode Selected
                   ptr->Finish_Instruction  = 0;
                   ptr->End_Recipe          = 1;
                   break;

      case  MOV  		: //Move OpCode Selected
                  (void)MOV_Action(ptr,125+(ptr->InstructionSet[ptr->Instruction_Counter]&0x1F));
                    break;

      case  WAIT  		:  // Wait OpCode Selected
                  (void)WAIT_Action(ptr,(ptr->InstructionSet[ptr->Instruction_Counter])&0x1F);
                    break;

      case FLIP       :   // GRAD Extension : Flip Command
                  (void)FLIP_Action(ptr);
                    break;

      case  LOOP  		:  // Loop_Start OpCode Selected
                  (void)LoopStart(ptr,(ptr->InstructionSet[ptr->Instruction_Counter])&0x1F);
                    break;

      case  END_LOOP  	:  // Loop_End OpCode Selected
                  (void)LoopEnd(ptr);
                    break;

      case  SKIP  		:  // GRAD Extension : Skip Command
                  (void)SKIP_Action(ptr,(ptr->InstructionSet[ptr->Instruction_Counter])&0x1F);
                    break;

      default     : // Invalid OpCode Detected
                  (void)printf("\r\n Error Operation : Invalid OpCode detected in Recipe for Servo A\r\n");
                  ptr->OpCodeError = 1;
                    break;
    }

    if((ptr->Finish_Instruction==1)&&((ptr->InstructionSet[ptr->Instruction_Counter] & (0xE0)) != 0x00))
    {
      ptr->Instruction_Counter = ptr->Instruction_Counter + 1;
    }
   }
  }
}

void* SystemEvalFunction( void* arg )
{
  while(1)
  {
  if(1)
  {
      EvalFlag 	=	1;

      ServoRoutineEval(&A);

      ServoRoutineEval(&B);

      LED_Action();
  }

  Pwm0PulsCtrl();

  Pwm1PulsCtrl();

  usleep(100000);

  }

  return arg;

}

void* PwmAEvalFunction(void* arg)
{
	while(1)
	{
		PwmAOffTime.tv_nsec	=	(100 - PwmDtyCycA)	*	 CLOCKPERIOD	;

		usleep((100 - PwmDtyCycA)*200);

		out8(PortAHandle,0xFF);

		PwmAOnTime.tv_nsec	=		PwmDtyCycA		*	 CLOCKPERIOD	;

		usleep(PwmDtyCycA*200);

		out8(PortAHandle,0x00);
	}

	return arg;
}

void* PwmBEvalFunction(void* arg)
{
	while(1)
	{
		PwmBOffTime.tv_nsec	=	(100 - PwmDtyCycB)	*	 CLOCKPERIOD	;

		usleep((100 - PwmDtyCycB)*200);

		out8(PortBHandle,0xFF);

		PwmBOnTime.tv_nsec	=		PwmDtyCycB		*	 CLOCKPERIOD	;

		usleep(PwmDtyCycB*200);

		out8(PortBHandle,0x00);
	}

	return arg;
}


int main()
{

	unsigned char i,inp[3];

	static int thread, policy;

	static struct sched_param parameters;

	static pthread_attr_t threadAttributes;

	(void)hardwareinit();

	(void) pthread_attr_init(&threadAttributes);

	(void) pthread_getschedparam(pthread_self(), &policy, &parameters);

	parameters.sched_priority--;

	pthread_attr_setschedparam(&threadAttributes, &parameters);

	thread = pthread_create(&RoutineEvaluation, &threadAttributes, SystemEvalFunction, NULL);

	parameters.sched_priority++;

	parameters.sched_priority++;

	pthread_attr_setschedparam(&threadAttributes, &parameters);

	thread = pthread_create(&PwmControlA, NULL , PwmAEvalFunction, NULL);

	thread = pthread_create(&PwmControlB, NULL , PwmBEvalFunction, NULL);

	 A.Finish_Instruction	=	1;

	 A.End_Recipe        	=	0;

	 A.Pause_Instruction  	=	1;

	 A.Current_Position		=	POS0;

	 B.Finish_Instruction	=	1;

	 B.End_Recipe        	=	0;

	 B.Pause_Instruction	=	1;

	 B.Current_Position		=	POS0;

	 for(i=0;InstructionSet1[i]!=0;i++)
	 {
		 A.InstructionSet[i] =InstructionSet1[i];
	 }
	 A.InstructionSet[i] =InstructionSet1[i];

	  for(i=0;InstructionSet2[i]!=0;i++)
	 {
		 B.InstructionSet[i] =InstructionSet2[i];
	 }
	 B.InstructionSet[i] =InstructionSet2[i];

	 while(1)
	  {
		(void)printf("\r\n Enter commands for Servo Motors: >");

		inp[0] = getchar();

		(void)printf("%c",inp[0]);

		if(inp[0]=='X' || inp[0]=='x')
		  break;

		inp[1] = getchar();

		(void)printf("%c",inp[1]);

		if(inp[0]=='X' || inp[0]=='x')
		  break;

		inp[2] = getchar();

		(void)printf("%c",inp[2]);

		if(inp[2]=='X' || inp[2]=='x')
		  break;

		if(0)
		{
		  (void)printf("\r\nInstruction not terminated with Return Key\n\r");
		} else
		{
		  switch(inp[0])
		  {
			case  'B': //  Character Input B
			case  'b': //  Character Input b

					  (void)printf("\r\nServo A : Restart Routine");

					   A.Current_Position     = POS0;

					   A.Finish_Instruction   = 1;

					   A.Instruction_Counter  = 0;

					   A.End_Recipe        = 0;

					   A.OpCodeError  = 0  ;

					   A.LoopError    = 0 ;

					   A.MovError     = 0 ;

					   StrdPortA = StrdPortA&0xF0;

					   break;

			case  'C': //  Character Input C
			case  'c': //  Character Input c

					   (void)printf("\r\nServo A : Continue Routine");

					   if(A.Pause_Instruction == 1)
						 A.Pause_Instruction  = 0;
					   else
						 (void)printf("\r\n Servo A Recipe not in PAUSE state\r\n");

					   break;

			case   'L': //  Character Input L
			case   'l': //  Character Input l

					   (void)printf("\r\nServo A : Move Left by 1 Step");

					   if(A.Pause_Instruction == 1 || A.Current_Position == POS5)
						 A.Current_Position = RANGECHECK(A.Current_Position+1);
					   else
						 (void)printf("\r\n Servo A Recipe in neither in PAUSE state nor the servo in extreme position\r\n");

					   break;

			case   'N':
			case   'n':  break;  //Do Nothing

			case   'P': //  Character Input P
			case   'p': //  Character Input p

					   (void)printf("\r\nServo A : Pause Routine");

					   if(A.Pause_Instruction != 1)
							  A.Pause_Instruction = 1;
					   else
						 (void)printf("\r\n Servo A Recipe already in PAUSE state\r\n");

					   break;

			case   'R': //  Character Input R
			case   'r': //  Character Input r

					   (void)printf("\r\nServo A : Move Right by 1 Step");

					   if(A.Pause_Instruction == 1 || A.Current_Position == POS0)
						 A.Current_Position = RANGECHECK(A.Current_Position-1);
					   else
						 (void)printf("\r\n Servo A Recipe in neither in PAUSE state nor the servo in extreme position\r\n");

					   break;

			default:  (void)printf("\r\n Invalid Charcter Input for Servo Motor A\r\n");
		  }

		  switch(inp[1])
		  {
			case  'B': //  Character Input B
			case  'b': //  Character Input b

					   (void)printf("\r\nServo B : Restart Routine");

					   B.Current_Position     = POS0;

					   B.Finish_Instruction   = 1;

					   B.Instruction_Counter  = 0;

					   B.End_Recipe        = 0;

					   B.OpCodeError  = 0  ;

					   B.LoopError    = 0 ;

					   B.MovError     = 0 ;

					   StrdPortA = StrdPortA&0x0F;

					   break;

			case  'C': //  Character Input C
			case  'c': //  Character Input c

					   (void)printf("\r\nServo B : Continue Routine");

					   if(B.Pause_Instruction == 1)
						 B.Pause_Instruction  = 0;
					   else
						 (void)printf("\r\n Servo B Recipe not in PAUSE state\r\n");

					   break;

			case   'L': //  Character Input L
			case   'l': //  Character Input l

					   (void)printf("\r\nServo B : Move Left by 1 Step");

					   if(B.Pause_Instruction == 1 || B.Current_Position == POS5)
						 B.Current_Position = RANGECHECK(B.Current_Position+1);
					   else
						 (void)printf("\r\n Servo B Recipe in neither in PAUSE state nor the servo in extreme position\r\n");

					   break;

			case   'N':
			case   'n':  break;  //Do Nothing

			case   'P': //  Character Input P
			case   'p': //  Character Input p

					   (void)printf("\r\nServo B : Pause Routine");

					   if(B.Pause_Instruction != 1)
							  B.Pause_Instruction = 1;
					   else
						 (void)printf("\r\n Servo B Recipe already in PAUSE state\r\n");

					   break;

			case   'R': //  Character Input R
			case   'r': //  Character Input r

					   (void)printf("\r\nServo B : Move Right by 1 Step");

					   if(B.Pause_Instruction == 1 || B.Current_Position == POS0)
						 B.Current_Position = RANGECHECK(B.Current_Position-1);
					   else
						 (void)printf("\r\n Servo B Recipe in neither in PAUSE state nor the servo in extreme position\r\n");

					   break;

			default:  (void)printf("\r\n Invalid Charcter Input for Servo Motor B\r\n");
		  }
		}
	  };
}
