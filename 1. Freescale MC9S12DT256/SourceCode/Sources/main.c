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

#include "userdefio.h" /* derivative-specific definitions */

// OPCODE Definition

#define RECIPE_END  0x00

#define MOV         0x20

#define WAIT        0x40

//-----------------------------------------------

#define	FLIP		    0x60     // GRAD Extension

//-----------------------------------------------

#define LOOP        0x80

#define END_LOOP    0xA0

//-----------------------------------------------

#define	SKIP		    0xC0     // GRAD Extension

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
	UINT8 OpCodeError, MovError, LoopError, End_Recipe;
	UINT8 LEDStatus, LEDErrDisp;
	UINT8 Start_LoopInstruction;
	UINT8 Instruction_Counter;
	UINT8 Finish_Instruction;
	UINT8 InstructionSet[30];
	UINT8 Pause_Instruction;
	UINT8 Start_Instruction;
	UINT8 Current_Position;
	UINT8 PositionTemp;
	UINT8 Loop_Counter;
	UINT8 WaitCounter;
	UINT8 MoveCounter;
}ServoCtrlSet;

ServoCtrlSet A,B;


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

static UINT8 InstructionSet1[] ={MOV+0,MOV+5,MOV+0,MOV+3,UNUSED+4,MOV+1,MOV+4,END_LOOP,MOV+0,MOV+2,WAIT+0,MOV+3,WAIT+0,MOV+2,MOV+3,WAIT+31,WAIT+31,WAIT+31,MOV+4,RECIPE_END};

static UINT8 InstructionSet2[] ={MOV+5,MOV+0,MOV+5,MOV+2,LOOP+4,MOV+14,MOV+1,END_LOOP,MOV+5,MOV+3,WAIT+0,MOV+2,WAIT+0,MOV+3,MOV+2,WAIT+31,WAIT+31,WAIT+31,MOV+1,RECIPE_END};




//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------


UINT8 ISRExecutionCntr = 0, EvalFlag = 0;

//
// Actuate PWM0 Singal to generate the duty cycle for selected motor position
//
//---------------------------------------------------------------------------       

void Pwm0PulsCtrl()
{
   InitializePWM0();

   switch (A.Current_Position)
   {
    case POS0  : PWMDTY0=0x04;
                          break;
    case POS1  : PWMDTY0=0x08;
                          break;
    case POS2  : PWMDTY0=0x0C;
                          break;
    case POS3  : PWMDTY0=0x10;
                          break;
    case POS4  : PWMDTY0=0x14;
                          break;
    case POS5  : PWMDTY0=0x18;
                          break;
    default   : PWMDTY0=0x04;                     
   }
}

//
// Actuate PWM1 Singal to generate the duty cycle for selected motor position
//
//---------------------------------------------------------------------------       

void Pwm1PulsCtrl()
{
    InitializePWM1();
  
    switch (B.Current_Position)
    {
      case POS0: PWMDTY1=0x04;
                          break;
      case POS1: PWMDTY1=0x08;
                          break;
      case POS2: PWMDTY1=0x0C;
                          break;
      case POS3: PWMDTY1=0x10;
                          break;
      case POS4: PWMDTY1=0x14;
                          break;
      case POS5: PWMDTY1=0x18;
                          break;
      default : PWMDTY1=0x04;
                          break;
   }
}

//
// Permanent ligthup of LEDs based on error status, toggle other LEDs every function call
//
//---------------------------------------------------------------------------------------       

void LED_Action()
{
  UINT8 StrdPortA ;
  
  StrdPortA = ~PORTA;
  
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
  
  PORTA = StrdPortA;
}


//
// Compare the target position to current Servo Position and delay number 
//  of cycles corresponding to difference between current position 
//  and target position
//
//---------------------------------------------------------------------------       

void MOV_Action(ServoCtrlSet *ptr, UINT8 target_position)
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

void WAIT_Action(ServoCtrlSet *ptr,UINT8 Op_Counter)
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

void LoopStart(ServoCtrlSet *ptr, UINT8 loop_counter)
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
void SKIP_Action(ServoCtrlSet *ptr,UINT8 Op_Counter)
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



#pragma push

#pragma CODE_SEG __SHORT_SEG NON_BANKED

void interrupt 9 OC1_isr( void )
{ 

  if(((ISRExecutionCntr++)%2) == 0)
  {
      EvalFlag 	=	1;
   
      ServoRoutineEval(&A);
    
      ServoRoutineEval(&B);
   
      LED_Action();
  }

  Pwm0PulsCtrl();
 
  Pwm1PulsCtrl();
 
  TFLG1     =   TFLG1_C1F_MASK;
}

#pragma pop
  
 
void main(void)
{
 
 UINT8 i,inp[3];
 
 A.Finish_Instruction=1;
 
 A.End_Recipe        = 0;
 
 A.Pause_Instruction  = 1;
 
 A.Current_Position = POS0;
 
 B.Finish_Instruction=1;
 
 B.End_Recipe        = 0;
 
 B.Pause_Instruction  = 1;
 
 B.Current_Position = POS0;
 
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
 
 (void)hardwareinit();
 
 while(1)
 {
    (void)printf("\r\n Enter commands for Servo Motors: >");
    
    inp[0] = GetChar();
    
    (void)printf("%c",inp[0]);
    
    if(inp[0]=='X' || inp[0]=='x')
      break;
    
    inp[1] = GetChar();
    
    (void)printf("%c",inp[1]);
    
    if(inp[0]=='X' || inp[0]=='x')
      break;
    
    inp[2] = GetChar();
    
    (void)printf("%c",inp[2]);
    
    if(inp[2]=='X' || inp[2]=='x')
      break;    
    
    if(inp[2]!=13)
    {
      (void)printf("\r\nInstruction not terminated with Return Key\n\r");
    } else
    {
      switch(inp[0])
      {
        case  66: //  Character Input B
        case  98: //  Character Input b
                   (void)printf("\r\nServo A : Restart Routine");
                   
                   A.Current_Position     = POS0;
                   
                   A.Finish_Instruction   = 1;
 
                   A.Instruction_Counter  = 0;
                   
                   A.End_Recipe        = 0;
                   
                   A.OpCodeError  = 0  ;
                   
                   A.LoopError    = 0 ;
                   
                   A.MovError     = 0 ;
                   
                   PORTA = PORTA&0xF0;
                   
                   break;
        
        case  67: //  Character Input C
        case  99: //  Character Input c
        
                   (void)printf("\r\nServo A : Continue Routine");
                   
                   if(A.Pause_Instruction == 1)
                     A.Pause_Instruction  = 0;
                   else
                     (void)printf("\r\n Servo A Recipe not in PAUSE state\r\n");
                   
                   break;
                   
        case   76: //  Character Input L
        case  108: //  Character Input l

                   (void)printf("\r\nServo A : Move Left by 1 Step");
                   
                   if(A.Pause_Instruction == 1 || A.Current_Position == POS5)
                     A.Current_Position = RANGECHECK(A.Current_Position+1);
                   else
                     (void)printf("\r\n Servo A Recipe in neither in PAUSE state nor the servo in extreme position\r\n");
                                      
                   break;
                   
        case   78:
        case  110:  break;  //Do Nothing
                   
        case   80: //  Character Input P
        case  112: //  Character Input p
        
                   (void)printf("\r\nServo A : Pause Routine");
                   
                   if(A.Pause_Instruction != 1)
                          A.Pause_Instruction = 1;
                   else 
                     (void)printf("\r\n Servo A Recipe already in PAUSE state\r\n");
                   
                   break;                   
                   
        case   82: //  Character Input R
        case  114: //  Character Input r
                   
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
                   
                   PORTA = PORTA&0x0F;
                   
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
