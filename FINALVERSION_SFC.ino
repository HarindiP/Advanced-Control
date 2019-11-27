
/*
 Example of a State Feedback Controler (SFC)
 University of Technology Sydney (UTS)
 Board: MKR1000
 Based: bAC18
 Code:  v3

 Created by Ricardo P. Aguiilera, 
            Manh (Danny) Duong Phung,

 Date: 14/04/2019
 

 Hardward tools: Extended Arduino Board for MKR1000
 Software tools: It requires the following libraries:
    Timer5
    MatrixMath
    PWM_MKR1000_AdvCtrl_UTS
 */


/******************************
 ******************************
 Libraries
 */
#include <Timer5.h>
#include <MatrixMath.h>
#include <PWM_MKR1000_AdvCtrl_UTS.h>


/******************************
 ******************************
 Definitions (constant parameters) and Variales
 */ 


//Discrete-Time LTI System To Be Controlled                              \
  x(k+1)=A·x(k)+B·u(k)                    \
    y(k)=C·x(k)                             \
                                            \
  where, x \in R^n, u \in R^m, y \in R^p    \

#define fs 50  // Sampling frequency [Hz]
                // Sampling Time, Ts=1/fs [s]

//Number of States, Inputs, and Outputs
#define n 4   //number of states 
#define m 1   //number of inputs 
#define p 2   //number of outputs

//Discrete- Time System Vectors
float x_k[n][1];  //State vector  x(k)
float u_k[m][1];  //Input vector  u(k)
float y_k[2][1];  //Output vector y(k)
float x_hat[n][1];  //hat vectors
float y_h[2][1];    //hatouts

//Discrete-Time System Matrices
float A[n][n];
float B[n][m];
float C[p][n];
float D[p][m];

//Intermediate Multiplications
float Ax_k[n][1];     //Intermediate vector A·x(k)
float Bu_k[n][1];     //Intermediate vector B·u(k)
float x_dot[n][1];  //Derivative of x
float Cx_k[p][1];
float Du_k[p][1];
float Ax_h[n][1];
float LyplusLy_h[n][1];
float x_dotminusAx_k[n][1];
float x_dotminusAx_kplusAx_h[n][1];



//SFC u(k)=-F*x(k)+r
float F[m][n];      //Designed offline
float r[m][1];      //Computed online
float M[m][p];      // r=M*y_ref
float Fx_k[m][1];
float Fx[m][1];     //Intermediate vector F·x(k)
float y_ref[n][1];  //Output Reference y*
float L[n][p];
float Ly[n][1];
float Ly_h[n][1];
float Ax_hsumBu[n][1];
float LyminusLy_h[n][1];
float x_dothat[n][1];
float Fx_hat[m][1];
float x_hat1[n][m];
float x_k1[n][m];
float y_k1[n][1];

//Inputs
float in1 = 0;
float in2 = 0;
float in3 = 0;
float in4 = 0;

//Outputs
float out1 = 0;
float out2 = 0;
float out3 = 0;
float out4 = 0;

int count = 0;


/******************************
 ******************************
 Initializations
 */
void setup()
{  
  //Initialize Serial Buse
    delay (7000);
    Serial.begin(9600); 

  //Sampling Time Ts=1/fs [s]
    float Ts=1/float(fs); //in seconds      
  
  //Reference
    y_ref[0][0]=0;      //cart position
    y_ref[1][0]=0;        
    y_ref[2][0]=0;      //swing angle
    y_ref[3][0]=0;  
    r[0][0]=0;              
    
  //Initialize matrices  
  /*  A[0][0]=1;  A[0][1]=0.002492; A[0][2]=-2.532e-5; A[0][3]=-3.166e-8;
    A[1][0]=0;  A[1][1]=0.9935; A[1][2]=-0.02026; A[1][3]=-2.532e-5;
    A[2][0]=0;  A[2][1]=-8.075e-6;  A[2][2]=0.9999; A[2][3]=0.025;
    A[3][0]=0;  A[3][1]=-0.00646;  A[3][2]=-0.04476; A[3][3]=0.9999;
  */
    A[0][0]=1;  A[0][1]=0.0195; A[0][2]=-0.0016; A[0][3]=0;
    A[1][0]=0;  A[1][1]=0.9495; A[1][2]=-0.1580; A[1][3]=-0.0016;
    A[2][0]=0;  A[2][1]=-0.0005;  A[2][2]=0.9922; A[2][3]=0.0199;
    A[3][0]=0;  A[3][1]=-0.0504;  A[3][2]=-0.7747; A[3][3]=0.9922;
  
  /*  B[0][0]=1.154e-6;
    B[1][0]=0.000923;
    B[2][0]=-1.161e-6;
    B[3][0]=-0.000929; */
    B[0][0]=0.0001;
    B[1][0]=0.0072;
    B[2][0]=-0.0002;
    B[3][0]=-0.0163;
    
  
 //   C[0][0]=1;  C[0][1]=0.001246;  C[0][2]=-1.266e-5;  C[0][3]=-1.583e-8;
   // C[1][0]=0;  C[1][1]=-4.038e-6;  C[1][2]=1;  C[1][3]=0.00125;
       C[0][0]=1;  C[0][1]=0;  C[0][2]=0;  C[0][3]=0;
    C[1][0]=0;  C[1][1]=0;  C[1][2]=1;  C[1][3]=0;


    D[0][0]=0;     
    D[1][0]=0;   
    

    F[0][0]=  1.4069   ;  F[0][1]= 0.9088 ; F[0][2]=-0.0357 ; F[0][3]=-0.4832;

    L[0][0]=0.1485  ;   L[0][1]=-0.1411;
    L[1][0]= 0.8982 ;   L[1][1]= -1.0574;
    L[2][0]= -0.1446 ;   L[2][1]=0.2281;
    L[3][0]=-1.2384 ;   L[3][1]=1.5359;
     
   
    
    u_k[0][0]=0;
    
    x_k[0][0]=0;
    x_k[1][0]=0;
    x_k[2][0]=0;
    x_k[3][0]=0;
    
    x_hat[0][0]=0.1;
    x_hat[1][0]=0.1;
    x_hat[2][0]=0.1;
    x_hat[3][0]=0.1;
    
    x_dot[0][0]=0;
    x_dot[1][0]=0;
    x_dot[2][0]=0;
    x_dot[3][0]=0;
    
    x_hat1[0][0]=0;
    x_hat1[1][0]=0;
    x_hat1[2][0]=0;
    x_hat1[3][0]=0;
    
    y_k[0][0]=0;
    y_k[1][0]=0;

   
  //Initialize I/O pins to measure execution time
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(out2,OUTPUT);
    digitalWrite(out2,LOW); 

  //ADC Resolution                                                      \
    The Due, Zero and MKR Family boards have 12-bit ADC capabilities    \
    that can be accessed by changing the resolution to 12.              \
    12 bits will return values from analogRead() between 0 and 4095.    \
    11 bits will return values from analogRead() between 0 and 2047.    \
    10 bits will return values from analogRead() between 0 and 1023.    \
    Default resolution if not used is 10bits.

    int res=12;
    analogReadResolution(res); //If commented, default resolution is 10bits

   
  // Configure PWM for outputs
    init_PWM_MKR1000_UTS();
  
  // define timer5 interruption freq (sampling time)
    MyTimer5.begin(fs);   //Argument is freq in Hz

  // define the interrupt callback function
    MyTimer5.attachInterrupt(Controller);
  
  // start the timer
    MyTimer5.start();   //Always start timer at the end
    
}





void Controller(void) {
/******************************
 ******************************
 Timer Interruption
 The code inside this section will be run at every Ts
 */

  //Start measuring execution time
 

  
/*
______________________
Board Inputs
______________________
*/
    //It is possible to adjust offset and gain of
    //the measurements. Also, disp = 1 will display
    //individual input in serial monitor
    
    //read_inputx(offset, gain, disp)


    in3 = read_input3(-2.5, 0.5, 0);     // -12v -> 12v
    in4 = read_input4(-1, 0.5, 0); 

    
   disp_inputs_all();
//Only display inputs for calibration. 
//Do not display them when running the controller

  /*___________________________
State Space Model Eqna 
___________________________
*/

  y_k[0][0]=in3;  //x1(k)
  y_k[1][0]=in4;    //in4

//Angle Deadzone
  if (y_k[1][0] < 0.08 && y_k[1][0] > -0.02)
  {
    y_k[1][0] = 0; 
  }










/*___________________________
State Feedback Controller
___________________________
*/


    



     /*___________________________
State Observer Controller
___________________________
*/
//Setting Integrators
 x_hat[0][0] =x_hat1[0][0];
 x_hat[1][0] =x_hat1[1][0];
 x_hat[2][0] =x_hat1[2][0];
 x_hat[3][0] =x_hat1[3][0];

//Mathematical Calculations
Matrix.Multiply((float*)B, (float*)u_k, n, m, 1, (float*)Bu_k);
Matrix.Multiply((float*)F, (float*)y_ref, m, n, 1, (float*)r);
Matrix.Multiply((float*)C, (float*)x_hat, p, n, 1, (float*)y_h);
Matrix.Multiply((float*)A, (float*)x_hat, n, n, 1, (float*)Ax_h);
Matrix.Add((float*)Ax_h, (float*)Bu_k, n, 1, (float*)Ax_hsumBu);
Matrix.Multiply((float*)L, (float*)y_k, n, p, 1, (float*)Ly);
Matrix.Multiply((float*)L, (float*)y_h, n, p, 1, (float*)Ly_h);
Matrix.Subtract((float*)Ly, (float*)Ly_h, n, 1, (float*)LyminusLy_h);
Matrix.Add((float*)Ax_hsumBu, (float*)LyminusLy_h, n, 1, (float*)x_dothat);                                                                             
Matrix.Multiply((float*)F, (float*)x_hat, m, n, 1, (float*)Fx_hat);   

  Serial.print("Fx:");
  Serial.print(Fx_hat[0][0], 5);
  Serial.print("r:");
  Serial.print(r[0][0], 5);
  Serial.print("y_ref:");
  Serial.print(y_ref[0][0], 5);
  Serial.print("x_hat:");
  Serial.print(x_hat[0][0]); 
  Serial.print("x_dothat:"); 
  Serial.print(x_dothat[0][0], 5); 

//CL Input
  for (int i = 0; i < m; i++)
    {
      u_k[i][0]=-Fx_hat[i][0]+r[i][0];
    }

 //Presetting Integrators   
  x_hat1[0][0]=x_dothat[0][0]; 
  x_hat1[1][0]=x_dothat[1][0];  
  x_hat1[2][0]=x_dothat[2][0];  
  x_hat1[3][0]=x_dothat[3][0]; 


    

//DeadZone
    if (u_k[0][0]<1.75 && u_k[0][0]> 1)
       {
        out1=0;
       }
     else if (u_k[0][0]>-2.33 && u_k[0][0]<1)
       {
        out1 =-2.4+0.48*u_k[0][0]; 
       }
     else if (u_k[0][0]<2.33 && u_k[0][0]>1)
       {
        out1 = 2.5+0.48*u_k[0][0]; 
       }
     else
       {
        out1=u_k[0][0];
       }
  Serial.print("UK:");
  Serial.print(u_k[0][0], 5);
//Finally, assign each controller output to one board output
      

      

    
   
    
 
/*
______________________
Board Outputs
______________________
*/
    //It is possible to adjust offset and gain of
    //each output. Also, disp = 1 will display
    //individual output in serial monitor
     
  //write_outx(value, offset, gain, disp) 
   write_out1(out1, 0,  1, 1);   // Pin 5  -12V to 12V
   //con2

   // disp_outputs_all();
//Only display outputs for calibration. 
//Do not display them when running the controller   

  //Stop measuring calculation time
  digitalWrite(out2,0);
 
  
     
}






//Main loop does nothing
void loop() { 
  

  //Left intentionally empty

  }






/*
______________________
Functions
______________________
*/

/*
 Display Inputs
 */
 void disp_inputs_all()
{
      Serial.print("In1: ");
      Serial.print(in1);
      Serial.print(" [V]  ");
      Serial.print("In2: ");
      Serial.print(in2);
      Serial.print(" [V]  ");
      Serial.print("In3: ");
      Serial.print(in3, 7);
      //Serial.print(" [V]  ");
      Serial.print("In4: ");
      Serial.print(in4, 7);
      //Serial.println(" [V]");
}

/*
 Display Outputs
 */
/* void disp_outputs_all()
{
      Serial.print("Out1: ");
      Serial.print(out1);
      Serial.print(" [V]  ");
      Serial.print("Out2: ");
      Serial.print(out2);
      Serial.print(" [V]  ");
      Serial.print("Out3: ");
      Serial.print(out3);
      Serial.print(" [V]  ");
      Serial.print("Out4: ");
      Serial.print(out4);
      Serial.println(" [V]");
}*/

/*
 Read Inputs
 */
float read_input1(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A1); // Read input 1 (0 –> 12V)
  in_float = (float)(in)*0.00287-0.11;
  in_float = in_float*gain + offset;
  //in_float = (float)(in);

  if (disp==1){
      Serial.print("In1: ");
      Serial.print(in1);
      Serial.println(" [V]");
  }
  
  return in_float; 
}

float read_input2(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A2); // Read input 2 (0 –> 12V)
  in_float = (float)(in)*0.00287-0.11;
  in_float = in_float*gain + offset;
  //in_float = (float)(in);

  if (disp==1){
      Serial.print("In2: ");
      Serial.print(in2);
      Serial.println(" [V]");
  }

  return in_float; 
}

float read_input3(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A5); // Read input 3 (0 -> +/-12)
  
  in_float = (float)(in)-2044;
  in_float = in_float*0.00617;
  in_float = (in_float+ offset)*gain ;

    if (disp==1){
      Serial.print("In3: ");
      Serial.print(in3);
      Serial.println(" [V]");
  }

  return in_float; 
}

float read_input4(float offset, float gain, int disp) {
  float in_float = 0;
  int in = analogRead(A6); // Read input 4 (0 -> +/-12)
  in_float = (float)(in)-2044;
  in_float = in_float*0.00617;
  in_float = (in_float+ offset)*gain ;

    if (disp==1){
      Serial.print("In4: ");
      Serial.print(in4);
      Serial.println(" [V]");
  }
  
 return in_float;
}



/*
 Write Outputs
 */

void write_out1(float out, float offset, float gain, int disp) // 0.3533  0.0394
{
  //CON 5
  float d=(out-0.3533+offset)*0.0394*gain + 0.5;
   if (d<0)
     d=0;
    REG_TCC0_CCB1= (int)((REG_TCC0_PER+1)*d);   
    if (disp==1){
   //  out1 = 1.2;
      Serial.print("Out1 : ");
      Serial.print((float)out1,1);
      Serial.println(" [V]");
  }
 }

void write_out2(float out, float offset, float gain, int disp)
{
   //Pin 4
    float d=(out-0.3933+offset)*0.0394*gain+0.5;
    if (d<0)
      d=0;
    REG_TCC0_CCB0= (int)((REG_TCC0_PER+1)*d); 
    if (disp==1){
      Serial.print("Out2 : ");
      Serial.print(out2);
      Serial.println(" [V]");
  }

}

void write_out3(float out, float offset, float gain, int disp)
{
   //Pin 7
    float d = (out+offset)*0.076*gain;
    if (d<0)
      d=0;
    REG_TCC0_CCB3 = (int)((REG_TCC0_PER+1)*d); 
    if (disp==1){
      Serial.print("Out3 : ");
      Serial.print(out3);
      Serial.println(" [V]");
  }

}

void write_out4(float out, float offset, float gain, int disp)
{
   //Pin 6
    float d = (out+offset)*0.076*gain;
    if (d<0)
      d=0;
    REG_TCC0_CCB2 = (int)((REG_TCC0_PER+1)*d); 
    if (disp==1){
      Serial.print("Out4 : ");
      Serial.print(out4);
      Serial.println(" [V]");
  }
 
}

/*PWM Explanation
The sawtooth carrier incrases from 0 to Tmax

Tmax = REG_TCC0_PER + 1

Tmax    _____________________
             /|     /|     /|
            / |    / |----/-|
CMP_i   ---/--|   /  |   /  |
          /   |  /   |  /   |
         /    |-/----| /    |
    0   /     |/     |/     └───────> t
       |      |      |      |
       |      |      |      |
PWM i Output  |      |      |
       ┌──┐   ┌┐     ┌────┐ 
       │  │   ││     │    │ 
       │  │   ││     │    │ 
       ┘  └───┘└─────┘    └───────> t
*/
