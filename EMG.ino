// This Arduino code is written for MaM Sense Board EMG output. For more info visit: https://mamhightech.com/MamSense.html
// The program detects hand movements and sends commands to actuators. It creates notch filter to suppress AC power line noise at 50 Hz
//(60 Hz if you are located in North and South America.)
// For the full table: https://en.wikipedia.org/wiki/Mains_electricity_by_country.
// If single frequency filter is not sufficient for you, activate the related codes to filter 2nd or 3rd harmonics.

const short Fs = 1000;    // Sampling frequency(Hz). Must be in range 500-1200 Hz. Default value: 1 kHz
const int   Ts = 1e6/Fs;  // Sampling period (us);
const short f0 = 50;      // Cut-off frequency of the notch filter(Hz).
const short f1 = 150;     // Second cut-off frequency(Hz). (Must be integer multiple of f0)
const float w0 = 2*3.1416*f0/Fs;  // Digital cut-off frequency (rad/sample)
const float w1 = 2*3.1416*f1/Fs;  //
const float p = 0.95;     //Quailty factor. Must be between 0.8 and 0.995. Default value: 0.95

const float mag = (1 -2*cos(w0) +1)/(1 -2*p*cos(w0) + p*p); // Find magnitude of the filter at w=0 to set the DC gain to 1;

//Create the coefficient matrices for notch filter at f0
const float num[3] = {1/mag,  -2*cos(w0)/mag, 1/mag};         
const float den[3] = {1, -2*p*cos(w0),  p*p};

 //Optional dual frequency notch filter. Comment this code block out to use dual frequency notch filter
// Create the coefficient matrices for notch filter at f1
const float mag2 = (1 -2*cos(w1) +1)/(1 -2*p*cos(w1) + p*p);
const float num2[3] = {1/mag2,  -2*cos(w1)/mag2, 1/mag2};         
const float den2[3] = {1, -2*p*cos(w1),  p*p};

//Convolve two filters to obtain dual notch filter at frequencies f0 and f1
const float num3[5] = {num[0]*num2[0], num2[0]*num[1]+num2[1]*num[0], num2[0]*num[2] + num2[1]*num[1] + num2[2]*num[0], num2[1]*num[2]+num2[2]*num[1], num2[2]*num[2]};
const float den3[5] = {1, den2[0]*den[1]+den2[1]*den[0], den2[0]*den[2] + den2[1]*den[1] + den2[2]*den[0], den2[1]*den[2]+den2[2]*den[1], den2[2]*den[2]};


unsigned long start_time=0;
unsigned long current_time=0;
short raw[100] =  {0};
short emg[100] = {0};
unsigned long count=4;

float curr_avg,prev_avg; // Variables to store current and previous average
const float alpha = 0.1;  // Moving average coefficient. If hand movements are not detected change this number.
const float prev_alpha = 1 - alpha;
const int avg_thrsh = 35; //If noise is detected as hand movements increase this threshold. If hand movements are not detected decrease it.
const int digPin = 13;

short command_counter=0;

void setup() {
Serial.begin(2000000);
start_time = micros();
pinMode(digPin, OUTPUT);
}

void loop() {

  current_time = micros();
  
  if(current_time - start_time>= Ts){
 
    start_time = current_time;
    raw[count%100] = analogRead(A2)-295;

    //Filters only the 1st harmonic of the powerline noise
    //emg[count%100] = round(raw[count%100]*num[0] + raw[(count-1)%100]*num[1] + raw[(count-2)%100]*num[2] - emg[(count-1)%100]*den[1] - emg[(count-2)%100]*den[2]);
  
    //Comment the line below out if you experience high power line noise and need to use dual frequency notch filter. Filters 1st and 2nd harmonics.
    emg[count%100] = round(raw[count%100]*num3[0] + raw[(count-1)%100]*num3[1] + raw[(count-2)%100]*num3[2] + raw[(count-3)%100]*num3[3] + raw[(count-4)%100]*num3[4] - emg[(count-1)%100]*den3[1] - emg[(count-2)%100]*den3[2] - emg[(count-3)%100]*den3[3] - emg[(count-4)%100]*den3[4] );
  
    curr_avg = abs(emg[count%100])*alpha + prev_avg*prev_alpha;  //Calculate the current average.
    prev_avg=curr_avg;                                           // 
    
    Serial.println(emg[count%100]);

    if(command_counter>0){
      command_counter--;
      if(command_counter==0){                 //Turn off the digital output if the muscle has been deactive for sufficient amount of time.
        digitalWrite(digPin, LOW);
      }
    }
  
    if(abs(emg[count%100]) - prev_avg >=avg_thrsh){  // Check if muscle movement is detected.
      //Serial.println("");
      if(command_counter==0){
       digitalWrite(digPin, HIGH);           //Turn on the motor or other actuators.
      }
      command_counter = Fs/4;
    }
    count++;
  }
}
