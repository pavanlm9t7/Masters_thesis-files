#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_SPI.h> //Hardware-specific library
#include <LCDWIKI_TOUCH.h> //touch screen library
#include <SoftwareSerial.h>




//paramters define
#define MODEL ST7796S
#define CS   A5    
#define CD   A3
#define RST  A4
#define LED  A0   //if you don't need to control the LED pin,you should set it to -1 and set it to 3.3V

//touch screen paramters define
#define TCS   2
#define TCLK  3
#define TDOUT 4
#define TDIN  5
#define TIRQ  6


// Initialize LCD with specified parameters
LCDWIKI_SPI mylcd(MODEL,CS,CD,RST,LED); //model,cs,dc,reset,led

// Initialize touch screen with specified parameters
LCDWIKI_TOUCH mytouch(TCS,TCLK,TDOUT,TDIN,TIRQ); //tcs,tclk,tdout,tdin,tirq

// Initialize a SoftwareSerial port for communication
SoftwareSerial SerialPort(7, 8); // RX, TX


// ... (color definitions, spacing, variables, etc.)
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define SPACE_X 40
#define SPACE_Y 40
#define BUTTON_SPACING_X 20
#define BUTTON_SPACING_Y 20
#define BUTTON_TEXTSIZE 2

uint16_t px,py;
String input;
float tset, tgen, t1, t2;
int inputlength=0;
bool input_state = false; 
bool machine_state = true; 
unsigned long lastTime = 0;
const unsigned long interval = 1000;





void string_update(){
  // Function to update the string based on touch inputs
  // ... (if conditions based on touch coordinates)

  int inputlength = strlen(input.c_str());
  if((px>25 && px<40)&&(py>25 && py<45 && input_state && machine_state)){
    input.concat('1');
    input_state = false;
  }
  if((px>85 && px<105)&&(py>25 && py<45 && input_state && machine_state)){
    input.concat('2');
    input_state = false;
  }
  if((px>135 && px<165)&&(py>25 && py<45 && input_state && machine_state)){
    input.concat('3');
    input_state = false;
  }
  if((px>25 && px<40)&&(py>75 && py<115 && input_state && machine_state)){
    input.concat('4');
    input_state = false;
  }
  if((px>85 && px<105)&&(py>75 && py<115 && input_state && machine_state)){
    input.concat('5');
    input_state = false;
  }
  if((px>135 && px<165)&&(py>75 && py<115 && input_state && machine_state)){
    input.concat('6');
    input_state = false;
  }
  if((px>25 && px<40)&&(py>135 && py<180 && input_state && machine_state)){
    input.concat('7');
    input_state = false;
  }
  if((px>85 && px<105)&&(py>135 && py<180 && input_state && machine_state)){
    input.concat('8');
    input_state = false;
  }
  if((px>135 && px<165)&&(py>135 && py<180 && input_state && machine_state)){
    input.concat('9');
    input_state = false;
  }
  if((px>25 && px<40)&&(py>190 && py<240 && input_state && machine_state)){
    if(inputlength == 0){
    input.concat('-');
    input_state = false;
  }}
  if((px>85 && px<105)&&(py>190 && py<240 && input_state && machine_state)){
    input.concat('0');
    input_state = false;
  }
  if((px>135 && px<165)&&(py>190 && py<240 && input_state)){
    char ch ='.';
    const char* str = input.c_str();

    // check if the character exists in the string
    if((inputlength !=0)&&(strchr(str, ch)== NULL)){
        input += ch;
    }
  }
  if((px>10 && px<105)&&(py>235 && py<300 && input_state && machine_state)){
    const char* charArray = input.c_str();
    if(input !=0){
    tset = atof(charArray);
    if(tset>65){
      tset = 65;
    }
    if(tset<-5){
      tset = -5;
    }
    }
    input='\0';
    input_state = false;
    mylcd.Fill_Rect(10,300,480,320,WHITE);
    mylcd.Set_Text_colour(RED);
    mylcd.Set_Text_Back_colour(128,128,128);
    mylcd.Set_Text_Size(5);
    mylcd.Fill_Rect(180,70,300,50,WHITE);
    mylcd.Print_Number_Float(tset, 2, 190, 70, '.', 0, ' '); 




  }
  if((px>115 && px<270)&&(py>255 && py<300 && input_state)){
    machine_state = false;
    input_state = false;
  }
  if((px>280 && px<375)&&(py>255 && py<300)&& input_state){
    machine_state = true;
  }
  mylcd.Set_Text_colour(BLACK);
  mylcd.Set_Text_Back_colour(128,128,128);
  mylcd.Set_Text_Size(2);
  mylcd.Print_String(input,10,300);
  delay(100);
}

void update_tgen(){
  // Function to update the generated temperature display
  // ... (update the generated temperature display)

    mylcd.Set_Text_colour(RED);
    mylcd.Set_Text_Back_colour(128,128,128);
    mylcd.Set_Text_Size(5);
    float t1 = tgen;
    if(t1 != t2){
    mylcd.Fill_Rect(180,180,300,50,WHITE);
    mylcd.Print_Number_Float(tgen, 2, 190, 180, '.', 0, ' '); 
    }
    t2=t1;
    delay(100);

}
void display_screen(){
  // Function to display the initial screen with buttons
  // ... (draw buttons and labels)

  int x=40;
  int y=40;
  float z=1.0;
  mylcd.Set_Text_colour(BLACK);
  mylcd.Set_Text_Back_colour(128,128,128);
  mylcd.Set_Text_Mode(1);
  mylcd.Set_Text_colour(BLACK);
  mylcd.Set_Text_Size(5);
  mylcd.Print_String("1", 20, 20);
  mylcd.Print_String("2", 80, 20);
  mylcd.Print_String("3", 140, 20);
  mylcd.Print_String("4", 20, 80);
  mylcd.Print_String("5", 80, 80);
  mylcd.Print_String("6", 140, 80);
  mylcd.Print_String("7", 20, 140);
  mylcd.Print_String("8", 80, 140);
  mylcd.Print_String("9", 140, 140);
  mylcd.Print_String("-", 20, 200);
  mylcd.Print_String("0", 80, 200);
  mylcd.Print_String(".", 140, 200);
  mylcd.Set_Text_Size(5);
  //mylcd.Draw_Rectangle(10,250,110,300);
  mylcd.Fill_Rect(10, 250, 100, 50,BLUE); 
  mylcd.Print_String("SET", 20, 260);
  mylcd.Draw_Rectangle(10,10,175,245);
  mylcd.Draw_Rectangle(11,11,174,244);
  mylcd.Draw_Rectangle(9,9,176,246);
  mylcd.Set_Text_Size(5);
  mylcd.Print_String("SET TEMP", 190, 20);
  mylcd.Print_String("GEN TEMP", 190, 120);
  //mylcd.Draw_Rectangle(115,250,270,300);
  mylcd.Fill_Rect(115, 250,160,50,RED); 
  mylcd.Print_String("LOCK", 140, 260);
  //mylcd.Draw_Rectangle(10,250,110,300);
  mylcd.Fill_Rect(280, 250, 100, 50,GREEN); 
  mylcd.Print_String("RUN",285, 260);
  
  
}


void setup() {
  // put your setup code here, to run once:
  // Setup function runs once at the beginning
  // Initialize LCD, touch screen, and pins
  // ... (initializations)

  mylcd.Init_LCD();
  mylcd.Set_Rotation(1);
  mytouch.TP_Set_Rotation(3);
  mytouch.TP_Init(mylcd.Get_Rotation(),mylcd.Get_Display_Width(),mylcd.Get_Display_Height());
  mylcd.Fill_Screen(WHITE);
  display_screen();
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, INPUT);
  Serial.begin(9600);  // Initialize hardware UART at 9600 bps
  SerialPort.begin(9600);
 
}



void loop() {
  // put your main code here, to run repeatedly:
  // If the machine is in "RUN" state
  // ... (read touch inputs, update tgen, communicate with SerialPort)
if(machine_state){
  px=0;
  py=0;
  mytouch.TP_Scan(0);
  if (mytouch.TP_Get_State()&TP_PRES_DOWN) 
  {
    px = mytouch.x;
    py = mytouch.y;
    input_state = true;
    string_update();
  }
  mylcd.Set_Draw_color(GREEN);
  mylcd.Fill_Circle(425,275,25);

  if (SerialPort.available()>8) {
    String a = "";
    char c;
    c = SerialPort.read();
    while(c!='S'){
     c = SerialPort.read();
    }
     tgen = SerialPort.parseFloat();
     SerialPort.flush();

    Serial.print("tgen: ");
   Serial.println(tgen);
  }
  //SerialPort.flush();

  // Send data back on the same SoftwareSerial port
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    String tset_string = String(tset);
    SerialPort.write(tset_string.c_str());
    lastTime = currentTime; 
  }
update_tgen();
  }
  
  

if(!machine_state){
  // If the machine is in "LOCK" state
  // ... (read touch inputs, update tgen)
  px=0;
  py=0;
  mytouch.TP_Scan(0);
  if (mytouch.TP_Get_State()&TP_PRES_DOWN) 
  {
    px = mytouch.x;
    py = mytouch.y;
    input_state = true;
    string_update();
  }
  mylcd.Set_Draw_color(RED);
  mylcd.Fill_Circle(425,275,25);
  update_tgen();

}
}
