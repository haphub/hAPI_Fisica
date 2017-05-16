/**
 *  Bubbles
 *
 *  Adapted by Colin Gallacher for use with hAPI_Fisica
 *  original code by Richard Marxer 
 *
 *  This code should 
 *  This example shows the use of anchors and distance joints in order
 *  to create a bridge.
 */

/* library imports *****************************************************************************************************/
import processing.serial.*;
import com.dhchoi.CountdownTimer;
import com.dhchoi.CountdownTimerService;

/* Device block definitions ********************************************************************************************/
Board             hapkit_board;
Device            haptic_paddle;
DeviceType        degreesOfFreedom;
boolean           rendering                  = false;


/* Simulation Speed Parameters ****************************************************************************************/
final long        SIMULATION_PERIOD          = 1; //ms
float             dt                         = SIMULATION_PERIOD/1000.0;
final long        HOUR_IN_MILLIS             = 36000000;
CountdownTimer    haptic_timer;


/* Communications parameters ******************************************************************************************/
byte              device_function            = 2;


/* generic data for a 1DOF device *************************************************************************************/
/* joint space */
float             force                      = 0; 
float             torque                     = 0;

/* task space */
PVector           pos_ee                     = new PVector(0, 0);
PVector           f_ee                       = new PVector(0, 0); 

HVirtualCoupling  s; 


/*Screen and world setup parameters*/ 
float pixelsPerCentimeter= 40.0; 


FWorld world;
float worldWidth = 10.0;
float worldHeight = 10.0; 

float edgeTopLeftX = 0.0-2.0; 
float edgeTopLeftY = 0.0-2.0; 
float edgeBottomRightX = worldWidth+2.0; 
float edgeBottomRightY = worldHeight+2.0;

int circleCount = 30;
float hole = 1.25;
float topMargin = 1.25;
float bottomMargin = 7.5;
float sideMargin = 2.5;
float xPos = 0;

float paddle_position;

PImage hapkit_avatar; 

void setup() {
  size(400, 400); // 10cm *40pxpercm


  /* BOARD */
  hapkit_board  = new Board(this, Serial.list()[0], 112500);

  /* DEVICE */
  haptic_paddle = new Device(degreesOfFreedom.HapticPaddle, device_function, hapkit_board);


  hAPI_Fisica.init(this);
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  
  world = new FWorld();
  world.setGravity(0, -400);


  FPoly l = new FPoly();
  l.vertex(worldWidth/2-hole/2, 0);
  l.vertex(0, 0);
  l.vertex(0, worldHeight);
  l.vertex(0+sideMargin, worldHeight);
  l.vertex(0+sideMargin, worldHeight-bottomMargin);
  l.vertex(worldWidth/2-hole/2, topMargin);
  l.setStatic(true);
  l.setFill(0);
  l.setFriction(0);
  world.add(l);

  FPoly r = new FPoly();
  r.vertex(worldWidth/2+hole/2, 0);
  r.vertex(worldWidth, 0);
  r.vertex(worldWidth, worldHeight);
  r.vertex(worldWidth-sideMargin, worldHeight);
  r.vertex(worldWidth-sideMargin, worldHeight-bottomMargin);
  r.vertex(worldWidth/2+hole/2, topMargin);
  r.setStatic(true);
  r.setFill(0);
  r.setFriction(0);
  world.add(r);
  
  s = new HVirtualCoupling(0.5f); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+2.0+worldWidth/2.0, edgeTopLeftY+5); 
  s.setToolPosition(edgeTopLeftX+2.0+worldWidth/2.0, edgeTopLeftY+5);
  hapkit_avatar = loadImage("../img/hapkit_avatar.png"); 
  hapkit_avatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(.75)));
  s.h_avatar.attachImage(hapkit_avatar); 

  
  //thread("drawBubble"); 
  
  world.draw();
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
    
      frameRate(40); 
    //frameRate(60); 
    //frameRate(100); 
}

  

void draw() {

  background(80, 120, 200);
  
  if (hapkit_board.data_available()){
    haptic_paddle.receive_data();    
    
    paddle_position = 1000*haptic_paddle.mechanisms.get_coordinate()[0]; // converts units  TODO: fix bug because should be from m to cm (i.e. scale by 100) Must be problem in haptic paddle mechanism object src code
        
  }
  
  f_ee.set(-s.getVCforceX(), s.getVCforceY());
  f_ee.div(10000); //dynes to newtons
  
  println(f_ee.y); 
  
  force = 2*f_ee.y; //scale the force to feel it on the device. 
  
  haptic_paddle.set_parameters(device_function, force, torque);
  haptic_paddle.send_data();
  
  
  if ((frameCount % 70) == 1) {
    thread("drawBubble"); 
  }

  world.draw();
  
}


void keyPressed() {
  try {
    saveFrame("screenshot.png");
  } 
  catch (Exception e) {
  }
}


void drawBubble(){
    FBlob b = new FBlob();
   
    float s = random(2, 4);
    s = s/3.0f; 
    float space = (worldWidth-sideMargin*2-s);
    xPos = (xPos + random(s, space/2)) % space;
    b.setAsCircle(sideMargin + xPos+s/2, worldHeight+random(2), s, circleCount);
    b.setStroke(0);
    b.setStrokeWeight(2);
    b.setFill(255);
    b.setFriction(0);
    b.setDensity(10);
    world.add(b);   
}


/**********************************************************************************************************************
 * Haptics simulation event, engages state of physical mechanism, calculates and updates physics simulation conditions
 **********************************************************************************************************************/ 
void onTickEvent(CountdownTimer t, long timeLeftUntilFinish){
  
  
  s.setToolPosition(edgeTopLeftX+2.0+worldWidth/2, edgeTopLeftY+5.0-paddle_position);
  
  s.updateCouplingForce();
  world.step(dt); 
  
}

/* Timer control event functions **************************************************************************************/

/**
 * haptic timer reset
 */
void onFinishEvent(CountdownTimer t){
  println("Resetting timer...");
  haptic_timer.reset();
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
}