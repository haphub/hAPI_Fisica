/**
 *  Anchors and the bridge
 *
 *  Adapted by Colin Gallacher for use with hAPI_Fisica
 *  original code by Richard Marxer 
 *
 *  This example shows the use of anchors and distance joints in order
 *  to create a bridge.
 */


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
float worldHeight = 20.0; 

float edgeTopLeftX = 0.0-2.0; 
float edgeTopLeftY = 0.0-2.0; 
float edgeBottomRightX = worldWidth+2.0; 
float edgeBottomRightY = worldHeight+2.0;


float paddle_position;

PImage hapkit_avatar; 
  PFont f;


/*Bridge Graphic objects*/ 
FBody[] steps = new FBody[31];
float frequency = 300;
float damping = 300;
float boxWidth;

int targetCount = 0; 

void setup() {
  size(400, 800);
  
  /* BOARD */
  hapkit_board  = new Board(this, Serial.list()[0], 112500);

  /* DEVICE */
  haptic_paddle = new Device(degreesOfFreedom.HapticPaddle, device_function, hapkit_board);

/*Initialize the world parameters*/ 

  hAPI_Fisica.init(this);
  hAPI_Fisica.setScale(pixelsPerCentimeter); 



/*world Setup*/ 
  world = new FWorld();
  world.setGravity((0.0), (50.0)); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.remove(world.top); 
  world.remove(world.left); 
  world.remove(world.right); 
  
  /* Haptic Tool Initialization   */

  s= new HVirtualCoupling((0.5)); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+15); 
  hapkit_avatar = loadImage("../img/hapkit_avatar.png"); 
  hapkit_avatar.resize((int)(hAPI_Fisica.worldToScreen(.75)), (int)(hAPI_Fisica.worldToScreen(.57)));
  s.h_avatar.attachImage(hapkit_avatar); 


  boxWidth =  hAPI_Fisica.screenToWorld(200)/(steps.length);

  for (int i=0; i<steps.length; i++) {
    if(i>steps.length/2-2 && i < steps.length/2+2 && i != steps.length/2) {
    steps[i] = new FBox(3*boxWidth,  .25+.1);
    steps[i].setPosition(map(i, 0, steps.length-1, boxWidth+.2,  hAPI_Fisica.screenToWorld(width)-boxWidth), worldHeight-worldHeight/3.0f);
    steps[i].setNoStroke();
    steps[i].setRestitution(0); 
    steps[i].setFill(120, 200, 190);
    world.add(steps[i]);
    }else{
      
    steps[i] = new FBox(boxWidth,  .25);
    steps[i].setPosition(map(i, 0, steps.length-1, boxWidth,  hAPI_Fisica.screenToWorld(width)-boxWidth), worldHeight-worldHeight/3.0f);
    steps[i].setNoStroke();
     steps[i].setRestitution(0); 
    steps[i].setFill(120, 200, 190);
    world.add(steps[i]);
  }
    
  }

  for (int i=1; i<steps.length; i++) {
    
    FDistanceJoint joint = new FDistanceJoint(steps[i-1], steps[i]);
    joint.setAnchor1(boxWidth/2, 0);
    joint.setAnchor2(-boxWidth/2, 0);
    joint.setFrequency(frequency);
    joint.setDamping(damping);
    joint.setFill(0);
    joint.calculateLength();
    world.add(joint);
  }

  FCircle left = new FCircle( .4);
  left.setStatic(true);
  left.setPosition(0, worldHeight-worldHeight/3.0f);
  left.setDrawable(false);
  world.add(left);

  FCircle right = new FCircle(0.4);
  right.setStatic(true);
  right.setPosition(hAPI_Fisica.screenToWorld(width), worldHeight-worldHeight/3.0f);
  right.setDrawable(false);
  world.add(right);

  FDistanceJoint firstJoint = new FDistanceJoint(steps[0], left);
  firstJoint.setAnchor1(-boxWidth/2, 0);
  firstJoint.setAnchor2(0, 0);
  firstJoint.setFrequency(frequency);
  firstJoint.setDamping(damping);
  firstJoint.calculateLength();
  firstJoint.setFill(0);
  world.add(firstJoint);

  FDistanceJoint finalJoint = new FDistanceJoint(steps[steps.length-1], right);
  finalJoint.setAnchor1(boxWidth/2, 0);
  finalJoint.setAnchor2(0, 0);
  finalJoint.setFrequency(frequency);
  finalJoint.setDamping(damping);
  finalJoint.calculateLength();
  finalJoint.setFill(0);
  world.add(finalJoint);
  
   world.draw();
  
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start(); 
  
  frameRate(100); 


  

  f = createFont("Arial",16,true); // STEP 2 Create Font
  textFont(f,16);                  // STEP 3 Specify font to be used
  fill(0);                         // STEP 4 Specify font color

}

void draw() {
  
  background(255);
   
   if(((61000-millis())/1000)>0){
  if (hapkit_board.data_available()){
    haptic_paddle.receive_data();    
    
    paddle_position = 1000*haptic_paddle.mechanisms.get_coordinate()[0]; // converts units  TODO: fix bug because should be from m to cm (i.e. scale by 100) Must be problem in haptic paddle mechanism object src code
        
  }
  
  f_ee.set(-s.getVCforceX(), s.getVCforceY());
  f_ee.div(10000); //dynes to newtons
   
  
  force =- 3*f_ee.y; //scale the force to feel it on the device. 
  
  haptic_paddle.set_parameters(device_function, force, torque);
  haptic_paddle.send_data();
  
  
    world.draw();
    //world.drawDebug(); 
    
      text("Time Left", 10, 20); 
    text(Integer.toString((61000-millis())/1000),100,20); 
   
   text("Targets Hit",10,50);   // STEP 5 Display Text
      text(Integer.toString(targetCount),100,50);   // STEP 5 Display Text
      


    
   }
   else{
      force =- 0; //scale the force to feel it on the device. 
  
  haptic_paddle.set_parameters(device_function, force, torque);
  haptic_paddle.send_data(); 
  
    text("Time Left", 10, 20); 
    text("Finished!",100,20); 
   
   text("Targets Hit",10,50);   // STEP 5 Display Text
      text(Integer.toString(targetCount),100,50);   // STEP 5 Display Text
      


   }
   


   
  if(isTarget == true && target.getContacts().size() ==1){
  
  ArrayList<FContact> c =  target.getContacts(); 
  
    world.removeBody(c.get(0).getBody1());
    world.removeBody(c.get(0).getBody2());
    isTarget = false; 
    targetCount ++; 
  
  }
}

boolean isTarget = false;
boolean isAmmo = false; 
FCircle target; 

void mousePressed() {
   
  float radius; 
  
  radius = random(2, 4);
  radius = radius/2.5f; 
  FCircle ammo = new FCircle(radius);
  ammo.setPosition(hAPI_Fisica.screenToWorld(mouseX), hAPI_Fisica.screenToWorld(mouseY));
  ammo.setDensity(.25);
  ammo.setRestitution(0); 
  ammo.setFill(random(0,255), random(0,255) , random(0,255));
  ammo.setNoStroke();
  world.add(ammo);  


if(!isTarget){
  radius = random(1, 5);
  radius = radius/2.0f; 
  target = new FCircle(radius);
  target.setSensor(true); 
  target.setGrabbable(false); 
  target.setPosition(worldWidth/random(1,3), worldHeight/10);
  target.setStatic(true);
  target.setFill(random(0,255), random(0,255) , random(0,255));
  target.setNoStroke();
  world.add(target);  
  isTarget = true; 
}

}

void keyPressed() {
  try {
    saveFrame("screenshot.png");
  } 
  catch (Exception e) {
  }

}


/**********************************************************************************************************************
 * Haptics simulation event, engages state of physical mechanism, calculates and updates physics simulation conditions
 **********************************************************************************************************************/ 
void onTickEvent(CountdownTimer t, long timeLeftUntilFinish){
   
  s.setToolPosition(edgeTopLeftX+2.0+worldWidth/2, edgeTopLeftY+15.0-paddle_position);
  
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