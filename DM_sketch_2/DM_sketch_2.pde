/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 * this is a modified version of the maze (fisica) demo. Modified by Devyani McLaren 
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 20;//10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  me; // the EE of physical haply

/* define maze blocks */
FBox              washroom, left_wall, start, top_wall, closet, bed, desk, dresser, p1, p2, jacket;
FBox              tko1, tko2, tko3, tko4, tko5, tko6, tko7, socks, socks1, socks2, socks3, lamp;
/* define game start */
FCircle           next, next1;
boolean           gameStart                           = false;
boolean           last                                 = false;
boolean           end                                 = false;

/* text font */
PFont             f;

PImage            bedP, deskP, p1P, p2P, jP, keyP, key2P, tko1P, tko2P, tko3P, tko4P, tko5P, sockP;
PImage            sock1P, sock2P, sock3P, lampP, tko6P, tko7P;

/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 800);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);
  
  /* device setup */
 
  haplyBoard          = new Board(this, Serial.list()[1], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  /* Set maze barriers */
  washroom                  = new FBox(8.0, 6.0); 
  washroom.setPosition(20.25 , 16.25);
  washroom.setFill(0);
  washroom.setNoStroke();
  washroom.setStaticBody(true);
  world.add(washroom);
  
  closet                  = new FBox(5.0, 6.0); 
  closet.setPosition(3.28 , 16.25);
  closet.setFill(0);
  closet.setNoStroke();
  closet.setStaticBody(true);
  world.add(closet);
  
  bed                  = new FBox(8.0, 5.0); 
  bed.setPosition(19 , 7.25);
  bed.setFill(0);
  bed.setNoStroke();
  bed.setStaticBody(true);
  world.add(bed);
  
  bedP = loadImage("./img/bed.png");
  bedP.resize(400, 250);
  bed.attachImage(bedP);
  
  desk              = new FBox(2.5, 6);
  desk.setPosition(2.265 , 4.5);
  desk.setFill(0);
  desk.setNoStroke();
  desk.setStaticBody(true);
  deskP = loadImage("./img/desk.jpg");
  deskP.resize(100, 250);
  desk.attachImage(deskP);
  world.add(desk);
  
  dresser              = new FBox(3.8, 2.0);
  dresser.setPosition(3.1 , 11);
  dresser.setFill(0);
  dresser.setNoStroke();
  dresser.setStaticBody(true);
  world.add(dresser);
 
  start                  = new FBox(1.0, 1.0);
  start.setPosition(11 ,16.25);
  start.setFill(0, 255, 0);
  start.setStaticBody(true);
  world.add(start);
  
  next                 = new FCircle(1.5);
  next.setPosition(22 ,2);
  next.setFill(0, 255, 0);
  next.setStroke(255,255,255);
  next.setStaticBody(true);
  keyP = loadImage("./img/keys.png");
  keyP.resize(90, 100);
  next.attachImage(keyP);
  //world.add(next);
  next1                 = new FCircle(1.5);
  next1.setPosition(22 ,2);
  next1.setFill(0, 255, 0);
  next1.setStroke(255,255,255);
  next1.setStaticBody(true);
  key2P = loadImage("./img/keys.png");
  key2P.resize(90, 100);
  next1.attachImage(key2P);
  
  p1                 = new FBox(2.3,2.3);
  p1.setPosition(16 ,2);
  p1.setFill(0, 255, 0);
  p1.setStroke(255,255,255);
  p1.setStaticBody(true);
  
  p1P = loadImage("./img/plant.png");
  p1P.resize(100, 90);
  p1.attachImage(p1P);
  
  p2                 = new FBox(2.3,2.3);
  p2.setPosition(16 ,4);
  p2.setFill(0, 255, 0);
  p2.setStroke(255,255,255);
  p2.setStaticBody(true);
  
  p2P = loadImage("./img/plant.png");
  p2P.resize(100, 90);
  p2.attachImage(p2P);

  jacket                 = new FBox(1.7,2.8);
  jacket.setPosition(16 ,11);
  jacket.setFill(0, 255, 0);
  jacket.setStroke(255,255,255);
  jacket.setStaticBody(true);
  
  jP = loadImage("./img/jacket.png");
  jP.resize(90, 120);
  jacket.attachImage(jP);
  
  tko1              = new FBox(1.5,2);
  tko1.setPosition(11.5 ,9);
  tko1.setFill(0, 255, 0);
  tko1.setStroke(255,255,255);
  tko1.setStaticBody(true);
  
  tko1P = loadImage("./img/takeout1.png");
  tko1P.resize(80, 110);
  tko1.attachImage(tko1P);
  
  tko2              = new FBox(1.5,2);
  tko2.setPosition(11.5 ,11.5);
  tko2.setFill(0, 255, 0);
  tko2.setStroke(255,255,255);
  tko2.setStaticBody(true);
  
  tko2P = loadImage("./img/takeout1.png");
  tko2P.resize(80, 110);
  tko2.attachImage(tko2P);
  
  tko3              = new FBox(1.5,2);
  tko3.setPosition(11.5 ,13.5);
  tko3.setFill(0, 255, 0);
  tko3.setStroke(255,255,255);
  tko3.setStaticBody(true);
  
  tko3P = loadImage("./img/takeout1.png");
  tko3P.resize(80, 110);
  tko3.attachImage(tko3P);
  
  
  tko4              = new FBox(1.5,2);
  tko4.setPosition(13.5 ,9);
  tko4.setFill(0, 255, 0);
  tko4.setStroke(255,255,255);
  tko4.setStaticBody(true);
  
  tko4P = loadImage("./img/takeout.png");
  tko4P.resize(110, 80);
  tko4.attachImage(tko4P);
  
  tko5              = new FBox(1.5,2);
  tko5.setPosition(11.5 ,15.0);
  tko5.setFill(0, 255, 0);
  tko5.setStroke(255,255,255);
  tko5.setStaticBody(true);
  
  tko5P = loadImage("./img/takeout1.png");
  tko5P.resize(110, 80);
  tko5.attachImage(tko5P);
  
  tko6              = new FBox(1.5,2);
  tko6.setPosition(11.5 ,16.5);
  tko6.setFill(0, 255, 0);
  tko6.setStroke(255,255,255);
  tko6.setStaticBody(true);
  
  tko6P = loadImage("./img/takeout1.png");
  tko6P.resize(110, 80);
  tko6.attachImage(tko6P);
  
  
  socks              = new FBox(1.5,2);
  socks.setPosition(8.0 ,9.8);
  socks.setFill(0, 255, 0);
  socks.setStroke(255,255,255);
  socks.setStaticBody(true);
  
  sockP = loadImage("./img/socks.png");
  sockP.resize(100, 85);
  socks.attachImage(sockP);
  
  socks1              = new FBox(1.5,2);
  socks1.setPosition(8.0 , 12.2);
  socks1.setFill(0, 255, 0);
  socks1.setStroke(255,255,255);
  socks1.setStaticBody(true);
  
  sock1P = loadImage("./img/socks.png");
  sock1P.resize(100, 85);
  socks1.attachImage(sock1P);
  
  socks2              = new FBox(1.5,2);
  socks2.setPosition(8.0 ,13.9);
  socks2.setFill(0, 255, 0);
  socks2.setStroke(255,255,255);
  socks2.setStaticBody(true);
  
  sock2P = loadImage("./img/socks.png");
  sock2P.resize(100, 85);
  socks2.attachImage(sock2P);
  
  socks3              = new FBox(1.5,2);
  socks3.setPosition(10,9);
  socks3.setFill(0, 255, 0);
  socks3.setStroke(255,255,255);
  socks3.setStaticBody(true);
  
  sock3P = loadImage("./img/socks.png");
  sock3P.resize(100, 85);
  socks3.attachImage(sock3P);
  
  lamp = new FBox(4,2);
  lamp.setPosition(7.2,5.5);
  lamp.setFill(0, 255, 0);
  lamp.setStroke(255,255,255);
  lamp.setStaticBody(true);
  
  lampP = loadImage("./img/lamp.png");
  lampP.resize(400, 185);
  lamp.attachImage(lampP);
  
  
  
  
  
   /* Setup the Virtual Coupling Contact Rendering Technique */
  me                   = new HVirtualCoupling((0.75)); 
  me.h_avatar.setDensity(4); 
  me.h_avatar.setFill(255,0,0); 
 // me.h_avatar.setSensor(true);

  me.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 18);
    
    if(gameStart){
      fill(0, 0, 0);
      
      washroom.setFill(51, 153, 255);
      text("Washroom", 780 ,525, 90);
      
      //start.setFill(255, 255, 255);
      start.setStroke(51, 153, 255);
      start.setFill(51, 153, 255);
      
      next.setFill(120,40,200);
      
      closet.setFill(192,192,192);
      text("Closet", 80, 525, 90);
      
      dresser.setFill(255,128,0);
      text("Dresser", 80, 390, 90);
    }

    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch green start box to start and \n find the purple target", 450, 700);
      //washroom.setFill(255, 255, 255);
      next.setFill(255,255,255);
      //closet.setFill(255,255,255);
      //dresser.setFill(255);
    }
    
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    me.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    me.updateCouplingForce();
 
    fEE.set(-me.getVirtualCouplingForceX(), me.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    if (me.h_avatar.isTouchingBody(start)){
      gameStart = true;
      start.setPosition(20.25,16.25);
      start.setFill(51, 153, 255);
      me.h_avatar.setSensor(false);
      world.add(next);
    }
    //messy room
    if (me.h_avatar.isTouchingBody(next)){
      world.remove(next);
      world.add(next1);
      next1.setPosition(4.2, 2.0);
      me.h_avatar.setSensor(false);
      last = true;
      world.add(p1);
      world.add(p2);
      world.add(jacket);
      world.add(tko1);
      world.add(tko2);
      world.add(tko3);
      world.add(tko4);
      world.add(tko5);
      world.add(tko6);
      world.add(socks);
      world.add(socks1);
      world.add(socks2);
      world.add(socks3);
      world.add(lamp);
    }
    //end


    
        world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}

/* end simulation section **********************************************************************************************/
