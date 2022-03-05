/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
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

PVector           fContact                            = new PVector(0, 0);
PVector           fDamping                            = new PVector(0, 0);
/* end effector radius in meters */
float             rEE                                 = 0.006;
float             rEEContact                          = 0.006;


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
float             worldHeight                         = 20.0;

float             edgeTopLeftX                        = 0.0;
float             edgeTopLeftY                        = 0.0;
float             edgeBottomRightX                    = worldWidth;
float             edgeBottomRightY                    = worldHeight;
float             edgeBottomLeftY                    = worldHeight;

float             gravityAcceleration                 = 15;//980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;


/* *********************************************************************define world shapes & attributes************************************************** */

final int WIDTH = 500;
final int HEIGHT = 500;

int rowh = 3;
int colh=5;

//int x1= int(worldWidth /(rowh+2) );
int y1= int(worldHeight / (colh+2));
//int lenx=x1;
//int leny=x1;



FBody[] particles = new FBody[15];
FBox bx1, bx2;

/* text font */
PFont             f;
int rotations = 0;

PShape pGraph, joint, endEffector;


  int i = 0;
  int cols = 5;
  int x = 6;
  int y = 5;
  int space = 3;
  int x1 = x;
  
    int i_1 = 0;
  int cols_1 = 5;
  int x_1 = 6;
  int y_1 = 5;
  int space_1 = 3;
  int x1_1 = x_1;
  
      int i_2 = 0;
  int cols_2 = 5;


/* *******************************************************************end define world shapes & attributes************************************************** */




/* end elements definition *********************************************************************************************/

/* setup section *******************************************************************************************************/
void setup() {
  
  println("hello");
  
  /* put setup code here, run once: */

  /* screen size definition */
  size(1000, 800);

  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  /* device setup */
  println("hello 3");
  haplyBoard          = new Board(this, Serial.list()[1], 0);
  println("hello 6");
  widgetOne           = new Device(widgetOneID, haplyBoard);
  println("hello 7");
  pantograph          = new Pantograph();
  println("hello 8");
  widgetOne.set_mechanism(pantograph);
  println("hello 9");
  widgetOne.add_actuator(1, CCW, 2);
  println("hello 10");
  widgetOne.add_actuator(2, CW, 1);
  println("hello 11");
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  println("hello 12");
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  println("hello 13");

  widgetOne.device_set_parameters();
  println("hello 14");
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this);
  println("hello 15");
  hAPI_Fisica.setScale(pixelsPerCentimeter);
  println("hello 16");
  world               = new FWorld();
  println("hello 17");
  println("hello 4");
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75));
  s.h_avatar.setDensity(4);
  s.h_avatar.setFill(0, 0, 200);
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2);
  println("hello 5");
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY));
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  /* *********************************************************************define world setup************************************************* */
  
  println("hello 2");
  
  //while (cols <= 15) {
    for( int j = i; j < cols; j ++) {
      if (cols <=5) {
     // println(j);
      particles[j] = new FBox(1.5, 1);
      particles[j].setPosition(x1,y);
      particles[j].setFill(120,200,190);
      particles[j].setStatic(true);
      world.add(particles[j]);
      x1 += space;
      }
      
    }
    y += 2;
    cols +=5;
    i += 5;
    x1 = x;
    
  }
  
  i_2 = 0;
  cols_2 = 5;  
  while (cols <= 15) {
    for( int j = i; j < cols-1; j ++) {
      FDistanceJoint jt = new FDistanceJoint(particles[j], particles[j+1]);
      jt.setFill(0);
      world.add(jt);
      
    }
    cols +=5;
    i += 5;
    
  }
  
 
  while (cols_1 < 15) {
    for( int j = i_1; j < cols; j ++) {
      if (j == 0) {
        FDistanceJoint jt = new FDistanceJoint(particles[j], particles[j+6]);
      } 
      FDistanceJoint jt = new FDistanceJoint(particles[j], particles[j+5]);
      jt.setFill(0);
      world.add(jt);
      
    }
    cols_1 +=5;
    i_1 += 5;
  }
  
  
  
  
 
   /* ******************************************************************end define world setup************************************************* */
  
  
  world.draw();

  /* setup framerate speed */
  frameRate(baseFrameRate);

  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/


/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  //if (renderingForce == false) {
    background(255);
    textFont(f, 22);
    //s.h_avatar.setSensor(false);
    /*
   for (int x = 0; x < width; x+=10) {
  for (int y = 0; y < height; y+=10) {
    noStroke();
    fill(0);
    rect(x, y, 2, 2);
  }
  
} 
*/
    
    world.draw();
  //}
}
/* end draw section ****************************************************************************************************/


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      angles.set(widgetOne.get_device_angles());
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));
    }

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7);
    s.updateCouplingForce();

    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    if (s.getAvatarPositionY() > 2) {
       s.h_avatar.setSensor(false);
      s.h_avatar.setDamping(150);
      
    } else if (s.getAvatarPositionY() > 5) {
      s.h_avatar.setSensor(false);
      s.h_avatar.setDamping(50);
      
    }
    else {
      s.h_avatar.setDamping(20);
      
    }

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
