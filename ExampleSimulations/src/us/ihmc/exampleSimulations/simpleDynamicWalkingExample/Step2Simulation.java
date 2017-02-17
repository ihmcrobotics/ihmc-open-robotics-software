package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 
 2 DOF Robot (1 leg)
 
    --Knee can move up and down & the body can rotate (pitch)
    
    --We can control both the body height and pitch
    
    --The robot cannot move in the Y-axis and cannot translate in the X-axis (only pitch). This is achieved as follows:
         a) The body joint is composed of a sliderJoint (allows motion in Z) and a pinJoint (allows rotation in XZ plane)
         b) The slider and pin joints are linked by a null link, which doesn't have dimensions or mass, but it is needed because 
            you cannot have joint-joint-joint.... you always need joint-link-joint-link....
         c) This "null link procedure" allows us to generate a customized joint! =) The root joint is now a "sliderPinJoint" instead of 
            a floatingJoint as in step0. This prevents the robot from falling to the sides because it cannot 
            translate in X.
            
     --Joint summary:
        (1) root joint: slider + pin (think of these 2 as a single customized joint)
        (2) hip joint: pin 
        (3) knee joint: slider
 
 **/

public class Step2Simulation
{

  SimulationConstructionSet sim;
  double deltaT = 0.00001;
  
  public Step2Simulation()
  {
     
    //Robot
     Step2Robot v2Robot = new Step2Robot();
     
     //Controller
     Step2Controller v2Controller = new Step2Controller(v2Robot,"v2Robot", deltaT);
     v2Robot.setController(v2Controller);
     
     //CoM (Silvain)
     YoFramePoint yoCenterOfMass = new YoFramePoint("CenterOfMass", ReferenceFrame.getWorldFrame(), new YoVariableRegistry("main"));
     Point3D centerOfMass = new Point3D();
     double totalMass = v2Robot.computeCenterOfMass(centerOfMass);
     yoCenterOfMass.set(centerOfMass);
     
     //Simulation object
     sim = new SimulationConstructionSet(v2Robot);
     sim.setGroundVisible(true);
     
     sim.setDT(deltaT, 10);
     sim.setCameraFix(0.0, 0.025, 1.05);
     sim.setCameraPosition(0.0, -20.0, 2.0);
     sim.changeBufferSize(32000);
     
     Thread myThread = new Thread(sim);
     myThread.start();
  }
   
   
   public static void main(String[] args)
   {
      new Step2Simulation();
   }

}
