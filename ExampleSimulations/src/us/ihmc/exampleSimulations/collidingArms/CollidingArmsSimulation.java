package us.ihmc.exampleSimulations.collidingArms;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class CollidingArmsSimulation
{

   public CollidingArmsSimulation()
   {
      CollidingArmRobotDescription armOneDescription = new CollidingArmRobotDescription("ArmOne", new Vector3d(-5.0, 0.0, 0.0));
      CollidingArmRobotDescription armTwoDescription = new CollidingArmRobotDescription("ArmTwo", new Vector3d(5.0, 0.0, 0.0));
            
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);
            
      SimulationConstructionSet scs = new SimulationConstructionSet(parameters); 
      
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(staticLinkGraphics);
      Robot armOne = new RobotFromDescription(armOneDescription);      
      Robot armTwo = new RobotFromDescription(armTwoDescription);
      
      scs.addRobot(armTwo);
      scs.addRobot(armOne);
//      scs.addRobot(armTwoDescription);
      
      scs.startOnAThread();
      
   }
   
   public static void main(String[] args)
   {
      new CollidingArmsSimulation();
   }
}
