package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;

public class LIPMWalkerRobot
{
   private Robot robot;

   public LIPMWalkerRobot()
   {
      RobotDefinitionFixedFrame definition = new RobotDefinitionFixedFrame();
      robot = new Robot(definition, "LIPMWalker");
   }

   public Robot getRobot()
   {
      return robot;
   }
}
