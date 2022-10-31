package us.ihmc.rdx.simulation.scs2;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;

import java.util.function.BiConsumer;

public class RDXSCS2SecondaryRobot
{
   private final RobotDefinition robotDefinition;
   private final BiConsumer<RobotDefinition, Robot> robotSetup;

   public RDXSCS2SecondaryRobot(RobotDefinition robotDefinition, BiConsumer<RobotDefinition, Robot> robotSetup)
   {
      this.robotDefinition = robotDefinition;
      this.robotSetup = robotSetup;
   }

   public Robot create()
   {
      Robot robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);
      robotSetup.accept(robotDefinition, robot);
      return robot;
   }
}
