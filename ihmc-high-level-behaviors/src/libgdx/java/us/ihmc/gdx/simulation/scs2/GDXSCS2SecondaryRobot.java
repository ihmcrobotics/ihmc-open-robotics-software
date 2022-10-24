package us.ihmc.gdx.simulation.scs2;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;

import java.util.function.BiConsumer;

public class GDXSCS2SecondaryRobot
{
   private final RobotDefinition robotDefinition;
   private final BiConsumer<RobotDefinition, Robot> robotSetup;

   public GDXSCS2SecondaryRobot(RobotDefinition robotDefinition, BiConsumer<RobotDefinition, Robot> robotSetup)
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
