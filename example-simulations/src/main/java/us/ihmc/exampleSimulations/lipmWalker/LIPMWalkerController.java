package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LIPMWalkerController implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public LIPMWalkerController(LIPMWalkerRobot robot) {
      this.robot = robot;
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      // TODO Auto-generated method stub
      return registry;
   }

   @Override
   public void doControl()
   {
      for (RobotSide side: RobotSide.values())
      {
         double kneeLength = robot.getKneeLength(side);
         double kneeVelocity = robot.getKneeVelocity(side);

         double kp = 100.0;
         double kd = 10.0;
         double desiredKneeLength = 1.0;

         double kneeForce = kp * (desiredKneeLength - kneeLength) + kd * (0.0 - kneeVelocity);
         robot.setKneeForce(side, kneeForce);
      }
   }

}
