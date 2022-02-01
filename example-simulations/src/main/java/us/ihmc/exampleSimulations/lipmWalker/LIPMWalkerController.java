package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LIPMWalkerController implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
   
   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);

   public LIPMWalkerController(LIPMWalkerRobot robot)
   {
      this.robot = robot;
      initialize();
   }

   @Override
   public void initialize()
   {
      kpKnee.set(1000.0);
      kdKnee.set(100.0);
      
      q_d_leftKnee.set(1.0);
      q_d_rightKnee.set(0.8);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      for (RobotSide side : RobotSide.values())
      {
         double kneeLength = robot.getKneeLength(side);
         double kneeVelocity = robot.getKneeVelocity(side);

         double desiredKneeLength = desiredKneeLengths.get(side).getValue();

         double kneeForce = kpKnee.getValue() * (desiredKneeLength - kneeLength) + kdKnee.getValue() * (0.0 - kneeVelocity);
         robot.setKneeForce(side, kneeForce);
      }
   }

}
