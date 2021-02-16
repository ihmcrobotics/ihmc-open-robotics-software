package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MultiPendulumController implements RobotController
{
   private static final double maxTau = 1000;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final MultiPendulumRobot robot;

   private final YoDouble jointStiffness = new YoDouble("jointStiffness", registry);
   private final YoDouble jointDamping = new YoDouble("jointDamping", registry);

   private final YoDouble[] jointSetpoints;

   public MultiPendulumController(MultiPendulumRobot robot)
   {
      this.robot = robot;
      jointSetpoints = new YoDouble[robot.getN()];

      jointStiffness.set(450.0);
      jointDamping.set(40.0);

      for (int i = 0; i < robot.getN(); i++)
      {
         jointSetpoints[i] = new YoDouble("joint" + i + "Setpoint", registry);
      }
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < robot.getN(); i++)
      {
         double deltaQ = AngleTools.computeAngleDifferenceMinusPiToPi(jointSetpoints[i].getValue(), robot.getScsJoints()[i].getQ());
         double tau = jointStiffness.getDoubleValue() * deltaQ - jointDamping.getValue() * (robot.getScsJoints()[i].getQD());
         robot.getScsJoints()[i].setTau(MathTools.clamp(tau, maxTau));
      }

      robot.updateState();
   }

   public void setSetpoints(double... q)
   {
      for (int i = 0; i < Math.min(robot.getN(), q.length); i++)
      {
         jointSetpoints[i].set(q[i]);
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
