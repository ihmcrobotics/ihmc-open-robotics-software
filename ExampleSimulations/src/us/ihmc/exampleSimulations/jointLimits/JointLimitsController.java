package us.ihmc.exampleSimulations.jointLimits;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class JointLimitsController extends SimpleRobotController
{
   private final JointLimitsRobot robot;

   private final DoubleYoVariable magnitude = new DoubleYoVariable("Magnitude", registry);
   private final DoubleYoVariable frequency = new DoubleYoVariable("Frequency", registry);
   private final DoubleYoVariable gain = new DoubleYoVariable("Gain", registry);

   private final DoubleYoVariable qDesired = new DoubleYoVariable("QDesired", registry);
   private final DoubleYoVariable qddDesired = new DoubleYoVariable("QDDDesired", registry);
   private final DoubleYoVariable qddDesiredLimited = new DoubleYoVariable("QDDDesiredLimited", registry);

   private final DoubleYoVariable lowerLimit = new DoubleYoVariable("LowerLimit", registry);
   private final DoubleYoVariable upperLimit = new DoubleYoVariable("UpperLimit", registry);

   private final DoubleYoVariable qddMax = new DoubleYoVariable("QDDMax", registry);
   private final DoubleYoVariable qddMin = new DoubleYoVariable("QDDMin", registry);
   private final DoubleYoVariable qddMaxAbs = new DoubleYoVariable("QDDMaxAbs", registry);

   public JointLimitsController(JointLimitsRobot robot)
   {
      this.robot = robot;

      magnitude.set(0.75);
      frequency.set(1.0);
      gain.set(10.0);

      lowerLimit.set(robot.getLowerLimit());
      upperLimit.set(robot.getUpperLimit());

      qddMaxAbs.set(10.0);
   }

   @Override
   public void doControl()
   {
      double f = frequency.getDoubleValue();
      double m = magnitude.getDoubleValue();

      double q_d = m * Math.sin(f * robot.getTime());
      double qdd_ff = -f*f*m * Math.sin(f * robot.getTime());
      double qdd_fb = (q_d - robot.getQ()) * gain.getDoubleValue();

      qDesired.set(q_d);
      qddDesired.set(qdd_ff + qdd_fb);

      limitQdd();
      robot.setQdd(qddDesiredLimited.getDoubleValue());
   }

   private void limitQdd()
   {
      qddMax.set(qddMaxAbs.getDoubleValue());
      qddMin.set(-qddMaxAbs.getDoubleValue());

      // --- do limiting here ---
      double timeHorizon = 0.05;
      double absoluteMaximumJointAcceleration = qddMaxAbs.getDoubleValue();
      double maxBreakAcceleration = 4.0;
      if (!Double.isInfinite(lowerLimit.getDoubleValue()))
      {
         // if we enter the limit do not explode:
         double distance = robot.getQ() - lowerLimit.getDoubleValue();
         distance = Math.max(0.0, distance);

         double qDotMin = -Math.pow(distance, 2) / timeHorizon;
         double qDDotMin = (qDotMin - robot.getQd()) / timeHorizon;
         qDDotMin = MathTools.clipToMinMax(qDDotMin, -absoluteMaximumJointAcceleration, maxBreakAcceleration);
         qddMin.set(qDDotMin);
      }
      if (!Double.isInfinite(upperLimit.getDoubleValue()))
      {
         // if we enter the limit do not explode:
         double distance = upperLimit.getDoubleValue() - robot.getQ();
         distance = Math.max(0.0, distance);

         double qDotMax = Math.pow(distance, 2) / timeHorizon;
         double qDDotMax = (qDotMax - robot.getQd()) / timeHorizon;
         qDDotMax = MathTools.clipToMinMax(qDDotMax, -maxBreakAcceleration, absoluteMaximumJointAcceleration);
         qddMax.set(qDDotMax);
      }
      // ---

      double qddLimited = MathTools.clipToMinMax(qddDesired.getDoubleValue(), qddMin.getDoubleValue(), qddMax.getDoubleValue());
      qddDesiredLimited.set(qddLimited);
   }
}
