package us.ihmc.exampleSimulations.jointLimits;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class JointLimitsController extends SimpleRobotController
{
   private static final double maxAbsJointVelocity = 9.0; // rad/s
   private static final double jointLimitDistanceForMaxVelocity = 30.0; // degree

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

   private final DoubleYoVariable filterAlpha = new DoubleYoVariable("FilterAlpha", registry);
   private final AlphaFilteredYoVariable lowerLimitFiltered = new AlphaFilteredYoVariable("LowerLimitFiltered", registry, filterAlpha);
   private final AlphaFilteredYoVariable upperLimitFiltered = new AlphaFilteredYoVariable("UpperLimitFiltered", registry, filterAlpha);

   private final DoubleYoVariable slope = new DoubleYoVariable("Slope", registry);

   public JointLimitsController(JointLimitsRobot robot, double controlDT)
   {
      this.robot = robot;

      magnitude.set(0.75);
      frequency.set(1.0);
      gain.set(10.0);

      lowerLimit.set(robot.getLowerLimit());
      upperLimit.set(robot.getUpperLimit());

      filterAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(10.0, controlDT));
      slope.set(maxAbsJointVelocity / Math.pow((jointLimitDistanceForMaxVelocity * Math.PI/180.0), 2.0));

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
      double timeHorizon = 0.02;
      double absoluteMaximumJointAcceleration = qddMaxAbs.getDoubleValue();
      double maxBreakAcceleration = 4.0;
      if (!Double.isInfinite(lowerLimit.getDoubleValue()))
      {
         // if we enter the limit do not explode:
         double distance = robot.getQ() - lowerLimit.getDoubleValue();
         distance = Math.max(0.0, distance);

         double qDotMin = -Math.pow(distance, 2) * slope.getDoubleValue();
         double qDDotMin = (qDotMin - robot.getQd()) / timeHorizon;
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, maxBreakAcceleration);

         lowerLimitFiltered.update(qDDotMin);
         qddMin.set(lowerLimitFiltered.getDoubleValue());
      }
      if (!Double.isInfinite(upperLimit.getDoubleValue()))
      {
         // if we enter the limit do not explode:
         double distance = upperLimit.getDoubleValue() - robot.getQ();
         distance = Math.max(0.0, distance);

         double qDotMax = Math.pow(distance, 2) * slope.getDoubleValue();
         double qDDotMax = (qDotMax - robot.getQd()) / timeHorizon;
         qDDotMax = MathTools.clamp(qDDotMax, -maxBreakAcceleration, absoluteMaximumJointAcceleration);

         upperLimitFiltered.update(qDDotMax);
         qddMax.set(upperLimitFiltered.getDoubleValue());
      }
      // ---

      double qddLimited = MathTools.clamp(qddDesired.getDoubleValue(), qddMin.getDoubleValue(), qddMax.getDoubleValue());
      qddDesiredLimited.set(qddLimited);
   }
}
