package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.math.DeadbandTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyControllerBoundCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixRMaj jointsRangeOfMotion;
   private final DMatrixRMaj jointLowerLimits;
   private final DMatrixRMaj jointUpperLimits;
   private final JointLimitEnforcement[] jointLimitTypes;
   private final JointLimitParameters[] jointLimitParameters;
   private final JointLimitData[] jointLimitData;

   private final YoDouble[] filterAlphas;
   private final YoDouble[] velocityDeadbandSizes;
   private final YoDouble[] romMarginFractions;
   private final YoDouble[] velocityGains;
   private final AlphaFilteredYoVariable[] filteredLowerLimits;
   private final AlphaFilteredYoVariable[] filteredUpperLimits;
   private final YoDouble[] lowerHardLimits;
   private final YoDouble[] upperHardLimits;

   private final YoBoolean areJointVelocityLimitsConsidered = new YoBoolean("areJointVelocityLimitsConsidered", registry);

   private final JointIndexHandler jointIndexHandler;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final double controlDT;

   public WholeBodyControllerBoundCalculator(JointIndexHandler jointIndexHandler,
                                             double controlDT,
                                             boolean considerJointVelocityLimits,
                                             YoRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.jointIndexHandler = jointIndexHandler;

      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      jointsRangeOfMotion = new DMatrixRMaj(numberOfDoFs, 1);
      jointLowerLimits = new DMatrixRMaj(numberOfDoFs, 1);
      jointUpperLimits = new DMatrixRMaj(numberOfDoFs, 1);

      jointLimitTypes = new JointLimitEnforcement[numberOfDoFs];
      jointLimitParameters = new JointLimitParameters[numberOfDoFs];
      jointLimitData = new JointLimitData[numberOfDoFs];

      filterAlphas = new YoDouble[numberOfDoFs];
      velocityDeadbandSizes = new YoDouble[numberOfDoFs];
      romMarginFractions = new YoDouble[numberOfDoFs];
      velocityGains = new YoDouble[numberOfDoFs];
      filteredLowerLimits = new AlphaFilteredYoVariable[numberOfDoFs];
      filteredUpperLimits = new AlphaFilteredYoVariable[numberOfDoFs];
      lowerHardLimits = new YoDouble[numberOfDoFs];
      upperHardLimits = new YoDouble[numberOfDoFs];

      areJointVelocityLimitsConsidered.set(considerJointVelocityLimits);

      for (OneDoFJointBasics joint : jointIndexHandler.getIndexedOneDoFJoints())
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();

         jointsRangeOfMotion.set(jointIndex, 0, jointLimitUpper - jointLimitLower);
         jointLowerLimits.set(jointIndex, 0, jointLimitLower);
         jointUpperLimits.set(jointIndex, 0, jointLimitUpper);
         jointLimitTypes[jointIndex] = JointLimitEnforcement.DEFAULT;
         jointLimitParameters[jointIndex] = new JointLimitParameters();
         jointLimitData[jointIndex] = new JointLimitData();

         YoDouble filterAlpha = new YoDouble("joint_limit_filter_alpha_" + joint.getName(), registry);
         YoDouble velocityDeadband = new YoDouble("joint_limit_velocity_deadband" + joint.getName(), registry);
         YoDouble romMarginFraction = new YoDouble("joint_limit_rom_margin_fraction" + joint.getName(), registry);
         filterAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(Double.POSITIVE_INFINITY, controlDT));
         AlphaFilteredYoVariable filteredLowerLimit = new AlphaFilteredYoVariable("qdd_min_filter_" + joint.getName(), registry, filterAlpha);
         AlphaFilteredYoVariable filteredUpperLimit = new AlphaFilteredYoVariable("qdd_max_filter_" + joint.getName(), registry, filterAlpha);
         YoDouble hardLowerLimit = new YoDouble("qdd_min_hard_" + joint.getName(), registry);
         YoDouble hardUpperLimit = new YoDouble("qdd_max_hard_" + joint.getName(), registry);
         filterAlphas[jointIndex] = filterAlpha;
         velocityDeadbandSizes[jointIndex] = velocityDeadband;
         romMarginFractions[jointIndex] = romMarginFraction;
         filteredLowerLimits[jointIndex] = filteredLowerLimit;
         filteredUpperLimits[jointIndex] = filteredUpperLimit;
         lowerHardLimits[jointIndex] = hardLowerLimit;
         upperHardLimits[jointIndex] = hardUpperLimit;

         YoDouble velocityGain = new YoDouble("joint_limit_velocity_gain_" + joint.getName(), registry);
         velocityGains[jointIndex] = velocityGain;
      }

      parentRegistry.addChild(registry);
   }

   /**
    * Sets whether the joint velocity limits as defined by {@link OneDoFJoint#getVelocityLimitLower()}
    * and {@link OneDoFJoint#getVelocityLimitUpper()} should be considered.
    * 
    * @param value if {@code true} the joint velocity limits will be considered, if {@code false} they
    *              will be ignored.
    */
   public void considerJointVelocityLimits(boolean value)
   {
      areJointVelocityLimitsConsidered.set(value);
   }

   public void submitJointLimitReductionCommand(JointLimitReductionCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJoints(); commandJointIndex++)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(command.getJoint(commandJointIndex));
         romMarginFractions[jointIndex].set(command.getJointLimitReductionFactor(commandJointIndex));
      }
   }

   public void submitJointLimitEnforcementMethodCommand(JointLimitEnforcementMethodCommand command)
   {
      for (int idx = 0; idx < command.getNumberOfJoints(); idx++)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(command.getJoint(idx));
         jointLimitTypes[jointIndex] = command.getJointLimitReductionFactor(idx);
         JointLimitParameters params = command.getJointLimitParameters(idx);
         if (params != null)
         {
            jointLimitParameters[jointIndex].set(params);
            filterAlphas[jointIndex].set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(params.getJointLimitFilterBreakFrequency(),
                                                                                                         controlDT));
            velocityGains[jointIndex].set(params.getVelocityControlGain());
            romMarginFractions[jointIndex].set(params.getRangeOfMotionMarginFraction());
            velocityDeadbandSizes[jointIndex].set(params.getVelocityDeadbandSize());
         }
      }
   }

   public void submitJointLimitEnforcementCommand(JointLimitEnforcementCommand command)
   {
      for (int idx = 0; idx < command.getNumberOfJoints(); idx++)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(command.getJoint(idx));
         JointLimitData data = command.getJointLimitData(idx);
         if (data != null)
            jointLimitData[jointIndex].set(data);
      }
   }

   public void computeJointVelocityLimits(DMatrixRMaj qDotMinToPack, DMatrixRMaj qDotMaxToPack)
   {
      CommonOps_DDRM.fill(qDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(qDotMaxToPack, Double.POSITIVE_INFINITY);

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);

         switch (jointLimitTypes[jointIndex])
         {
            case DEFAULT:
               computeVelocityLimitDefault(joint, qDotMinToPack, qDotMaxToPack);
               break;
            case NONE:
               break;
            default:
               throw new RuntimeException("Implement case!");
         }
      }
   }

   private void computeVelocityLimitDefault(OneDoFJointBasics joint, DMatrixRMaj qDotMinToPack, DMatrixRMaj qDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);

      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

      if (!Double.isInfinite(jointLimitUpper) && !Double.isInfinite(jointLimitLower))
      {
         double limitMargin = romMarginFractions[index].getDoubleValue() * (jointLimitUpper - jointLimitLower);
         jointLimitUpper -= limitMargin;
         jointLimitLower += limitMargin;
      }

      double velocityLimitLower;
      double velocityLimitUpper;

      if (areJointVelocityLimitsConsidered.getBooleanValue())
      {
         velocityLimitLower = joint.getVelocityLimitLower();
         velocityLimitUpper = joint.getVelocityLimitUpper();
      }
      else
      {
         velocityLimitLower = Double.NEGATIVE_INFINITY;
         velocityLimitUpper = Double.POSITIVE_INFINITY;
      }

      if (!Double.isInfinite(jointLimitLower) || !Double.isInfinite(velocityLimitLower))
      {
         double qDotMinFromFD = (jointLimitLower - joint.getQ()) / controlDT;
         double qDotMin = MathTools.clamp(qDotMinFromFD, velocityLimitLower, velocityLimitUpper);

         qDotMinToPack.set(index, 0, qDotMin);
      }

      if (!Double.isInfinite(jointLimitUpper) || !Double.isInfinite(velocityLimitUpper))
      {
         double qDotMaxFromFD = (jointLimitUpper - joint.getQ()) / controlDT;
         double qDotMax = MathTools.clamp(qDotMaxFromFD, velocityLimitLower, velocityLimitUpper);

         qDotMaxToPack.set(index, 0, qDotMax);
      }
   }

   public void computeJointAccelerationLimits(double absoluteMaximumJointAcceleration, DMatrixRMaj qDDotMinToPack, DMatrixRMaj qDDotMaxToPack)
   {
      CommonOps_DDRM.fill(qDDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(qDDotMaxToPack, Double.POSITIVE_INFINITY);

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);

         switch (jointLimitTypes[jointIndex])
         {
            case DEFAULT:
               computeAccelerationLimitDefault(joint, absoluteMaximumJointAcceleration, qDDotMinToPack, qDDotMaxToPack);
               break;
            case RESTRICTIVE:
               computeAccelerationLimitRestrictive(joint, absoluteMaximumJointAcceleration, qDDotMinToPack, qDDotMaxToPack);
               break;
            case NONE:
               break;
            default:
               throw new RuntimeException("Implement case!");
         }
      }
   }

   private void computeAccelerationLimitDefault(OneDoFJointBasics joint,
                                                double absoluteMaximumJointAcceleration,
                                                DMatrixRMaj qDDotMinToPack,
                                                DMatrixRMaj qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

      if (!Double.isInfinite(jointLimitUpper) && !Double.isInfinite(jointLimitLower))
      {
         double limitMargin = romMarginFractions[index].getDoubleValue() * (jointLimitUpper - jointLimitLower);
         jointLimitUpper -= limitMargin;
         jointLimitLower += limitMargin;
      }

      double qDDotMin = Double.NEGATIVE_INFINITY;
      double qDDotMax = Double.POSITIVE_INFINITY;

      double velocityLimitLower;
      double velocityLimitUpper;

      if (areJointVelocityLimitsConsidered.getBooleanValue())
      {
         velocityLimitLower = joint.getVelocityLimitLower();
         velocityLimitUpper = joint.getVelocityLimitUpper();
      }
      else
      {
         velocityLimitLower = Double.NEGATIVE_INFINITY;
         velocityLimitUpper = Double.POSITIVE_INFINITY;
      }

      double brakeVelocity = DeadbandTools.applyDeadband(velocityDeadbandSizes[index].getDoubleValue(), joint.getQd());
      if (!Double.isInfinite(jointLimitLower) || !Double.isInfinite(velocityLimitLower))
      {
         double qDotMinFromFD = (jointLimitLower - joint.getQ()) / controlDT;
         double qDotMin = MathTools.clamp(qDotMinFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMin = 2.0 * (qDotMin - brakeVelocity) / controlDT;
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, 0.0);
         qDDotMinToPack.set(index, 0, qDDotMin);
         lowerHardLimits[index].set(qDDotMin);
      }
      if (!Double.isInfinite(jointLimitUpper) || !Double.isInfinite(velocityLimitUpper))
      {
         double qDotMaxFromFD = (jointLimitUpper - joint.getQ()) / controlDT;
         double qDotMax = MathTools.clamp(qDotMaxFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMax = 2.0 * (qDotMax - brakeVelocity) / controlDT;
         qDDotMax = MathTools.clamp(qDDotMax, -0.0, absoluteMaximumJointAcceleration);
         qDDotMaxToPack.set(index, 0, qDDotMax);
         upperHardLimits[index].set(qDDotMax);
      }
   }

   private void computeAccelerationLimitRestrictive(OneDoFJointBasics joint,
                                                    double absoluteMaximumJointAcceleration,
                                                    DMatrixRMaj qDDotMinToPack,
                                                    DMatrixRMaj qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);

      JointLimitParameters params = jointLimitParameters[index];

      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);
      if (!Double.isInfinite(jointLimitUpper) && !Double.isInfinite(jointLimitLower))
      {
         double limitMargin = romMarginFractions[index].getDoubleValue() * (jointLimitUpper - jointLimitLower);
         jointLimitUpper -= limitMargin;
         jointLimitLower += limitMargin;
      }

      double velocityLimitLower;
      double velocityLimitUpper;

      if (areJointVelocityLimitsConsidered.getBooleanValue())
      {
         velocityLimitLower = joint.getVelocityLimitLower();
         velocityLimitUpper = joint.getVelocityLimitUpper();
      }
      else
      {
         velocityLimitLower = Double.NEGATIVE_INFINITY;
         velocityLimitUpper = Double.POSITIVE_INFINITY;
      }

      double qDDotMin = -absoluteMaximumJointAcceleration;
      double qDDotMax = absoluteMaximumJointAcceleration;

      double brakeVelocity = DeadbandTools.applyDeadband(velocityDeadbandSizes[index].getDoubleValue(), joint.getQd());
      double slope = params.getMaxAbsJointVelocity() / Math.pow(params.getJointLimitDistanceForMaxVelocity(), 2.0);

      // --- do limiting here ---
      double maxBrakeAcceleration = 0.99 * absoluteMaximumJointAcceleration;
      if (!Double.isInfinite(jointLimitLower) || !Double.isInfinite(velocityLimitLower))
      {
         double distance = joint.getQ() - jointLimitLower;
         distance = Math.max(0.0, distance);

         double qDotMinFromFD = -Math.pow(distance, 2) * slope;
         double qDotMin = MathTools.clamp(qDotMinFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMin = (qDotMin - brakeVelocity) * velocityGains[index].getDoubleValue();
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, maxBrakeAcceleration);
      }

      if (!Double.isInfinite(jointLimitUpper) || !Double.isInfinite(velocityLimitUpper))
      {
         double distance = jointLimitUpper - joint.getQ();
         distance = Math.max(0.0, distance);

         double qDotMaxFromFD = Math.pow(distance, 2) * slope;
         double qDotMax = MathTools.clamp(qDotMaxFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMax = (qDotMax - brakeVelocity) * velocityGains[index].getDoubleValue();
         qDDotMax = MathTools.clamp(qDDotMax, -maxBrakeAcceleration, absoluteMaximumJointAcceleration);
      }
      // ---

      filteredLowerLimits[index].update(qDDotMin);
      filteredUpperLimits[index].update(qDDotMax);
      qDDotMinToPack.set(index, 0, filteredLowerLimits[index].getDoubleValue());
      qDDotMaxToPack.set(index, 0, filteredUpperLimits[index].getDoubleValue());
   }

   public void enforceJointTorqueLimits(LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList)
   {
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);

         if (jointIndex < jointLimitData.length && jointLimitData[jointIndex] != null)
         {
            JointLimitData jointLimitData = this.jointLimitData[jointIndex];
            enforceJointTorqueLimit(joint, jointDesiredOutput, jointLimitData);
         }
      }
   }

   private void enforceJointTorqueLimit(OneDoFJointBasics joint, JointDesiredOutputBasics jointDesiredOutput, JointLimitData jointLimitData)
   {
      double torque;
      if (!jointDesiredOutput.hasDesiredTorque())
         torque = 0.0;
      else
         torque = jointDesiredOutput.getDesiredTorque();

      if (!jointLimitData.hasPositionSoftLowerLimit() && joint.getQ() < jointLimitData.getPositionSoftLowerLimit())
      {
         double stiffnessTorque = 0.0;
         if (jointLimitData.hasPositionLimitStiffness())
            stiffnessTorque = jointLimitData.getJointLimitStiffness() * (jointLimitData.getPositionSoftLowerLimit() - joint.getQ());

         double dampingTorque = 0.0;
         if (jointLimitData.hasPositionLimitDamping())
            dampingTorque = jointLimitData.getJointLimitDamping() * joint.getQd();

         torque = Math.max(torque, stiffnessTorque - dampingTorque);
      }

      if (!jointLimitData.hasPositionSoftUpperLimit() && joint.getQ() > jointLimitData.getPositionSoftUpperLimit())
      {
         double stiffnessTorque = 0.0;
         if (jointLimitData.hasPositionLimitStiffness())
            stiffnessTorque = jointLimitData.getJointLimitStiffness() * (jointLimitData.getPositionSoftUpperLimit() - joint.getQ());

         double dampingTorque = 0.0;
         if (jointLimitData.hasPositionLimitDamping())
            dampingTorque = jointLimitData.getJointLimitDamping() * joint.getQd();

         torque = Math.min(torque, stiffnessTorque - dampingTorque);
      }

      if (jointLimitData.hasTorqueUpperLimit())
         torque = Math.min(torque, jointLimitData.getTorqueUpperLimit());
      if (jointLimitData.hasTorqueLowerLimit())
         torque = Math.max(torque, jointLimitData.getTorqueLowerLimit());

      jointDesiredOutput.setDesiredTorque(torque);
   }
}
