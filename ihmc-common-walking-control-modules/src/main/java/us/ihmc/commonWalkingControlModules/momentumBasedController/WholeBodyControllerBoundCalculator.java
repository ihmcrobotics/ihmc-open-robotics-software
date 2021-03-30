package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

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
   private final List<JointLimitEnforcement> jointLimitTypes = new ArrayList<>();
   private final List<JointLimitParameters> jointLimitParameters = new ArrayList<>();
   private final List<JointLimitData> jointLimitData = new ArrayList<>();

   private final List<YoDouble> filterAlphas = new ArrayList<>();
   private final List<YoDouble> velocityDeadbandSizes = new ArrayList<>();
   private final List<YoDouble> romMarginFractions = new ArrayList<>();
   private final List<YoDouble> velocityGains = new ArrayList<>();
   private final List<AlphaFilteredYoVariable> filteredLowerLimits = new ArrayList<>();
   private final List<AlphaFilteredYoVariable> filteredUpperLimits = new ArrayList<>();
   private final List<YoDouble> lowerHardLimits = new ArrayList<>();
   private final List<YoDouble> upperHardLimits = new ArrayList<>();

   private final YoBoolean areJointVelocityLimitsConsidered = new YoBoolean("areJointVelocityLimitsConsidered", registry);

   private final JointIndexHandler jointIndexHandler;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final double controlDT;

   public WholeBodyControllerBoundCalculator(JointIndexHandler jointIndexHandler, double controlDT, boolean considerJointVelocityLimits,
                                             YoRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.jointIndexHandler = jointIndexHandler;

      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      jointsRangeOfMotion = new DMatrixRMaj(numberOfDoFs, 1);
      jointLowerLimits = new DMatrixRMaj(numberOfDoFs, 1);
      jointUpperLimits = new DMatrixRMaj(numberOfDoFs, 1);

      areJointVelocityLimitsConsidered.set(considerJointVelocityLimits);

      for (OneDoFJointBasics joint : jointIndexHandler.getIndexedOneDoFJoints())
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();

         jointsRangeOfMotion.set(jointIndex, 0, jointLimitUpper - jointLimitLower);
         jointLowerLimits.set(jointIndex, 0, jointLimitLower);
         jointUpperLimits.set(jointIndex, 0, jointLimitUpper);
         jointLimitTypes.set(jointIndex, JointLimitEnforcement.DEFAULT);
         jointLimitParameters.set(jointIndex, new JointLimitParameters());

         YoDouble filterAlpha = new YoDouble("joint_limit_filter_alpha_" + joint.getName(), parentRegistry);
         YoDouble velocityDeadband = new YoDouble("joint_limit_velocity_deadband" + joint.getName(), parentRegistry);
         YoDouble romMarginFraction = new YoDouble("joint_limit_rom_margin_fraction" + joint.getName(), parentRegistry);
         filterAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(Double.POSITIVE_INFINITY, controlDT));
         AlphaFilteredYoVariable filteredLowerLimit = new AlphaFilteredYoVariable("qdd_min_filter_" + joint.getName(), registry, filterAlpha);
         AlphaFilteredYoVariable filteredUpperLimit = new AlphaFilteredYoVariable("qdd_max_filter_" + joint.getName(), registry, filterAlpha);
         YoDouble hardLowerLimit = new YoDouble("qdd_min_hard_" + joint.getName(), registry);
         YoDouble hardUpperLimit = new YoDouble("qdd_max_hard_" + joint.getName(), registry);
         filterAlphas.set(jointIndex, filterAlpha);
         velocityDeadbandSizes.set(jointIndex, velocityDeadband);
         romMarginFractions.set(jointIndex, romMarginFraction);
         filteredLowerLimits.set(jointIndex, filteredLowerLimit);
         filteredUpperLimits.set(jointIndex, filteredUpperLimit);
         lowerHardLimits.set(jointIndex, hardLowerLimit);
         upperHardLimits.set(jointIndex, hardUpperLimit);

         YoDouble velocityGain = new YoDouble("joint_limit_velocity_gain_" + joint.getName(), registry);
         velocityGains.set(jointIndex, velocityGain);
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
         romMarginFractions.get(jointIndex).set(command.getJointLimitReductionFactor(commandJointIndex));
      }
   }

   public void submitJointLimitEnforcementMethodCommand(JointLimitEnforcementMethodCommand command)
   {
      for (int idx = 0; idx < command.getNumberOfJoints(); idx++)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(command.getJoint(idx));
         jointLimitTypes.set(jointIndex, command.getJointLimitReductionFactor(idx));
         JointLimitParameters params = command.getJointLimitParameters(idx);
         if (params != null)
         {
            jointLimitParameters.get(jointIndex).set(params);
            filterAlphas.get(jointIndex).set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(params.getJointLimitFilterBreakFrequency(), controlDT));
            velocityGains.get(jointIndex).set(params.getVelocityControlGain());
            romMarginFractions.get(jointIndex).set(params.getRangeOfMotionMarginFraction());
            velocityDeadbandSizes.get(jointIndex).set(params.getVelocityDeadbandSize());
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
            jointLimitData.set(jointIndex, data);
      }
   }

   public void computeJointVelocityLimits(DMatrixRMaj qDotMinToPack, DMatrixRMaj qDotMaxToPack)
   {
      CommonOps_DDRM.fill(qDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(qDotMaxToPack, Double.POSITIVE_INFINITY);

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);

         switch (jointLimitTypes.get(jointIndex))
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

      double limitMargin = romMarginFractions.get(index).getDoubleValue() * (jointLimitUpper - jointLimitLower);
      jointLimitUpper -= limitMargin;
      jointLimitLower += limitMargin;

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

      for ( OneDoFJointBasics joint : oneDoFJoints)
      {
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);

         switch (jointLimitTypes.get(jointIndex))
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

   private void computeAccelerationLimitDefault(OneDoFJointBasics joint, double absoluteMaximumJointAcceleration, DMatrixRMaj qDDotMinToPack,
                                                DMatrixRMaj qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

      double limitMargin = romMarginFractions.get(index).getDoubleValue() * (jointLimitUpper - jointLimitLower);
      jointLimitUpper -= limitMargin;
      jointLimitLower += limitMargin;

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

      double brakeVelocity = DeadbandTools.applyDeadband(velocityDeadbandSizes.get(index).getDoubleValue(), joint.getQd());
      if (!Double.isInfinite(jointLimitLower) || !Double.isInfinite(velocityLimitLower))
      {
         double qDotMinFromFD = (jointLimitLower - joint.getQ()) / controlDT;
         double qDotMin = MathTools.clamp(qDotMinFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMin = 2.0 * (qDotMin - brakeVelocity) / controlDT;
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, 0.0);
         qDDotMinToPack.set(index, 0, qDDotMin);
         lowerHardLimits.get(index).set(qDDotMin);
      }
      if (!Double.isInfinite(jointLimitUpper) || !Double.isInfinite(velocityLimitUpper))
      {
         double qDotMaxFromFD = (jointLimitUpper - joint.getQ()) / controlDT;
         double qDotMax = MathTools.clamp(qDotMaxFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMax = 2.0 * (qDotMax - brakeVelocity) / controlDT;
         qDDotMax = MathTools.clamp(qDDotMax, -0.0, absoluteMaximumJointAcceleration);
         qDDotMaxToPack.set(index, 0, qDDotMax);
         upperHardLimits.get(index).set(qDDotMax);
      }
   }

   private void computeAccelerationLimitRestrictive(OneDoFJointBasics joint, double absoluteMaximumJointAcceleration, DMatrixRMaj qDDotMinToPack,
                                                    DMatrixRMaj qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

      double limitMargin = romMarginFractions.get(index).getDoubleValue() * (jointLimitUpper - jointLimitLower);
      jointLimitUpper -= limitMargin;
      jointLimitLower += limitMargin;

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

      JointLimitParameters params = jointLimitParameters.get(index);
      double brakeVelocity = DeadbandTools.applyDeadband(velocityDeadbandSizes.get(index).getDoubleValue(), joint.getQd());
      double slope = params.getMaxAbsJointVelocity() / Math.pow(params.getJointLimitDistanceForMaxVelocity(), 2.0);

      // --- do limiting here ---
      double maxBrakeAcceleration = 0.99 * absoluteMaximumJointAcceleration;
      if (!Double.isInfinite(jointLimitLower) || !Double.isInfinite(velocityLimitLower))
      {
         double distance = joint.getQ() - jointLimitLower;
         distance = Math.max(0.0, distance);

         double qDotMinFromFD = -Math.pow(distance, 2) * slope;
         double qDotMin = MathTools.clamp(qDotMinFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMin = (qDotMin - brakeVelocity) * velocityGains.get(index).getDoubleValue();
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, maxBrakeAcceleration);
      }

      if (!Double.isInfinite(jointLimitUpper) || !Double.isInfinite(velocityLimitUpper))
      {
         double distance = jointLimitUpper - joint.getQ();
         distance = Math.max(0.0, distance);

         double qDotMaxFromFD = Math.pow(distance, 2) * slope;
         double qDotMax = MathTools.clamp(qDotMaxFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMax = (qDotMax - brakeVelocity) * velocityGains.get(index).getDoubleValue();
         qDDotMax = MathTools.clamp(qDDotMax, -maxBrakeAcceleration, absoluteMaximumJointAcceleration);
      }
      // ---

      filteredLowerLimits.get(index).update(qDDotMin);
      filteredUpperLimits.get(index).update(qDDotMax);
      qDDotMinToPack.set(index, 0, filteredLowerLimits.get(index).getDoubleValue());
      qDDotMaxToPack.set(index, 0, filteredUpperLimits.get(index).getDoubleValue());
   }

   public void enforceJointTorqueLimits(LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList)
   {
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);

         if (jointIndex < jointLimitData.size() && jointLimitData.get(jointIndex) != null)
         {
            JointLimitData jointLimitData = this.jointLimitData.get(jointIndex);
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
