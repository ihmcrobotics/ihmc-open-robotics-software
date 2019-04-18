package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.HashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyControllerBoundCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DenseMatrix64F jointsRangeOfMotion;
   private final DenseMatrix64F jointLowerLimits;
   private final DenseMatrix64F jointUpperLimits;
   private final HashMap<OneDoFJointBasics, JointLimitEnforcement> jointLimitTypes = new HashMap<>();
   private final HashMap<OneDoFJointBasics, JointLimitParameters> jointLimitParameters = new HashMap<>();
   private final HashMap<OneDoFJointBasics, JointLimitData> jointLimitData = new HashMap<>();

   private final HashMap<OneDoFJointBasics, YoDouble> filterAlphas = new HashMap<>();
   private final HashMap<OneDoFJointBasics, YoDouble> velocityGains = new HashMap<>();
   private final HashMap<OneDoFJointBasics, AlphaFilteredYoVariable> filteredLowerLimits = new HashMap<>();
   private final HashMap<OneDoFJointBasics, AlphaFilteredYoVariable> filteredUpperLimits = new HashMap<>();
   private final HashMap<OneDoFJointBasics, YoDouble> lowerHardLimits = new HashMap<>();
   private final HashMap<OneDoFJointBasics, YoDouble> upperHardLimits = new HashMap<>();

   private final YoBoolean areJointVelocityLimitsConsidered = new YoBoolean("areJointVelocityLimitsConsidered", registry);

   private final JointIndexHandler jointIndexHandler;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final double controlDT;

   public WholeBodyControllerBoundCalculator(JointIndexHandler jointIndexHandler, double controlDT, boolean considerJointVelocityLimits,
                                             YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.jointIndexHandler = jointIndexHandler;

      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      jointsRangeOfMotion = new DenseMatrix64F(numberOfDoFs, 1);
      jointLowerLimits = new DenseMatrix64F(numberOfDoFs, 1);
      jointUpperLimits = new DenseMatrix64F(numberOfDoFs, 1);

      areJointVelocityLimitsConsidered.set(considerJointVelocityLimits);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();

         jointsRangeOfMotion.set(jointIndex, 0, jointLimitUpper - jointLimitLower);
         jointLowerLimits.set(jointIndex, 0, jointLimitLower);
         jointUpperLimits.set(jointIndex, 0, jointLimitUpper);
         jointLimitTypes.put(joint, JointLimitEnforcement.DEFAULT);
         jointLimitParameters.put(joint, new JointLimitParameters());

         YoDouble filterAlpha = new YoDouble("joint_limit_filter_alpha_" + joint.getName(), parentRegistry);
         filterAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(Double.POSITIVE_INFINITY, controlDT));
         AlphaFilteredYoVariable filteredLowerLimit = new AlphaFilteredYoVariable("qdd_min_filter_" + joint.getName(), registry, filterAlpha);
         AlphaFilteredYoVariable filteredUpperLimit = new AlphaFilteredYoVariable("qdd_max_filter_" + joint.getName(), registry, filterAlpha);
         YoDouble hardLowerLimit = new YoDouble("qdd_min_hard_" + joint.getName(), registry);
         YoDouble hardUpperLimit = new YoDouble("qdd_max_hard_" + joint.getName(), registry);
         filterAlphas.put(joint, filterAlpha);
         filteredLowerLimits.put(joint, filteredLowerLimit);
         filteredUpperLimits.put(joint, filteredUpperLimit);
         lowerHardLimits.put(joint, hardLowerLimit);
         upperHardLimits.put(joint, hardUpperLimit);


         YoDouble velocityGain = new YoDouble("joint_limit_velocity_gain_" + joint.getName(), registry);
         velocityGains.put(joint, velocityGain);
      }

      parentRegistry.addChild(registry);
   }

   public void submitJointLimitReductionCommand(JointLimitReductionCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJoints(); commandJointIndex++)
      {
         OneDoFJointBasics joint = command.getJoint(commandJointIndex);
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double originalJointLimitLower = joint.getJointLimitLower();
         double originalJointLimitUpper = joint.getJointLimitUpper();
         double limitReduction = command.getJointLimitReductionFactor(commandJointIndex) * jointsRangeOfMotion.get(jointIndex, 0);
         jointLowerLimits.set(jointIndex, 0, originalJointLimitLower + limitReduction);
         jointUpperLimits.set(jointIndex, 0, originalJointLimitUpper - limitReduction);
      }
   }

   public void submitJointLimitEnforcementMethodCommand(JointLimitEnforcementMethodCommand command)
   {
      for (int idx = 0; idx < command.getNumberOfJoints(); idx++)
      {
         OneDoFJointBasics joint = command.getJoint(idx);
         jointLimitTypes.put(joint, command.getJointLimitReductionFactor(idx));
         JointLimitParameters params = command.getJointLimitParameters(idx);
         if (params != null)
         {
            jointLimitParameters.get(joint).set(params);
            filterAlphas.get(joint).set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(params.getJointLimitFilterBreakFrequency(), controlDT));
            velocityGains.get(joint).set(params.getVelocityControlGain());
         }
      }
   }

   public void submitJointLimitEnforcementCommand(JointLimitEnforcementCommand command)
   {
      for (int idx = 0; idx < command.getNumberOfJoints(); idx++)
      {
         OneDoFJointBasics joint = command.getJoint(idx);
         JointLimitData data = command.getJointLimitData(idx);
         if (data != null)
            jointLimitData.put(joint, data);
      }
   }

   public void computeJointVelocityLimits(DenseMatrix64F qDotMinToPack, DenseMatrix64F qDotMaxToPack)
   {
      CommonOps.fill(qDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDotMaxToPack, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];

         switch (jointLimitTypes.get(joint))
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

   private void computeVelocityLimitDefault(OneDoFJointBasics joint, DenseMatrix64F qDotMinToPack, DenseMatrix64F qDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);

      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

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

   public void computeJointAccelerationLimits(double absoluteMaximumJointAcceleration, DenseMatrix64F qDDotMinToPack, DenseMatrix64F qDDotMaxToPack)
   {
      CommonOps.fill(qDDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDDotMaxToPack, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];

         switch (jointLimitTypes.get(joint))
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

   private void computeAccelerationLimitDefault(OneDoFJointBasics joint, double absoluteMaximumJointAcceleration, DenseMatrix64F qDDotMinToPack,
                                                DenseMatrix64F qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

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

      if (!Double.isInfinite(jointLimitLower) || !Double.isInfinite(velocityLimitLower))
      {
         double qDotMinFromFD = (jointLimitLower - joint.getQ()) / controlDT;
         double qDotMin = MathTools.clamp(qDotMinFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMin = (qDotMin - joint.getQd()) / controlDT;
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, 0.0);
         qDDotMinToPack.set(index, 0, qDDotMin);
         lowerHardLimits.get(joint).set(qDDotMin);
      }
      if (!Double.isInfinite(jointLimitUpper) || !Double.isInfinite(velocityLimitUpper))
      {
         double qDotMaxFromFD = (jointLimitUpper - joint.getQ()) / controlDT;
         double qDotMax = MathTools.clamp(qDotMaxFromFD, velocityLimitLower, velocityLimitUpper);

         qDDotMax = (qDotMax - joint.getQd()) / controlDT;
         qDDotMax = MathTools.clamp(qDDotMax, -0.0, absoluteMaximumJointAcceleration);
         qDDotMaxToPack.set(index, 0, qDDotMax);
         upperHardLimits.get(joint).set(qDDotMax);
      }
   }

   private void computeAccelerationLimitRestrictive(OneDoFJointBasics joint, double absoluteMaximumJointAcceleration, DenseMatrix64F qDDotMinToPack,
                                                    DenseMatrix64F qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

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

      JointLimitParameters params = jointLimitParameters.get(joint);
      double slope = params.getMaxAbsJointVelocity() / Math.pow(params.getJointLimitDistanceForMaxVelocity(), 2.0);

      // --- do limiting here ---
      double maxBreakAcceleration = 0.99 * absoluteMaximumJointAcceleration;
      if (!Double.isInfinite(jointLimitLower) || !Double.isInfinite(velocityLimitLower))
      {
         double distance = joint.getQ() - jointLimitLower;
         distance = Math.max(0.0, distance);

         double qDotMinFromFD = -Math.pow(distance, 2) * slope;
         double qDotMin = MathTools.clamp(qDotMinFromFD, velocityLimitLower, velocityLimitUpper);
         qDDotMin = (qDotMin - joint.getQd()) * velocityGains.get(joint).getDoubleValue();
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, maxBreakAcceleration);
      }

      if (!Double.isInfinite(jointLimitUpper) || !Double.isInfinite(velocityLimitUpper))
      {
         double distance = jointLimitUpper - joint.getQ();
         distance = Math.max(0.0, distance);

         double qDotMaxFromFD = Math.pow(distance, 2) * slope;
         double qDotMax = MathTools.clamp(qDotMaxFromFD, velocityLimitLower, velocityLimitUpper);
         qDDotMax = (qDotMax - joint.getQd()) * velocityGains.get(joint).getDoubleValue();
         qDDotMax = MathTools.clamp(qDDotMax, -maxBreakAcceleration, absoluteMaximumJointAcceleration);
      }
      // ---

      filteredLowerLimits.get(joint).update(qDDotMin);
      filteredUpperLimits.get(joint).update(qDDotMax);
      qDDotMinToPack.set(index, 0, filteredLowerLimits.get(joint).getDoubleValue());
      qDDotMaxToPack.set(index, 0, filteredUpperLimits.get(joint).getDoubleValue());
   }

   public void enforceJointTorqueLimits(LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList)
   {
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);

         if (jointLimitData.containsKey(joint))
         {
            JointLimitData jointLimitData = this.jointLimitData.get(joint);
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
