package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.HashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class InverseDynamicsQPBoundCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DenseMatrix64F jointsRangeOfMotion;
   private final DenseMatrix64F jointLowerLimits;
   private final DenseMatrix64F jointUpperLimits;
   private final HashMap<OneDoFJoint, JointLimitEnforcement> jointLimitTypes = new HashMap<>();
   private final HashMap<OneDoFJoint, JointLimitParameters> jointLimitParameters = new HashMap<>();

   private final HashMap<OneDoFJoint, DoubleYoVariable> filterAlphas = new HashMap<>();
   private final HashMap<OneDoFJoint, DoubleYoVariable> velocityGains = new HashMap<>();
   private final HashMap<OneDoFJoint, AlphaFilteredYoVariable> filteredLowerLimits = new HashMap<>();
   private final HashMap<OneDoFJoint, AlphaFilteredYoVariable> filteredUpperLimits = new HashMap<>();

   private final JointIndexHandler jointIndexHandler;
   private final OneDoFJoint[] oneDoFJoints;

   private final double controlDT;

   public InverseDynamicsQPBoundCalculator(JointIndexHandler jointIndexHandler, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.jointIndexHandler = jointIndexHandler;

      oneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      int numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      jointsRangeOfMotion = new DenseMatrix64F(numberOfDoFs , 1);
      jointLowerLimits = new DenseMatrix64F(numberOfDoFs, 1);
      jointUpperLimits = new DenseMatrix64F(numberOfDoFs, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         int jointIndex = jointIndexHandler.getOneDoFJointIndex(joint);
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();

         jointsRangeOfMotion.set(jointIndex, 0, jointLimitUpper - jointLimitLower);
         jointLowerLimits.set(jointIndex, 0, jointLimitLower);
         jointUpperLimits.set(jointIndex, 0, jointLimitUpper);
         jointLimitTypes.put(joint, JointLimitEnforcement.DEFAULT);
         jointLimitParameters.put(joint, new JointLimitParameters());

         DoubleYoVariable filterAlpha = new DoubleYoVariable("joint_limit_filter_alpha_" + joint.getName(), parentRegistry);
         filterAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(Double.POSITIVE_INFINITY, controlDT));
         AlphaFilteredYoVariable filteredLowerLimit = new AlphaFilteredYoVariable("qdd_min_filter_" + joint.getName(), registry, filterAlpha);
         AlphaFilteredYoVariable filteredUpperLimit = new AlphaFilteredYoVariable("qdd_max_filter_" + joint.getName(), registry, filterAlpha);
         filterAlphas.put(joint, filterAlpha);
         filteredLowerLimits.put(joint, filteredLowerLimit);
         filteredUpperLimits.put(joint, filteredUpperLimit);
         
         DoubleYoVariable velocityGain = new DoubleYoVariable("joint_limit_velocity_gain_" + joint.getName(), registry);
         velocityGains.put(joint, velocityGain);
      }

      parentRegistry.addChild(registry);
   }

   public void submitJointLimitReductionCommand(JointLimitReductionCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJoints(); commandJointIndex++)
      {
         OneDoFJoint joint = command.getJoint(commandJointIndex);
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
         OneDoFJoint joint = command.getJoint(idx);
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

   public void computeJointVelocityLimits(DenseMatrix64F qDotMinToPack, DenseMatrix64F qDotMaxToPack)
   {
      CommonOps.fill(qDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDotMaxToPack, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

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

   private void computeVelocityLimitDefault(OneDoFJoint joint, DenseMatrix64F qDotMinToPack, DenseMatrix64F qDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      if (!Double.isInfinite(jointLimitLower))
         qDotMinToPack.set(index, 0, (jointLimitLower - joint.getQ()) / controlDT);
      double jointLimitUpper = jointUpperLimits.get(index, 0);
      if (!Double.isInfinite(jointLimitUpper))
         qDotMaxToPack.set(index, 0, (jointLimitUpper - joint.getQ()) / controlDT);
   }

   public void computeJointAccelerationLimits(double absoluteMaximumJointAcceleration, DenseMatrix64F qDDotMinToPack, DenseMatrix64F qDDotMaxToPack)
   {
      CommonOps.fill(qDDotMinToPack, Double.NEGATIVE_INFINITY);
      CommonOps.fill(qDDotMaxToPack, Double.POSITIVE_INFINITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

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

   private void computeAccelerationLimitDefault(OneDoFJoint joint, double absoluteMaximumJointAcceleration,
         DenseMatrix64F qDDotMinToPack, DenseMatrix64F qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

      double qDDotMin = Double.NEGATIVE_INFINITY;
      double qDDotMax = Double.POSITIVE_INFINITY;

      if (!Double.isInfinite(jointLimitLower))
      {
         double qDotMin = (jointLimitLower - joint.getQ()) / controlDT;
         qDDotMin = (qDotMin - joint.getQd()) / controlDT;
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, 0.0);
         qDDotMinToPack.set(index, 0, qDDotMin);
      }
      if (!Double.isInfinite(jointLimitUpper))
      {
         double qDotMax = (jointLimitUpper - joint.getQ()) / controlDT;
         qDDotMax = (qDotMax - joint.getQd()) / controlDT;
         qDDotMax = MathTools.clamp(qDDotMax, -0.0, absoluteMaximumJointAcceleration);
         qDDotMaxToPack.set(index, 0, qDDotMax);
      }
   }

   private void computeAccelerationLimitRestrictive(OneDoFJoint joint, double absoluteMaximumJointAcceleration,
         DenseMatrix64F qDDotMinToPack, DenseMatrix64F qDDotMaxToPack)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

      double qDDotMin = -absoluteMaximumJointAcceleration;
      double qDDotMax = absoluteMaximumJointAcceleration;

      JointLimitParameters params = jointLimitParameters.get(joint);
      double slope = params.getMaxAbsJointVelocity() / Math.pow(params.getJointLimitDistanceForMaxVelocity(), 2.0);

      // --- do limiting here ---
      double maxBreakAcceleration = 0.99 * absoluteMaximumJointAcceleration;
      if (!Double.isInfinite(jointLimitLower))
      {
         double distance = joint.getQ() - jointLimitLower;
         distance = Math.max(0.0, distance);

         double qDotMin = - Math.pow(distance, 2) * slope;
         qDDotMin = (qDotMin - joint.getQd()) * velocityGains.get(joint).getDoubleValue();
         qDDotMin = MathTools.clamp(qDDotMin, -absoluteMaximumJointAcceleration, maxBreakAcceleration);
      }
      if (!Double.isInfinite(jointLimitUpper))
      {
         double distance = jointLimitUpper - joint.getQ();
         distance = Math.max(0.0, distance);

         double qDotMax = Math.pow(distance, 2) * slope;
         qDDotMax = (qDotMax - joint.getQd()) * velocityGains.get(joint).getDoubleValue();
         qDDotMax = MathTools.clamp(qDDotMax, -maxBreakAcceleration, absoluteMaximumJointAcceleration);
      }
      // ---

      filteredLowerLimits.get(joint).update(qDDotMin);
      filteredUpperLimits.get(joint).update(qDDotMax);
      qDDotMinToPack.set(index, 0, filteredLowerLimits.get(joint).getDoubleValue());
      qDDotMaxToPack.set(index, 0, filteredUpperLimits.get(joint).getDoubleValue());
   }
}
