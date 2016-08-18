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

   private final JointIndexHandler jointIndexHandler;
   private final OneDoFJoint[] oneDoFJoints;

   private final double controlDT;

   private final DoubleYoVariable jointLimitVelocityGain = new DoubleYoVariable("RestrictiveLimitVelocityGain", registry);
   private final DoubleYoVariable alphaLimits = new DoubleYoVariable("AlphaLimits", registry);
   private final HashMap<OneDoFJoint, AlphaFilteredYoVariable> filteredLowerLimits = new HashMap<>();
   private final HashMap<OneDoFJoint, AlphaFilteredYoVariable> filteredUpperLimits = new HashMap<>();

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

         AlphaFilteredYoVariable filteredLowerLimit = new AlphaFilteredYoVariable(joint.getName() + "_filteredLimitLower", registry, alphaLimits);
         AlphaFilteredYoVariable filteredUpperLimit = new AlphaFilteredYoVariable(joint.getName() + "_filteredLimitUpper", registry, alphaLimits);
         filteredLowerLimits.put(joint, filteredLowerLimit);
         filteredUpperLimits.put(joint, filteredUpperLimit);
      }

      alphaLimits.set(0.0);
      jointLimitVelocityGain.set(0.3);
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
         case RESTRICTIVE:
            computeVelocityLimitDefault(joint, qDotMinToPack, qDotMaxToPack);
            computeVelocityLimitRestrictive(joint, qDotMinToPack, qDotMaxToPack);
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

   private void computeVelocityLimitRestrictive(OneDoFJoint joint, DenseMatrix64F qDotMinToPack, DenseMatrix64F qDotMaxToPack)
   {
      // TODO Auto-generated method stub
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
            computeAccelerationLimitDefault(joint, absoluteMaximumJointAcceleration, qDDotMinToPack, qDDotMaxToPack, controlDT);
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
         DenseMatrix64F qDDotMinToPack, DenseMatrix64F qDDotMaxToPack, double timeHorizon)
   {
      int index = jointIndexHandler.getOneDoFJointIndex(joint);
      double jointLimitLower = jointLowerLimits.get(index, 0);
      double jointLimitUpper = jointUpperLimits.get(index, 0);

      double qDDotMin = Double.NEGATIVE_INFINITY;
      double qDDotMax = Double.POSITIVE_INFINITY;

      if (!Double.isInfinite(jointLimitLower))
      {
         double qDotMin = (jointLimitLower - joint.getQ()) / timeHorizon;
         qDDotMin = (qDotMin - joint.getQd()) / timeHorizon;
         qDDotMin = MathTools.clipToMinMax(qDDotMin, -absoluteMaximumJointAcceleration, 0.0);
         qDDotMinToPack.set(index, 0, qDDotMin);
      }
      if (!Double.isInfinite(jointLimitUpper))
      {
         double qDotMax = (jointLimitUpper - joint.getQ()) / timeHorizon;
         qDDotMax = (qDotMax - joint.getQd()) / timeHorizon;
         qDDotMax = MathTools.clipToMinMax(qDDotMax, -0.0, absoluteMaximumJointAcceleration);
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

      if (!Double.isInfinite(jointLimitLower))
      {
         double qBreak = jointLimitLower - jointLimitVelocityGain.getDoubleValue() * joint.getQd();
         if (joint.getQ() < qBreak)
         {
            double gain = (qBreak - joint.getQ()) / (qBreak - jointLimitLower);
            gain = MathTools.clipToMinMax(gain, 0.0, 1.0);
            qDDotMin += 1.99 * absoluteMaximumJointAcceleration * gain;
         }
      }
      if (!Double.isInfinite(jointLimitUpper))
      {
         double qBreak = jointLimitUpper - jointLimitVelocityGain.getDoubleValue() * joint.getQd();
         if (joint.getQ() > qBreak)
         {
            double gain = (joint.getQ() - qBreak) / (jointLimitUpper - qBreak);
            gain = MathTools.clipToMinMax(gain, 0.0, 1.0);
            qDDotMax -= 1.99 * absoluteMaximumJointAcceleration * gain;
         }
      }

      filteredLowerLimits.get(joint).update(qDDotMin);
      filteredUpperLimits.get(joint).update(qDDotMax);

      qDDotMinToPack.set(index, 0, filteredLowerLimits.get(joint).getDoubleValue());
      qDDotMaxToPack.set(index, 0, filteredUpperLimits.get(joint).getDoubleValue());
   }
}
