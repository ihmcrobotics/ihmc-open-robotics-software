package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.HashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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
         qDDotMin = MathTools.clipToMinMax(qDDotMin, -absoluteMaximumJointAcceleration, 0.0);
         qDDotMinToPack.set(index, 0, qDDotMin);
      }
      if (!Double.isInfinite(jointLimitUpper))
      {
         double qDotMax = (jointLimitUpper - joint.getQ()) / controlDT;
         qDDotMax = (qDotMax - joint.getQd()) / controlDT;
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

      // --- do limiting here ---
      double timeHorizon = 5.0 * controlDT;
      double maxBreakAcceleration = 0.99 * absoluteMaximumJointAcceleration;
      if (!Double.isInfinite(jointLimitLower))
      {
         double distance = joint.getQ() - jointLimitLower;
         distance = Math.max(0.0, distance);

         double qDotMin = - Math.pow(distance, 2) / timeHorizon;
         qDDotMin = (qDotMin - joint.getQd()) / timeHorizon;
         qDDotMin = MathTools.clipToMinMax(qDDotMin, -absoluteMaximumJointAcceleration, maxBreakAcceleration);
      }
      if (!Double.isInfinite(jointLimitUpper))
      {
         double distance = jointLimitUpper - joint.getQ();
         distance = Math.max(0.0, distance);

         double qDotMax = Math.pow(distance, 2) / timeHorizon;
         qDDotMax = (qDotMax - joint.getQd()) / timeHorizon;
         qDDotMax = MathTools.clipToMinMax(qDDotMax, -maxBreakAcceleration, absoluteMaximumJointAcceleration);
      }
      // ---

      qDDotMinToPack.set(index, 0, qDDotMin);
      qDDotMaxToPack.set(index, 0, qDDotMax);
   }
}
