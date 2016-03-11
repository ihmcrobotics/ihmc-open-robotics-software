package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects.PrivilegedConfigurationInverseKinematicsCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class JointPrivilegedConfigurationHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isJointPrivilegedConfigurationEnabled = new BooleanYoVariable("isJointPrivilegedConfigurationEnabled", registry);
   private final DoubleYoVariable defaultWeight = new DoubleYoVariable("jointPrivilegedConfigurationDefaultWeight", registry);

   private final DenseMatrix64F privilegedConfigurations;
   private final DenseMatrix64F privilegedVelocities;
   private final DenseMatrix64F weight;
   private final DenseMatrix64F selectionMatrix;

   private final DenseMatrix64F jointSquaredRangeOfMotions;
   private final DenseMatrix64F positionsAtMidRangeOfMotion;

   private final OneDoFJoint[] oneDoFJoints;
   private final Map<OneDoFJoint, MutableInt> jointIndices;

   private final int numberOfDoFs;

   public JointPrivilegedConfigurationHandler(InverseDynamicsJoint[] jointsToOptimizeFor, YoVariableRegistry parentRegistry)
   {
      oneDoFJoints = ScrewTools.filterJoints(jointsToOptimizeFor, OneDoFJoint.class);
      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(oneDoFJoints);

      privilegedConfigurations = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedVelocities = new DenseMatrix64F(numberOfDoFs, 1);
      weight = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      selectionMatrix = CommonOps.identity(numberOfDoFs);

      jointSquaredRangeOfMotions = new DenseMatrix64F(numberOfDoFs, 1);
      positionsAtMidRangeOfMotion = new DenseMatrix64F(numberOfDoFs, 1);
      jointIndices = new HashMap<>(numberOfDoFs);

      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

         jointIndices.put(joint, new MutableInt(i));

         double jointLimitUpper = joint.getJointLimitUpper();
         if (Double.isNaN(jointLimitUpper) || Double.isInfinite(jointLimitUpper))
            jointLimitUpper = Math.PI;
         double jointLimitLower = joint.getJointLimitLower();
         if (Double.isNaN(jointLimitLower) || Double.isInfinite(jointLimitLower))
            jointLimitLower = -Math.PI;
         jointSquaredRangeOfMotions.set(i, 0, MathTools.square(jointLimitUpper - jointLimitLower));
         positionsAtMidRangeOfMotion.set(i, 0, 0.5 * (jointLimitUpper + jointLimitLower));
      }

      defaultWeight.set(1.0);

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         double qd = -2.0 * (joint.getQ() - privilegedConfigurations.get(i, 0)) / jointSquaredRangeOfMotions.get(i, 0);
         privilegedVelocities.set(i, 0, qd);
      }
   }

   public void submitPrivilegedConfigurationInverseKinematicsCommand(PrivilegedConfigurationInverseKinematicsCommand command)
   {
      isJointPrivilegedConfigurationEnabled.set(command.isEnabled());

      if (command.hasNewDefaultWeight())
      {
         defaultWeight.set(command.getDefaultWeight());
         updateWeights();
      }

      if (command.hasNewPrivilegedConfigurationOption())
      {
         switch (command.getPrivilegedConfigurationOption())
         {
         case AT_CURRENT:
            setPrivilegedConfigurationAtCurrent();
            break;
         case AT_ZERO:
            setPrivilegedConfigurationAtZero();
            break;
         case AT_MID_RANGE:
            setPrivilegedConfigurationAtMidRange();
            break;
         default:
            throw new RuntimeException("Cannot handle the PrivilegedConfigurationOption:" + command.getPrivilegedConfigurationOption());
         }
      }

      for (int i = 0; i < command.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         int jointIndex = jointIndices.get(joint).intValue();

         if (command.hasNewPrivilegedConfiguration(i))
            privilegedConfigurations.set(jointIndex, 0, command.getPrivilegedConfiguration(i));

         if (command.hasNewWeight(i))
            weight.set(jointIndex, jointIndex, command.getWeight(i));
      }
   }

   private void updateWeights()
   {
      for (int i = 0; i < numberOfDoFs; i++)
         weight.set(i, i, defaultWeight.getDoubleValue());
   }

   private void setPrivilegedConfigurationAtCurrent()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         privilegedConfigurations.set(i, 0, joint.getQ());
      }
   }

   private void setPrivilegedConfigurationAtZero()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         privilegedConfigurations.set(i, 0, 0.0);
      }
   }

   private void setPrivilegedConfigurationAtMidRange()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         privilegedConfigurations.set(i, 0, positionsAtMidRangeOfMotion.get(i));
      }
   }

   public boolean isEnabled()
   {
      return isJointPrivilegedConfigurationEnabled.getBooleanValue();
   }

   public DenseMatrix64F getPrivilegedJointVelocities()
   {
      return privilegedVelocities;
   }

   public DenseMatrix64F getWeight()
   {
      return weight;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public OneDoFJoint[] getJoints()
   {
      return oneDoFJoints;
   }
}
