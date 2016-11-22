package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

/**
 * This class computes the input for the optimization based on the desired privileged configuration commands.
 */
public class JointPrivilegedConfigurationHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isJointPrivilegedConfigurationEnabled = new BooleanYoVariable("isJointPrivilegedConfigurationEnabled", registry);
   private final DoubleYoVariable weight = new DoubleYoVariable("jointPrivilegedConfigurationDefaultWeight", registry);
   private final DoubleYoVariable configurationGain = new DoubleYoVariable("jointPrivilegedConfigurationGain", registry);
   private final DoubleYoVariable velocityGain = new DoubleYoVariable("jointPrivilegedVelocityGain", registry);
   private final DoubleYoVariable maxVelocity = new DoubleYoVariable("jointPrivilegedConfigurationMaxVelocity", registry);
   private final DoubleYoVariable maxAcceleration = new DoubleYoVariable("jointPrivilegedConfigurationMaxAcceleration", registry);

   private final Map<OneDoFJoint, DoubleYoVariable> yoJointPriviligedConfigurations = new HashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> yoJointPriviligedVelocities = new HashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> yoJointPriviligedAccelerations = new HashMap<>();

   private final DenseMatrix64F privilegedConfigurations;
   private final DenseMatrix64F privilegedVelocities;
   private final DenseMatrix64F privilegedAccelerations;
   private final DenseMatrix64F selectionMatrix;

   private final DenseMatrix64F jointSquaredRangeOfMotions;
   private final DenseMatrix64F positionsAtMidRangeOfMotion;

   private final OneDoFJoint[] oneDoFJoints;
   private final Map<OneDoFJoint, MutableInt> jointIndices;

   private final List<RigidBody> chainBases = new ArrayList<>();
   private final List<RigidBody> chainEndEffectors = new ArrayList<>();

   private final int numberOfDoFs;

   // TODO During toe off, this guy behaves differently and tends to corrupt the CMP. Worst part is that the achieved CMP appears to not show that. (Sylvain)
   public JointPrivilegedConfigurationHandler(OneDoFJoint[] oneDoFJoints, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoints = oneDoFJoints;
      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(oneDoFJoints);

      privilegedConfigurations = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedVelocities = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
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

         String jointName = joint.getName();
         yoJointPriviligedConfigurations.put(joint, new DoubleYoVariable("q_priv_" + jointName, registry));
         yoJointPriviligedVelocities.put(joint, new DoubleYoVariable("qd_priv_" + jointName, registry));
         yoJointPriviligedAccelerations.put(joint, new DoubleYoVariable("qdd_priv_" + jointName, registry));
      }

      configurationGain.set(20.0);
      velocityGain.set(6.0);
      maxVelocity.set(2.0);
      maxAcceleration.set(Double.POSITIVE_INFINITY);
      weight.set(5.0);

      for (int i = 0; i < numberOfDoFs; i++)
         setPrivilegedConfigurationFromOption(PrivilegedConfigurationOption.AT_MID_RANGE, i);

      parentRegistry.addChild(registry);
   }

   /**
    * Clears the information on the kinematic chains. These are used to compute the necessary Jacobians to project into the null space.
    */
   public void reset()
   {
      chainBases.clear();
      chainEndEffectors.clear();
   }

   /**
    * Computes the desired joint velocity to be submitted to the inverse kinematics control core to achieve the desired privileged configuration.
    * Uses a simple proportional controller with saturation limits based on the position error.
    */
   public void computePrivilegedJointVelocities()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         double qd = 2.0 * configurationGain.getDoubleValue() * (privilegedConfigurations.get(i, 0) - joint.getQ()) / jointSquaredRangeOfMotions.get(i, 0);
         qd = MathTools.clipToMinMax(qd, maxVelocity.getDoubleValue());
         privilegedVelocities.set(i, 0, qd);
         yoJointPriviligedVelocities.get(joint).set(qd);
      }
   }

   /**
    * Computes the desired joint accelerations to be submitted to the inverse dynamics control core to achieve the desired privileged configuration.
    * Uses a simple PD controller with saturation limits based on the position error.
    */
   public void computePrivilegedJointAccelerations()
   {
      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         double qdd = 2.0 * configurationGain.getDoubleValue() * (privilegedConfigurations.get(i, 0) - joint.getQ()) / jointSquaredRangeOfMotions.get(i, 0);
         qdd -= velocityGain.getDoubleValue() * joint.getQd();
         qdd = MathTools.clipToMinMax(qdd, maxAcceleration.getDoubleValue());
         privilegedAccelerations.set(i, 0, qdd);
         yoJointPriviligedAccelerations.get(joint).set(qdd);
      }
   }

   public void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      isJointPrivilegedConfigurationEnabled.set(command.isEnabled());

      if (command.hasNewWeight())
      {
         weight.set(command.getWeight());
      }

      if (command.hasNewConfigurationGain())
         configurationGain.set(command.getConfigurationGain());
      if (command.hasNewVelocityGain())
         velocityGain.set(command.getVelocityGain());
      if (command.hasNewMaxVelocity())
         maxVelocity.set(command.getMaxVelocity());
      if (command.hasNewMaxAcceleration())
         maxAcceleration.set(command.getMaxAcceleration());

      if (command.hasNewPrivilegedConfigurationDefaultOption())
      {
         PrivilegedConfigurationOption defaultOption = command.getPrivilegedConfigurationDefaultOption();
         for (int i = 0; i < numberOfDoFs; i++)
            setPrivilegedConfigurationFromOption(defaultOption, i);
      }

      for (int i = 0; i < command.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = command.getJoint(i);
         MutableInt mutableIndex = jointIndices.get(joint);
         if (mutableIndex == null)
        	 continue;

         int jointIndex = mutableIndex.intValue();

         if (command.hasNewPrivilegedConfiguration(i))
         {
            double qPrivileged = command.getPrivilegedConfiguration(i);
            privilegedConfigurations.set(jointIndex, 0, qPrivileged);
            yoJointPriviligedConfigurations.get(oneDoFJoints[jointIndex]).set(qPrivileged);;
         }

         if (command.hasNewPrivilegedConfigurationOption(i))
         {
            PrivilegedConfigurationOption option = command.getPrivilegedConfigurationOption(i);
            setPrivilegedConfigurationFromOption(option, jointIndex);
         }
      }

      for (int chainIndex = 0; chainIndex < command.getNumberOfChains(); chainIndex++)
      {
         chainBases.add(command.getChainBase(chainIndex));
         chainEndEffectors.add(command.getChainEndEffector(chainIndex));
      }
   }

   private void setPrivilegedConfigurationFromOption(PrivilegedConfigurationOption option, int jointIndex)
   {
      double qPrivileged;

      switch (option)
      {
      case AT_CURRENT:
         qPrivileged = oneDoFJoints[jointIndex].getQ();
         break;
      case AT_ZERO:
         qPrivileged = 0.0;
         break;
      case AT_MID_RANGE:
         qPrivileged = positionsAtMidRangeOfMotion.get(jointIndex);
         break;
      default:
         throw new RuntimeException("Cannot handle the PrivilegedConfigurationOption:" + option);
      }

      privilegedConfigurations.set(jointIndex, 0, qPrivileged);
      yoJointPriviligedConfigurations.get(oneDoFJoints[jointIndex]).set(qPrivileged);;
   }

   public boolean isEnabled()
   {
      return isJointPrivilegedConfigurationEnabled.getBooleanValue();
   }

   /**
    * @return matrix of privileged joint velocities to be submitted to the inverse kinematics controller core.
    */
   public DenseMatrix64F getPrivilegedJointVelocities()
   {
      return privilegedVelocities;
   }

   /**
    * @return matrix of privileged joint accelerations to be submitted ot the inverse dynamics controller core.
    */
   public DenseMatrix64F getPrivilegedJointAccelerations()
   {
      return privilegedAccelerations;
   }

   /**
    * @param joint one DoF joint in question
    * @return desired privileged joint acceleration
    */
   public double getPrivilegedJointAcceleration(OneDoFJoint joint)
   {
      return privilegedAccelerations.get(jointIndices.get(joint).intValue(), 0);
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   /**
    * @return one DoF joints to be considered by for the privileged configuration command.
    */
   public OneDoFJoint[] getJoints()
   {
      return oneDoFJoints;
   }

   /**
    * This weight is the respective priority placed on the privileged command in the optimization.
    * @return weight for the privileged command in the optimization.
    */
   public double getWeight()
   {
      return weight.getDoubleValue();
   }

   /**
    * Returns the number of kinematic chains that contain privileged configurations.
    * @return number of kinematic chains
    */
   public int getNumberOfChains()
   {
      return chainBases.size();
   }

   /**
    * Returns the base of the current kinematic chain to compute the Jacobian.
    * @param chainIndex the current chain number
    * @return base body of the current kinematic chain.
    */
   public RigidBody getChainBase(int chainIndex)
   {
      return chainBases.get(chainIndex);
   }

   /**
    * Returns the end effectors of the current kinematic chain to compute the Jacobian.
    * @param chainIndex the current chain number.
    * @return end effector body of the current kinematic chain.
    */
   public RigidBody getChainEndEffector(int chainIndex)
   {
      return chainEndEffectors.get(chainIndex);
   }
}
