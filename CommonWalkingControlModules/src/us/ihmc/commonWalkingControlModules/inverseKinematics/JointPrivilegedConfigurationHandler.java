package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

/**
 * This class computes the input for the optimization based on the desired privileged configuration commands.
 */
public class JointPrivilegedConfigurationHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable isJointPrivilegedConfigurationEnabled = new BooleanYoVariable("isJointPrivilegedConfigurationEnabled", registry);
   private final DoubleYoVariable defaultConfigurationWeight = new DoubleYoVariable("jointPrivilegedConfigurationDefaultWeight", registry);
   private final DoubleYoVariable defaultConfigurationGain = new DoubleYoVariable("jointPrivilegedConfigurationDefaultGain", registry);
   private final DoubleYoVariable defaultVelocityGain = new DoubleYoVariable("jointPrivilegedVelocityDefaultGain", registry);
   private final DoubleYoVariable defaultMaxVelocity = new DoubleYoVariable("jointPrivilegedConfigurationDefaultMaxVelocity", registry);
   private final DoubleYoVariable defaultMaxAcceleration = new DoubleYoVariable("jointPrivilegedConfigurationDefaultMaxAcceleration", registry);

   private final Map<OneDoFJoint, DoubleYoVariable> yoJointPrivilegedConfigurations = new HashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> yoJointPrivilegedVelocities = new HashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> yoJointPrivilegedAccelerations = new HashMap<>();

   private final DenseMatrix64F privilegedConfigurations;
   private final DenseMatrix64F privilegedVelocities;
   private final DenseMatrix64F privilegedAccelerations;
   private final DenseMatrix64F selectionMatrix;

   private final DenseMatrix64F privilegedConfigurationWeights;
   private final DenseMatrix64F privilegedConfigurationGains;
   private final DenseMatrix64F privilegedVelocityGains;
   private final DenseMatrix64F privilegedMaxVelocities;
   private final DenseMatrix64F privilegedMaxAccelerations;

   private final DenseMatrix64F jointSquaredRangeOfMotions;
   private final DenseMatrix64F positionsAtMidRangeOfMotion;

   private final OneDoFJoint[] oneDoFJoints;
   private final Map<OneDoFJoint, MutableInt> jointIndices;

   private final int numberOfDoFs;

   private final ArrayList<PrivilegedConfigurationCommand> commandList = new ArrayList<>();
   private final ArrayList<OneDoFJoint> jointsWithConfiguration = new ArrayList<>();

   // TODO During toe off, this guy behaves differently and tends to corrupt the CMP. Worst part is that the achieved CMP appears to not show that. (Sylvain)
   public JointPrivilegedConfigurationHandler(OneDoFJoint[] oneDoFJoints, JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
         YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoints = oneDoFJoints;
      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(oneDoFJoints);

      privilegedConfigurations = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedVelocities = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
      selectionMatrix = CommonOps.identity(numberOfDoFs);

      privilegedConfigurationWeights = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      privilegedConfigurationGains = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedVelocityGains = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedMaxVelocities = new DenseMatrix64F(numberOfDoFs, 1);
      privilegedMaxAccelerations = new DenseMatrix64F(numberOfDoFs, 1);

      jointSquaredRangeOfMotions = new DenseMatrix64F(numberOfDoFs, 1);
      positionsAtMidRangeOfMotion = new DenseMatrix64F(numberOfDoFs, 1);
      jointIndices = new HashMap<>(numberOfDoFs);

      // FIXME: at 40.0 the robot sometimes get stuck at the end of transfer when taking one step at a time.
      // The nullspace computed during toe-off is wrong because it does not consider the jacobian nor the proper selection matrix.
      // That nullspace is used to project the privileged joint velocities/accelerations.
      // Set it to 20.0 when getting stuck in transfer. Be careful because 20.0 is not enough to escape singularity at the beginning of the swing.
      defaultConfigurationGain.set(jointPrivilegedConfigurationParameters.getDefaultConfigurationGain());
      defaultVelocityGain.set(jointPrivilegedConfigurationParameters.getDefaultVelocityGain());
      defaultMaxVelocity.set(jointPrivilegedConfigurationParameters.getDefaultMaxVelocity());
      defaultMaxAcceleration.set(jointPrivilegedConfigurationParameters.getDefaultMaxAcceleration());
      defaultConfigurationWeight.set(jointPrivilegedConfigurationParameters.getDefaultWeight());

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
         yoJointPrivilegedConfigurations.put(joint, new DoubleYoVariable("q_priv_" + jointName, registry));
         yoJointPrivilegedVelocities.put(joint, new DoubleYoVariable("qd_priv_" + jointName, registry));
         yoJointPrivilegedAccelerations.put(joint, new DoubleYoVariable("qdd_priv_" + jointName, registry));
      }

      for (int i = 0; i < numberOfDoFs; i++)
         setPrivilegedConfigurationFromOption(PrivilegedConfigurationOption.AT_MID_RANGE, i);

      parentRegistry.addChild(registry);
   }

   /**
    * Computes the desired joint velocity to be submitted to the inverse kinematics control core to achieve the desired privileged configuration.
    * Uses a simple proportional controller with saturation limits based on the position error.
    */
   public void computePrivilegedJointVelocities()
   {
      processPrivilegedConfigurationCommands();

      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         double qd = 2.0 * privilegedConfigurationGains.get(i, 0) * (privilegedConfigurations.get(i, 0) - joint.getQ()) / jointSquaredRangeOfMotions.get(i, 0);
         qd = MathTools.clamp(qd, privilegedMaxVelocities.get(i, 0));
         privilegedVelocities.set(i, 0, qd);
         yoJointPrivilegedVelocities.get(joint).set(qd);
      }
   }

   /**
    * Computes the desired joint accelerations to be submitted to the inverse dynamics control core to achieve the desired privileged configuration.
    * Uses a simple PD controller with saturation limits based on the position error.
    */
   public void computePrivilegedJointAccelerations()
   {
      processPrivilegedConfigurationCommands();

      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         double qdd = 2.0 * privilegedConfigurationGains.get(i, 0) * (privilegedConfigurations.get(i, 0) - joint.getQ()) / jointSquaredRangeOfMotions.get(i, 0);
         qdd -= privilegedVelocityGains.get(i, 0) * joint.getQd();
         qdd = MathTools.clamp(qdd, privilegedMaxAccelerations.get(i, 0));
         privilegedAccelerations.set(i, 0, qdd);
         yoJointPrivilegedAccelerations.get(joint).set(qdd);
      }
   }

   /**
    * Adds a privileged configuration command to be processed later.
    * Note that any weight, configuration gain, velocity gain, max velocity, and max acceleration, will be overwritten by the last one added.
    * Additionally, the last default configuration will be the one used.
    * The same is applicable for different requested privileged configurations.
    * @param command command to add to list
    */
   public void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      commandList.add(command);

      isJointPrivilegedConfigurationEnabled.set(command.isEnabled());

      if (command.hasNewDefaultWeight())
         defaultConfigurationWeight.set(command.getDefaultWeight());
      if (command.hasNewDefaultConfigurationGain())
         defaultConfigurationGain.set(command.getDefaultConfigurationGain());
      if (command.hasNewDefaultVelocityGain())
         defaultVelocityGain.set(command.getDefaultVelocityGain());
      if (command.hasNewDefaultMaxVelocity())
         defaultMaxVelocity.set(command.getDefaultMaxVelocity());
      if (command.hasNewDefaultMaxAcceleration())
         defaultMaxAcceleration.set(command.getDefaultMaxAcceleration());
   }

   private void processPrivilegedConfigurationCommands()
   {
      processDefaultPrivilegedConfigurationOptions();
      processPrivilegedConfigurations();

      commandList.clear();
      jointsWithConfiguration.clear();
   }

   private void processDefaultPrivilegedConfigurationOptions()
   {
      for (int commandIndex = 0; commandIndex < commandList.size(); commandIndex++)
      {
         PrivilegedConfigurationCommand command = commandList.get(commandIndex);

         if (command.hasNewPrivilegedConfigurationDefaultOption())
         {
            PrivilegedConfigurationOption defaultOption = command.getPrivilegedConfigurationDefaultOption();
            for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
               setPrivilegedConfigurationFromOption(defaultOption, jointIndex);
         }
      }
   }

   private void processPrivilegedConfigurations()
   {
      for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
      {
         privilegedConfigurationWeights.set(jointIndex, jointIndex, defaultConfigurationWeight.getDoubleValue());
         privilegedConfigurationGains.set(jointIndex, 0, defaultConfigurationGain.getDoubleValue());
         privilegedVelocityGains.set(jointIndex, 0, defaultVelocityGain.getDoubleValue());
         privilegedMaxVelocities.set(jointIndex, 0, defaultMaxVelocity.getDoubleValue());
         privilegedMaxAccelerations.set(jointIndex, 0, defaultMaxAcceleration.getDoubleValue());
      }

      for (int commandIndex = 0; commandIndex < commandList.size(); commandIndex++)
      {
         PrivilegedConfigurationCommand command = commandList.get(commandIndex);

         for (int jointNumber = 0; jointNumber < command.getNumberOfJoints(); jointNumber++)
         {
            OneDoFJoint joint = command.getJoint(jointNumber);
            MutableInt mutableIndex = jointIndices.get(joint);
            if (mutableIndex == null)
               continue;

            int jointIndex = mutableIndex.intValue();

            if (command.hasNewPrivilegedConfiguration(jointNumber))
            {
               OneDoFJoint configuredJoint = oneDoFJoints[jointIndex];
               double qPrivileged = command.getPrivilegedConfiguration(jointNumber);
               privilegedConfigurations.set(jointIndex, 0, qPrivileged);
               yoJointPrivilegedConfigurations.get(oneDoFJoints[jointIndex]).set(qPrivileged);

               if (!jointsWithConfiguration.contains(configuredJoint))
                  jointsWithConfiguration.add(configuredJoint);
               else
                  PrintTools.warn(this, "Overwriting privileged configuration angle for joint " + configuredJoint.getName() + ".");
            }

            if (command.hasNewPrivilegedConfigurationOption(jointNumber))
            {
               OneDoFJoint configuredJoint = oneDoFJoints[jointIndex];
               PrivilegedConfigurationOption option = command.getPrivilegedConfigurationOption(jointNumber);
               setPrivilegedConfigurationFromOption(option, jointIndex);

               if (!jointsWithConfiguration.contains(configuredJoint))
                  jointsWithConfiguration.add(configuredJoint);
               else
                  PrintTools.warn(this, "Overwriting privileged configuration option for joint " + configuredJoint.getName() + ".");
            }

            processConfigurationWeightsAndGains(command, jointIndex);
         }
      }
   }

   private void processConfigurationWeightsAndGains(PrivilegedConfigurationCommand command, int jointIndex)
   {
      if (command.hasWeight())
         privilegedConfigurationWeights.set(jointIndex, jointIndex, command.getWeight());
      if (command.hasConfigurationGain())
         privilegedConfigurationGains.set(jointIndex, 0, command.getConfigurationGain());
      if (command.hasVelocityGain())
         privilegedVelocityGains.set(jointIndex, 0, command.getVelocityGain());
      if (command.hasMaxVelocity())
         privilegedMaxVelocities.set(jointIndex, 0, command.getMaxVelocity());
      if (command.hasMaxAcceleration())
         privilegedMaxAccelerations.set(jointIndex, 0, command.getMaxAcceleration());
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
      yoJointPrivilegedConfigurations.get(oneDoFJoints[jointIndex]).set(qPrivileged);;
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
    * @return matrix of weights for the privileged command in the optimization.
    */
   public DenseMatrix64F getWeights()
   {
      return privilegedConfigurationWeights;
   }

   /**
    * @param joint one DoF joint in question
    * @return desired privileged weight
    */
   public double getWeight(OneDoFJoint joint)
   {
      int jointIndex = jointIndices.get(joint).intValue();
      return privilegedConfigurationWeights.get(jointIndex, jointIndex);
   }
}
