package us.ihmc.commonWalkingControlModules.inverseKinematics;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.impl.Constants;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class computes the input for the optimization based on the desired privileged configuration
 * commands.
 */
public class JointPrivilegedConfigurationHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean isJointPrivilegedConfigurationEnabled = new YoBoolean("isJointPrivilegedConfigurationEnabled", registry);
   private boolean hasDefaultConfigurationWeightChanged = true;
   private final YoDouble defaultConfigurationWeight = new YoDouble("jointPrivilegedConfigurationDefaultWeight", registry);
   private boolean hasDefaultConfigurationGainChanged = true;
   private final YoDouble defaultConfigurationGain = new YoDouble("jointPrivilegedConfigurationDefaultGain", registry);
   private boolean hasDefaultVelocityGainChanged = true;
   private final YoDouble defaultVelocityGain = new YoDouble("jointPrivilegedVelocityDefaultGain", registry);
   private boolean hasDefaultMaxVelocityChanged = true;
   private final YoDouble defaultMaxVelocity = new YoDouble("jointPrivilegedConfigurationDefaultMaxVelocity", registry);
   private boolean hasDefaultMaxAccelerationChanged = true;
   private final YoDouble defaultMaxAcceleration = new YoDouble("jointPrivilegedConfigurationDefaultMaxAcceleration", registry);

   private final YoDouble[] yoJointPrivilegedConfigurations;
   private final YoDouble[] yoJointPrivilegedVelocities;
   private final YoDouble[] yoJointPrivilegedAccelerations;

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

   private final OneDoFJointBasics[] oneDoFJoints;
   private final TObjectIntHashMap<OneDoFJointBasics> jointIndices;

   private final int numberOfDoFs;

   private final ArrayList<PrivilegedJointSpaceCommand> accelerationCommandList = new ArrayList<>();
   private final ArrayList<PrivilegedJointSpaceCommand> velocityCommandList = new ArrayList<>();
   private final ArrayList<PrivilegedConfigurationCommand> configurationCommandList = new ArrayList<>();
   private final ArrayList<OneDoFJointBasics> jointsWithConfiguration = new ArrayList<>();

   // TODO During toe off, this guy behaves differently and tends to corrupt the CMP. Worst part is that the achieved CMP appears to not show that. (Sylvain)
   public JointPrivilegedConfigurationHandler(OneDoFJointBasics[] oneDoFJoints, JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
                                              YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoints = oneDoFJoints;
      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(oneDoFJoints); // note that this should be equal to oneDoFJoints.length

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
      jointIndices = new TObjectIntHashMap<>(numberOfDoFs, Constants.DEFAULT_LOAD_FACTOR, -1);

      // FIXME: at 40.0 the robot sometimes get stuck at the end of transfer when taking one step at a time.
      // The nullspace computed during toe-off is wrong because it does not consider the jacobian nor the proper selection matrix.
      // That nullspace is used to project the privileged joint velocities/accelerations.
      // Set it to 20.0 when getting stuck in transfer. Be careful because 20.0 is not enough to escape singularity at the beginning of the swing.
      defaultConfigurationGain.set(jointPrivilegedConfigurationParameters.getDefaultConfigurationGain());
      defaultVelocityGain.set(jointPrivilegedConfigurationParameters.getDefaultVelocityGain());
      defaultMaxVelocity.set(jointPrivilegedConfigurationParameters.getDefaultMaxVelocity());
      defaultMaxAcceleration.set(jointPrivilegedConfigurationParameters.getDefaultMaxAcceleration());
      defaultConfigurationWeight.set(jointPrivilegedConfigurationParameters.getDefaultWeight());

      defaultConfigurationGain.addVariableChangedListener(v -> hasDefaultConfigurationGainChanged = true);
      defaultVelocityGain.addVariableChangedListener(v -> hasDefaultVelocityGainChanged = true);
      defaultMaxVelocity.addVariableChangedListener(v -> hasDefaultMaxVelocityChanged = true);
      defaultMaxAcceleration.addVariableChangedListener(v -> hasDefaultMaxAccelerationChanged = true);
      defaultConfigurationWeight.addVariableChangedListener(v -> hasDefaultConfigurationWeightChanged = true);

      yoJointPrivilegedConfigurations = new YoDouble[numberOfDoFs];
      yoJointPrivilegedVelocities = new YoDouble[numberOfDoFs];
      yoJointPrivilegedAccelerations = new YoDouble[numberOfDoFs];

      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];

         jointIndices.put(joint, i);

         double jointLimitUpper = joint.getJointLimitUpper();
         if (Double.isNaN(jointLimitUpper) || Double.isInfinite(jointLimitUpper))
            jointLimitUpper = Math.PI;
         double jointLimitLower = joint.getJointLimitLower();
         if (Double.isNaN(jointLimitLower) || Double.isInfinite(jointLimitLower))
            jointLimitLower = -Math.PI;
         jointSquaredRangeOfMotions.set(i, 0, MathTools.square(jointLimitUpper - jointLimitLower));
         positionsAtMidRangeOfMotion.set(i, 0, 0.5 * (jointLimitUpper + jointLimitLower));

         String jointName = joint.getName();
         yoJointPrivilegedConfigurations[i] = new YoDouble("q_priv_" + jointName, registry);
         yoJointPrivilegedVelocities[i] = new YoDouble("qd_priv_" + jointName, registry);
         yoJointPrivilegedAccelerations[i] = new YoDouble("qdd_priv_" + jointName, registry);
      }

      for (int i = 0; i < numberOfDoFs; i++)
         setPrivilegedConfigurationFromOption(PrivilegedConfigurationOption.AT_MID_RANGE, i);

      parentRegistry.addChild(registry);
   }

   /**
    * Computes the desired joint velocity to be submitted to the inverse kinematics control core to
    * achieve the desired privileged configuration. Uses a simple proportional controller with
    * saturation limits based on the position error.
    */
   public void computePrivilegedJointVelocities()
   {
      processPrivilegedConfigurationCommands();

      for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
      {
         OneDoFJointBasics joint = oneDoFJoints[jointIndex];
         double qd = 2.0 * privilegedConfigurationGains.get(jointIndex, 0) * (privilegedConfigurations.get(jointIndex, 0) - joint.getQ())
               / jointSquaredRangeOfMotions.get(jointIndex, 0);
         qd = MathTools.clamp(qd, privilegedMaxVelocities.get(jointIndex, 0));
         privilegedVelocities.set(jointIndex, 0, qd);
         yoJointPrivilegedVelocities[jointIndex].set(qd);
      }

      processPrivilegedVelocityCommands();
   }

   /**
    * Computes the desired joint accelerations to be submitted to the inverse dynamics control core to
    * achieve the desired privileged configuration. Uses a simple PD controller with saturation limits
    * based on the position error.
    */
   public void computePrivilegedJointAccelerations()
   {
      processPrivilegedConfigurationCommands();

      for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
      {
         OneDoFJointBasics joint = oneDoFJoints[jointIndex];
         double qdd = 2.0 * privilegedConfigurationGains.get(jointIndex, 0) * (privilegedConfigurations.get(jointIndex, 0) - joint.getQ())
               / jointSquaredRangeOfMotions.get(jointIndex, 0);
         qdd -= privilegedVelocityGains.get(jointIndex, 0) * joint.getQd();
         qdd = MathTools.clamp(qdd, privilegedMaxAccelerations.get(jointIndex, 0));
         privilegedAccelerations.set(jointIndex, 0, qdd);
         yoJointPrivilegedAccelerations[jointIndex].set(qdd);
      }

      processPrivilegedAccelerationCommands();
   }

   public void submitPrivilegedAccelerations(PrivilegedJointSpaceCommand command)
   {
      accelerationCommandList.add(command);
      isJointPrivilegedConfigurationEnabled.set(command.isEnabled());
   }

   public void submitPrivilegedVelocities(PrivilegedJointSpaceCommand command)
   {
      velocityCommandList.add(command);
      isJointPrivilegedConfigurationEnabled.set(command.isEnabled());
   }

   /**
    * Adds a privileged configuration command to be processed later. Note that any weight,
    * configuration gain, velocity gain, max velocity, and max acceleration, will be overwritten by the
    * last one added. Additionally, the last default configuration will be the one used. The same is
    * applicable for different requested privileged configurations.
    * 
    * @param command command to add to list
    */
   public void submitPrivilegedConfigurationCommand(PrivilegedConfigurationCommand command)
   {
      configurationCommandList.add(command);

      isJointPrivilegedConfigurationEnabled.set(command.isEnabled());

      OneDoFJointPrivilegedConfigurationParameters defaultParameters = command.getDefaultParameters();

      if (defaultParameters.hasWeight())
         defaultConfigurationWeight.set(defaultParameters.getWeight());
      if (defaultParameters.hasConfigurationGain())
         defaultConfigurationGain.set(defaultParameters.getConfigurationGain());
      if (defaultParameters.hasVelocityGain())
         defaultVelocityGain.set(defaultParameters.getVelocityGain());
      if (defaultParameters.hasMaxVelocity())
         defaultMaxVelocity.set(defaultParameters.getMaxVelocity());
      if (defaultParameters.hasMaxAcceleration())
         defaultMaxAcceleration.set(defaultParameters.getMaxAcceleration());
   }

   private void processPrivilegedAccelerationCommands()
   {
      for (int commandIndex = 0; commandIndex < accelerationCommandList.size(); commandIndex++)
      {
         PrivilegedJointSpaceCommand command = accelerationCommandList.get(commandIndex);

         for (int jointNumber = 0; jointNumber < command.getNumberOfJoints(); jointNumber++)
         {
            OneDoFJointBasics joint = command.getJoint(jointNumber);
            int jointIndex = jointIndices.get(joint);

            if (jointIndex == jointIndices.getNoEntryValue())
               continue;

            OneDoFJointBasics configuredJoint = oneDoFJoints[jointIndex];

            if (command.hasNewPrivilegedCommand(jointNumber))
            {
               double qdd = command.getPrivilegedCommand(jointNumber);
               qdd = MathTools.clamp(qdd, privilegedMaxAccelerations.get(jointIndex, 0));

               privilegedAccelerations.set(jointIndex, 0, qdd);
               yoJointPrivilegedAccelerations[jointIndex].set(qdd);
            }

            if (command.hasWeight(jointNumber))
               privilegedConfigurationWeights.set(jointIndex, jointIndex, command.getWeight(jointNumber));

            if (!jointsWithConfiguration.contains(configuredJoint))
               jointsWithConfiguration.add(configuredJoint);
            else
               LogTools.warn("Overwriting privileged acceleration for joint " + configuredJoint.getName() + ".");
         }
      }

      accelerationCommandList.clear();
   }

   private void processPrivilegedVelocityCommands()
   {
      for (int commandIndex = 0; commandIndex < velocityCommandList.size(); commandIndex++)
      {
         PrivilegedJointSpaceCommand command = velocityCommandList.get(commandIndex);

         for (int jointNumber = 0; jointNumber < command.getNumberOfJoints(); jointNumber++)
         {
            OneDoFJointBasics joint = command.getJoint(jointNumber);
            int jointIndex = jointIndices.get(joint);

            if (jointIndex == jointIndices.getNoEntryValue())
               continue;

            OneDoFJointBasics configuredJoint = oneDoFJoints[jointIndex];

            if (command.hasNewPrivilegedCommand(jointNumber))
            {
               double qd = command.getPrivilegedCommand(jointNumber);
               qd = MathTools.clamp(qd, privilegedMaxVelocities.get(jointIndex, 0));

               privilegedVelocities.set(jointIndex, 0, qd);
               yoJointPrivilegedVelocities[jointIndex].set(qd);
            }

            if (command.hasWeight(jointNumber))
               privilegedConfigurationWeights.set(jointIndex, jointIndex, command.getWeight(jointNumber));

            if (!jointsWithConfiguration.contains(configuredJoint))
               jointsWithConfiguration.add(configuredJoint);
            else
               LogTools.warn("Overwriting privileged velocity for joint " + configuredJoint.getName() + ".");
         }
      }

      velocityCommandList.clear();
   }

   private void processPrivilegedConfigurationCommands()
   {
      processDefaultPrivilegedConfigurationOptions();
      processDefaultParameters();
      processPrivilegedConfigurations();

      configurationCommandList.clear();
      jointsWithConfiguration.clear();
   }

   private void processDefaultPrivilegedConfigurationOptions()
   {
      for (int commandIndex = 0; commandIndex < configurationCommandList.size(); commandIndex++)
      {
         OneDoFJointPrivilegedConfigurationParameters defaultParameters = configurationCommandList.get(commandIndex).getDefaultParameters();

         if (defaultParameters.hasPrivilegedConfigurationOption())
         {
            PrivilegedConfigurationOption defaultOption = defaultParameters.getPrivilegedConfigurationOption();
            for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
               setPrivilegedConfigurationFromOption(defaultOption, jointIndex);
         }
      }
   }

   private void processDefaultParameters()
   {
      if (hasDefaultConfigurationWeightChanged)
      {
         for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
            privilegedConfigurationWeights.set(jointIndex, jointIndex, defaultConfigurationWeight.getDoubleValue());
         hasDefaultConfigurationWeightChanged = false;
      }

      if (hasDefaultConfigurationGainChanged)
      {
         for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
            privilegedConfigurationGains.set(jointIndex, 0, defaultConfigurationGain.getDoubleValue());
         hasDefaultConfigurationGainChanged = false;
      }

      if (hasDefaultVelocityGainChanged)
      {
         for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
            privilegedVelocityGains.set(jointIndex, 0, defaultVelocityGain.getDoubleValue());
         hasDefaultVelocityGainChanged = false;
      }

      if (hasDefaultMaxVelocityChanged)
      {
         for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
            privilegedMaxVelocities.set(jointIndex, 0, defaultMaxVelocity.getDoubleValue());
         hasDefaultMaxVelocityChanged = false;
      }

      if (hasDefaultMaxAccelerationChanged)
      {
         for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
            privilegedMaxAccelerations.set(jointIndex, 0, defaultMaxAcceleration.getDoubleValue());
         hasDefaultMaxAccelerationChanged = false;
      }
   }

   private void processPrivilegedConfigurations()
   {
      for (int commandIndex = 0; commandIndex < configurationCommandList.size(); commandIndex++)
      {
         PrivilegedConfigurationCommand command = configurationCommandList.get(commandIndex);

         for (int jointNumber = 0; jointNumber < command.getNumberOfJoints(); jointNumber++)
         {
            OneDoFJointBasics joint = command.getJoint(jointNumber);
            int jointIndex = jointIndices.get(joint);

            if (jointIndex == jointIndices.getNoEntryValue())
               continue;

            OneDoFJointPrivilegedConfigurationParameters jointSpecificParameters = command.getJointSpecificParameters(jointNumber);

            if (jointSpecificParameters.hasPrivilegedConfiguration())
            {
               OneDoFJointBasics configuredJoint = oneDoFJoints[jointIndex];
               double qPrivileged = jointSpecificParameters.getPrivilegedConfiguration();
               privilegedConfigurations.set(jointIndex, 0, qPrivileged);
               yoJointPrivilegedConfigurations[jointIndex].set(qPrivileged);

               if (!jointsWithConfiguration.contains(configuredJoint))
                  jointsWithConfiguration.add(configuredJoint);
               else
                  LogTools.warn("Overwriting privileged configuration angle for joint " + configuredJoint.getName() + ".");
            }

            if (jointSpecificParameters.hasPrivilegedConfigurationOption())
            {
               OneDoFJointBasics configuredJoint = oneDoFJoints[jointIndex];
               PrivilegedConfigurationOption option = jointSpecificParameters.getPrivilegedConfigurationOption();
               setPrivilegedConfigurationFromOption(option, jointIndex);

               if (!jointsWithConfiguration.contains(configuredJoint))
                  jointsWithConfiguration.add(configuredJoint);
               else
                  LogTools.warn("Overwriting privileged configuration option for joint " + configuredJoint.getName() + ".");
            }

            processConfigurationWeightsAndGains(jointSpecificParameters, jointIndex);
         }
      }
   }

   private void processConfigurationWeightsAndGains(OneDoFJointPrivilegedConfigurationParameters jointSpecificParameters, int jointIndex)
   {
      if (jointSpecificParameters.hasWeight())
         privilegedConfigurationWeights.set(jointIndex, jointIndex, jointSpecificParameters.getWeight());
      if (jointSpecificParameters.hasConfigurationGain())
         privilegedConfigurationGains.set(jointIndex, 0, jointSpecificParameters.getConfigurationGain());
      if (jointSpecificParameters.hasVelocityGain())
         privilegedVelocityGains.set(jointIndex, 0, jointSpecificParameters.getVelocityGain());
      if (jointSpecificParameters.hasMaxVelocity())
         privilegedMaxVelocities.set(jointIndex, 0, jointSpecificParameters.getMaxVelocity());
      if (jointSpecificParameters.hasMaxAcceleration())
         privilegedMaxAccelerations.set(jointIndex, 0, jointSpecificParameters.getMaxAcceleration());
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
      yoJointPrivilegedConfigurations[jointIndex].set(qPrivileged);
   }

   public boolean isEnabled()
   {
      return isJointPrivilegedConfigurationEnabled.getBooleanValue();
   }

   /**
    * @return matrix of privileged joint velocities to be submitted to the inverse kinematics
    *         controller core.
    */
   public DenseMatrix64F getPrivilegedJointVelocities()
   {
      return privilegedVelocities;
   }

   /**
    * @return matrix of privileged joint accelerations to be submitted ot the inverse dynamics
    *         controller core.
    */
   public DenseMatrix64F getPrivilegedJointAccelerations()
   {
      return privilegedAccelerations;
   }

   /**
    * @param joint one DoF joint in question
    * @return desired privileged joint acceleration
    */
   public double getPrivilegedJointAcceleration(OneDoFJointBasics joint)
   {
      return privilegedAccelerations.get(jointIndices.get(joint), 0);
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   /**
    * @return one DoF joints to be considered by for the privileged configuration command.
    */
   public OneDoFJointBasics[] getJoints()
   {
      return oneDoFJoints;
   }

   /**
    * This weight is the respective priority placed on the privileged command in the optimization.
    * 
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
   public double getWeight(OneDoFJointBasics joint)
   {
      int jointIndex = jointIndices.get(joint);
      return privilegedConfigurationWeights.get(jointIndex, jointIndex);
   }
}
