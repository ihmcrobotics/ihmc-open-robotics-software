package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class HighLevelControllerState implements State, JointLoadStatusProvider
{
   protected final YoVariableRegistry registry;

   private final JointSettingsHelper jointSettingsHelper;

   private final HighLevelControllerName highLevelControllerName;
   protected final OneDoFJointBasics[] controlledJoints;

   public HighLevelControllerState(HighLevelControllerName stateEnum, HighLevelControllerParameters parameters,
                                   OneDoFJointBasics[] controlledJoints)
   {
      this("", stateEnum, parameters, controlledJoints);
   }

   public HighLevelControllerState(String namePrefix, HighLevelControllerName stateEnum, HighLevelControllerParameters parameters,
                                   OneDoFJointBasics[] controlledJoints)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.highLevelControllerName = stateEnum;
      this.controlledJoints = controlledJoints;
      jointSettingsHelper = new JointSettingsHelper(parameters, controlledJoints, this, stateEnum, registry);
   }

   public HighLevelControllerState(String namePrefix, HighLevelControllerName stateEnum, OneDoFJointBasics[] controlledJoints)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.highLevelControllerName = stateEnum;
      this.controlledJoints = controlledJoints;
      jointSettingsHelper = null;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   protected JointDesiredOutputList getStateSpecificJointSettings()
   {
      jointSettingsHelper.update();
      return jointSettingsHelper.getStateSpecificJointSettings();
   }

   /**
    * Before calling this make sure to call {@link #getStateSpecificJointSettings()} to update the
    * acceleration integration parameters.
    */
   public JointAccelerationIntegrationCommand getAccelerationIntegrationCommand()
   {
      return jointSettingsHelper.getJointAccelerationIntegrationCommand();
   }

   public abstract JointDesiredOutputListReadOnly getOutputForLowLevelController();

   @Override
   public boolean isDone(double timeInState)
   {
      return false;
   }

   /**
    * Override this if you are using a controller that has contact switching and you would like to switch
    * the joint behavior based on whether a joint is loaded or not.
    */
   @Override
   public boolean isJointLoadBearing(String jointName)
   {
      return false;
   }

   /**
    * Allows the state estimator mode to change based on the controller state. E.g. diagnostic states that have the robot
    * hanging should return the {@link StateEstimatorMode#FROZEN} mode to prevent the state estimator from trying to
    * estimate the position of a floating robot. Modes such as walking can overwrite this method and set the state estimation
    * mode to {@link StateEstimatorMode#NORMAL}.
    * <p>
    * On each high level controller state change a message requesting the state estimator mode returned for the upcoming state
    * will be published.
    * </p>
    * @return the desired state estimator mode of the controller state.
    */
   public StateEstimatorMode getStateEstimatorMode()
   {
      return StateEstimatorMode.FROZEN;
   }

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelControllerName;
   }
}
