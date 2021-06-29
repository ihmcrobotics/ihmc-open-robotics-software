package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class HighLevelControllerState implements State, JointLoadStatusProvider
{
   protected final YoRegistry registry;

   private final JointSettingsHelper jointSettingsHelper;

   private final HighLevelControllerName highLevelControllerName;
   protected final OneDoFJointBasics[] controlledJoints;

   protected YoBoolean requestTransitionToPushRecovery;

   public HighLevelControllerState(HighLevelControllerName stateEnum, HighLevelControllerParameters parameters,
                                   OneDoFJointBasics[] controlledJoints)
   {
      this("", stateEnum, parameters, controlledJoints);
   }

   public HighLevelControllerState(String namePrefix, HighLevelControllerName stateEnum, HighLevelControllerParameters parameters,
                                   OneDoFJointBasics[] controlledJoints)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.highLevelControllerName = stateEnum;
      this.controlledJoints = controlledJoints;
      requestTransitionToPushRecovery = new YoBoolean(stateEnum+"requestTransitionToPushRecovery", registry);
      jointSettingsHelper = new JointSettingsHelper(parameters, controlledJoints, this, stateEnum, registry);
   }

   public HighLevelControllerState(String namePrefix, HighLevelControllerName stateEnum, OneDoFJointBasics[] controlledJoints)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.highLevelControllerName = stateEnum;
      this.controlledJoints = controlledJoints;
      jointSettingsHelper = null;
   }

   public YoRegistry getYoRegistry()
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

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return null;
   }

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

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelControllerName;
   }
}
