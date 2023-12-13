package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class HighLevelControllerState implements State, JointLoadStatusProvider, SCS2YoGraphicHolder
{
   protected final YoRegistry registry;

   private final JointSettingsHelper jointSettingsHelper;

   private final HighLevelControllerName highLevelControllerName;
   protected final OneDoFJointBasics[] controlledJoints;
   private HighLevelControllerName previousHighLevelControllerName = null;

   public HighLevelControllerState(HighLevelControllerName stateEnum, HighLevelControllerParameters parameters, OneDoFJointBasics[] controlledJoints)
   {
      this("", stateEnum, parameters, controlledJoints);
   }

   public HighLevelControllerState(String namePrefix,
                                   HighLevelControllerName stateEnum,
                                   HighLevelControllerParameters parameters,
                                   OneDoFJointBasics[] controlledJoints)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.highLevelControllerName = stateEnum;
      this.controlledJoints = controlledJoints;
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
    * Override this if you are using a controller that has contact switching and you would like to
    * switch the joint behavior based on whether a joint is loaded or not.
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

   public void setPreviousHighLevelControllerName(HighLevelControllerName previousHighLevelControllerName)
   {
      this.previousHighLevelControllerName = previousHighLevelControllerName;
   }

   public HighLevelControllerName getPreviousHighLevelControllerName()
   {
      return previousHighLevelControllerName;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
