package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class HighLevelControllerState implements State, JointLoadStatusProvider
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JointSettingsHelper jointSettingsHelper;

   private final HighLevelControllerName highLevelControllerName;

   public HighLevelControllerState(HighLevelControllerName stateEnum, HighLevelControllerParameters parameters,
                                   HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.highLevelControllerName = stateEnum;
      OneDoFJoint[] controlledJoints = ScrewTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class);
      jointSettingsHelper = new JointSettingsHelper(parameters, controlledJoints, this, stateEnum, registry);
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

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelControllerName;
   }
}
