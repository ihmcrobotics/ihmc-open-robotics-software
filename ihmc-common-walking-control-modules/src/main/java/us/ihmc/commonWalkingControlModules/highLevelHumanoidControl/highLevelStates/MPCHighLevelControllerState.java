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

public abstract class MPCHighLevelControllerState extends HighLevelControllerState
{
   public MPCHighLevelControllerState(HighLevelControllerName stateEnum, HighLevelControllerParameters parameters, OneDoFJointBasics[] controlledJoints)
   {
      super(stateEnum, parameters, controlledJoints);
   }

   public MPCHighLevelControllerState(String namePrefix,
                                      HighLevelControllerName stateEnum,
                                      HighLevelControllerParameters parameters,
                                      OneDoFJointBasics[] controlledJoints)
   {
      super(namePrefix, stateEnum, parameters, controlledJoints);
   }

   public MPCHighLevelControllerState(String namePrefix, HighLevelControllerName stateEnum, OneDoFJointBasics[] controlledJoints)
   {
      super(namePrefix, stateEnum, controlledJoints);
   }

}