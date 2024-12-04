package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

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

   public abstract ControllerCoreOutput getControllerCoreOutput();

   public abstract ControllerCoreCommand getControllerCoreCommandData();

   public abstract WholeBodyControllerCore getControllerCore();
}
