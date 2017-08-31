package us.ihmc.atlas.highLevelHumanoidControl;

import us.ihmc.atlas.highLevelHumanoidControl.highLevelStates.NewAtlasStandPrepSetpoints;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.NewMomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.*;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.robotSide.SideDependentList;

public class NewAtlasMomentumControllerFactory extends NewMomentumBasedControllerFactory
{
   public NewAtlasMomentumControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
                                            SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames,
                                            WalkingControllerParameters walkingControllerParameters, ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters,
                                            NewHighLevelControllerStates initialControllerState, NewHighLevelControllerStates fallbackControllerState)
   {
      super(contactableBodiesFactory, footForceSensorNames, footContactSensorNames, wristSensorNames, walkingControllerParameters, capturePointPlannerParameters,
            new NewAtlasStandPrepSetpoints(), initialControllerState, fallbackControllerState);
   }

   @Override
   public NewDoNothingControllerState createDoNothingControllerState(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      return new NewDoNothingControllerState(controllerToolbox);
   }

   @Override
   public NewStandPrepControllerState createStandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepSetpoints standPrepSetpoints)
   {
      return new NewStandPrepControllerState(controllerToolbox, standPrepSetpoints);
   }

   @Override
   public NewStandReadyControllerState createStandReadyControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepSetpoints standPrepSetpoints)
   {
      return new NewStandReadyControllerState(controllerToolbox, standPrepSetpoints);
   }

   @Override
   public NewStandTransitionControllerState createStandTransitionControllerState(NewStandReadyControllerState standReadyControllerState,
                                                                                 NewWalkingControllerState walkingControllerState,
                                                                                 HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      return new NewStandTransitionControllerState(standReadyControllerState, walkingControllerState, controllerToolbox);
   }

   @Override
   public NewWalkingControllerState createWalkingControllerState(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                                                 HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
                                                                 ICPTrajectoryPlannerParameters capturePointPlannerParameters,
                                                                 HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      return new NewWalkingControllerState(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters, capturePointPlannerParameters, controllerToolbox);
   }
}
