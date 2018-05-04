package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.FlightState;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionCondition;

public class FlightToLandingCondition implements StateTransitionCondition
{
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final FlightState flightState;

   public FlightToLandingCondition(FlightState flightState, SideDependentList<FootSwitchInterface> footSwitches)
   {
      this.footSwitches = footSwitches;
      this.flightState = flightState;
   }

   @Override
   public boolean checkCondition()
   {
      boolean hasMadeGroundContact = flightState.isDone();
      if (hasMadeGroundContact)
      {
         for(RobotSide robotSide : RobotSide.values)
            hasMadeGroundContact |= footSwitches.get(robotSide).hasFootHitGround();
      }
      return hasMadeGroundContact;
   }
}
