package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class FlightToLandingCondition implements StateTransitionCondition
{
   private final SideDependentList<FootSwitchInterface> footSwitches;

   public FlightToLandingCondition(SideDependentList<FootSwitchInterface> footSwitches)
   {
      this.footSwitches = footSwitches;
   }

   @Override
   public boolean checkCondition()
   {
      boolean hasMadeGroundContact = false;
      for(RobotSide robotSide : RobotSide.values)
      {
         hasMadeGroundContact |= footSwitches.get(robotSide).hasFootHitGround();
      }
      return hasMadeGroundContact;
   }
}
