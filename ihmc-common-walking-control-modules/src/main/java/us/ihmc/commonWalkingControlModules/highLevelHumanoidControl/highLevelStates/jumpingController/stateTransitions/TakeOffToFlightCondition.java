package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class TakeOffToFlightCondition implements StateTransitionCondition
{
   private final SideDependentList<FootSwitchInterface> footSwitches;
   
   public TakeOffToFlightCondition(SideDependentList<FootSwitchInterface> footSwitches)
   {
      this.footSwitches = footSwitches;
   }

   @Override
   public boolean checkCondition()
   {
      boolean transitionCondition = true;
      for (RobotSide robotSide : RobotSide.values)
      {
         transitionCondition &= !footSwitches.get(robotSide).hasFootHitGround();
      }
      return transitionCondition;
   }
}
