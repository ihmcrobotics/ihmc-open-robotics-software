package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ContactStateManager
{
   private final JumpControllerParameters jumpControllerParameters;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   
   private JumpStateEnum currentState;
   
   public ContactStateManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpControllerParameters, YoVariableRegistry registry)
   {
      this.jumpControllerParameters = jumpControllerParameters;
      this.controllerToolbox = controllerToolbox;
      this.footSwitches = controllerToolbox.getFootSwitches();
   }
   
   public void updateState(JumpStateEnum stateEnum)
   {
      this.currentState = stateEnum;
   }
   
   public void compute()
   {
      switch(currentState)
      {
      case STANDING:
         for(RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
         break;
      case TAKE_OFF:
         for(RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
         break;
      case FLIGHT:
         for(RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFree(robotSide);
         break;
      case LANDING:
         for(RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
         break;
      default:
         throw new RuntimeException("Invalid jump state");
      }
   }
}
