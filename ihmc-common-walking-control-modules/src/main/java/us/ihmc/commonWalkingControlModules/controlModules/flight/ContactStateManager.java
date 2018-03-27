package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ContactStateManager implements JumpControlManagerInterface
{
   private final JumpControllerParameters jumpControllerParameters;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private EnumProvider<JumpStateEnum> currentState;

   public ContactStateManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpControllerParameters,
                              YoVariableRegistry registry)
   {
      this.jumpControllerParameters = jumpControllerParameters;
      this.controllerToolbox = controllerToolbox;
      this.footSwitches = controllerToolbox.getFootSwitches();
   }

   @Override
   public void compute()
   {
      switch (currentState.getValue())
      {
      case STANDING:
         for (RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
         break;
      case TAKE_OFF:
         for (RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
         break;
      case FLIGHT:
         for (RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFree(robotSide);
         break;
      case LANDING:
         for (RobotSide robotSide : RobotSide.values)
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
         break;
      default:
         throw new RuntimeException("Invalid jump state");
      }
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public void setStateEnumProvider(EnumProvider<JumpStateEnum> stateEnumProvider)
   {
      this.currentState = stateEnumProvider;
   }
}
