package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FeetJumpManager implements JumpControlManagerInterface
{
   private final InverseDynamicsCommandList inverseDynamicsCommandList;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final SideDependentList<ContactableFoot> contactableFeet;
   private final SideDependentList<JumpFootControlModule> footControlModules;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   
   private EnumProvider<JumpStateEnum> currentState;

   public FeetJumpManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters jumpControlParameters, YoVariableRegistry registry)
   {
      this.controllerToolbox = controllerToolbox;
      this.footSwitches = controllerToolbox.getFootSwitches();
      this.footContactStates = controllerToolbox.getFootContactStates();
      this.contactableFeet = controllerToolbox.getContactableFeet();
      this.inverseDynamicsCommandList = new InverseDynamicsCommandList();
      this.footControlModules = new SideDependentList<>();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody pelvis = fullRobotModel.getPelvis();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState footContactState = footContactStates.get(robotSide);
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         ContactableFoot contactableFoot = contactableFeet.get(robotSide);
         JumpFootControlModule footControlModule = new JumpFootControlModule(robotSide, footContactState, footSwitch, contactableFoot, elevator, pelvis, jumpControlParameters,
                                                                             registry);
         footControlModules.put(robotSide, footControlModule);
      }
   }

   @Override
   public void setStateEnumProvider(EnumProvider<JumpStateEnum> stateEnumProvider)
   {
      this.currentState = stateEnumProvider;
   }

   @Override
   public void compute()
   {
      switch (currentState.getValue())
      {
      case STANDING:
         for (RobotSide robotSide : RobotSide.values)
         {
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
            footControlModules.get(robotSide).setToDampedCompliantMode();
         }
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
      inverseDynamicsCommandList.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         JumpFootControlModule footControlModule = footControlModules.get(robotSide);
         inverseDynamicsCommandList.addCommand(footControlModule.getInverseDynamicsCommand());
      }
      return inverseDynamicsCommandList;
   }
   
   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }
}
