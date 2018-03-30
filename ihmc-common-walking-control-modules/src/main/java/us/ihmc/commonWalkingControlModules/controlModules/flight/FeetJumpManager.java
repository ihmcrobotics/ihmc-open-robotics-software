package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FeetJumpManager implements JumpControlManagerInterface
{
   private final InverseDynamicsCommandList inverseDynamicsCommandList;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final SideDependentList<ContactableFoot> contactableFeet;
   private final SideDependentList<JumpFootControlModule> footControlModules;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

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
         JumpFootControlModule footControlModule = new JumpFootControlModule(robotSide, footContactState, footSwitch, contactableFoot, elevator, pelvis,
                                                                             jumpControlParameters, registry);
         footControlModules.put(robotSide, footControlModule);
      }
   }

   public void computeForDampedCompliantMode()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).complyAndDamp();
      }
   }

   public void makeFeetFullyConstrained()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.setFootContactStateFullyConstrained(robotSide);
      }
   }
   
   public void makeFeetFullyUnconstrained()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.setFootContactStateFree(robotSide);
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

   public double getGroundReactionForceZ()
   {
      double totalForceZ = 0.0;
      for(RobotSide robotSide : RobotSide.values)
         totalForceZ += footControlModules.get(robotSide).getGroundReactionForceZ();
      return totalForceZ;
   }
}
