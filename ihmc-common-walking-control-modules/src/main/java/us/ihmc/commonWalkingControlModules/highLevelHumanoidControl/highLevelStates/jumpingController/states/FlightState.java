package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class FlightState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final CentroidalMomentumManager wholeBodyMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final FeetJumpManager feetManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final SideDependentList<FootSwitchInterface> footSwitches;

   public FlightState(WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox,
                      CentroidalMomentumManager centroidalMomentumManager, GravityCompensationManager gravityCompensationManager,
                      SideDependentList<RigidBodyControlManager> handManagers, FeetJumpManager feetManager, Map<String, RigidBodyControlManager> bodyManagerMap)
   {
      super(stateEnum);
      this.controllerToolbox = controllerToolbox;
      this.wholeBodyMomentumManager = centroidalMomentumManager;
      this.gravityCompensationManager = gravityCompensationManager;
      this.feetManager = feetManager;
      this.handManagers = handManagers;
      this.chestManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getChest().getName());
      this.headManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getHead().getName());
      this.footSwitches = controllerToolbox.getFootSwitches();
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doAction()
   {
      wholeBodyMomentumManager.computeMomentumRateOfChangeForFreeFall();
      gravityCompensationManager.setRootJointAccelerationForFreeFall();
      feetManager.makeFeetFullyUnconstrained();

      for (RobotSide side : RobotSide.values)
         handManagers.get(side).compute();
      chestManager.compute();
      headManager.compute();
   }

   @Override
   public void doTransitionIntoAction()
   {
      controllerToolbox.clearContacts();
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(side);
         handManager.holdInTaskspace();
      }
      headManager.holdInJointspace();
      chestManager.holdInJointspace();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
