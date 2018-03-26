package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FlightState extends AbstractJumpingState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final CentroidalMomentumManager wholeBodyMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final SideDependentList<RigidBodyControlManager> footManagers;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;

   private final WholeBodyControlCoreToolbox controlCoreToolbox;

   public FlightState(WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox,
                      CentroidalMomentumManager centroidalMomentumManager, GravityCompensationManager gravityCompensationManager,
                      SideDependentList<RigidBodyControlManager> handManagers, SideDependentList<RigidBodyControlManager> feetManagers,
                      Map<String, RigidBodyControlManager> bodyManagerMap)
   {
      super(stateEnum);
      this.controllerToolbox = controllerToolbox;
      this.controlCoreToolbox = controlCoreToolbox;
      this.wholeBodyMomentumManager = centroidalMomentumManager;
      this.gravityCompensationManager = gravityCompensationManager;
      this.footManagers = feetManagers;
      this.handManagers = handManagers;
      this.chestManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getChest().getName());
      this.headManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getHead().getName());
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doAction()
   {
      updateManagerState();
      wholeBodyMomentumManager.compute();
      gravityCompensationManager.compute();
      for (RobotSide side : RobotSide.values)
      {
         handManagers.get(side).compute();
         footManagers.get(side).compute();
      }
      chestManager.compute();
      headManager.compute();
   }

   private void updateManagerState()
   {
      wholeBodyMomentumManager.updateState(stateEnum);
      gravityCompensationManager.updateState(stateEnum);
   }

   @Override
   public void doTransitionIntoAction()
   {
      controllerToolbox.clearContacts();
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(side);
         handManager.holdInTaskspace();
         RigidBodyControlManager footManger = footManagers.get(side);
         footManger.holdInTaskspace();
      }
      headManager.holdInTaskspace();
      chestManager.holdInTaskspace();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
