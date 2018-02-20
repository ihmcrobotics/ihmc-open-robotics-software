package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.foot.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JumpControlManagerFactory;
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

   private final WholeBodyControlCoreToolbox controlCoreToolbox;

   public FlightState(WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox,
                      CentroidalMomentumManager centroidalMomentumManager, GravityCompensationManager gravityCompensationManager,
                      SideDependentList<RigidBodyControlManager> handManagers, SideDependentList<RigidBodyControlManager> feetManagers)
   {
      super(stateEnum);
      this.controllerToolbox = controllerToolbox;
      this.controlCoreToolbox = controlCoreToolbox;
      this.wholeBodyMomentumManager = centroidalMomentumManager;
      this.gravityCompensationManager = gravityCompensationManager;
      this.footManagers = feetManagers;
      this.handManagers = handManagers;
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
      for(RobotSide side : RobotSide.values)
      {
         handManagers.get(side).compute();
         footManagers.get(side).compute();;
      }

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
      for(RobotSide side : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(side);
         handManager.holdInTaskspace();
         RigidBodyControlManager footManger = footManagers.get(side);
         footManger.holdInJointspace();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
