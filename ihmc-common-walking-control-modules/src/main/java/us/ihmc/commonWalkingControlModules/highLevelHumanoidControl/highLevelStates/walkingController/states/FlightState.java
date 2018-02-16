package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.foot.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JumpControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;

public class FlightState extends AbstractJumpingState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final CentroidalMomentumManager wholeBodyMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   
   //private final FeetJumpManager feetJumpManager;
   private final WholeBodyControlCoreToolbox controlCoreToolbox;

   
   public FlightState(WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox, JumpControlManagerFactory jumpControlManagerFactory)
   {
      super(stateEnum);
      this.controllerToolbox = controllerToolbox;
      this.controlCoreToolbox = controlCoreToolbox;
      this.wholeBodyMomentumManager = jumpControlManagerFactory.getOrCreateWholeBodyMomentumManager();
      this.gravityCompensationManager = jumpControlManagerFactory.getOrCreateGravityCompensationManager();
      //this.feetJumpManager = feetJumpManager;
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
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
