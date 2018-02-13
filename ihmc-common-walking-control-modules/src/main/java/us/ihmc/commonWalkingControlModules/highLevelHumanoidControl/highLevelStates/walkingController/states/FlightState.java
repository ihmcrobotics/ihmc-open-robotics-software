package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.WholeBodyMomentumManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandInterface;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class FlightState extends AbstractJumpingState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyMomentumManager wholeBodyMomentumManager;
   private final FeetJumpManager feetJumpManager;
   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private final SpatialAccelerationVector zeroGravitationalAcceleration;
   private final RigidBody rootBody;
   
   public FlightState(WholeBodyControlCoreToolbox controlCoreToolbox, HighLevelHumanoidControllerToolbox controllerToolbox, WholeBodyMomentumManager wholeBodyMomentumManager, FeetJumpManager feetJumpManager)
   {
      super(stateEnum);
      this.controllerToolbox = controllerToolbox;
      this.controlCoreToolbox = controlCoreToolbox;
      this.wholeBodyMomentumManager = wholeBodyMomentumManager;
      this.feetJumpManager = feetJumpManager;
      this.rootBody = controlCoreToolbox.getRootBody();
      this.zeroGravitationalAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), new Vector3D(0.0, 0.0, -9.81), new Vector3D(0.0, 0.0, 0.0));
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doAction()
   {
      wholeBodyMomentumManager.update(stateEnum);
      wholeBodyMomentumManager.compute();
      feetJumpManager.compute();
      controlCoreToolbox.getSpatialAccelerationCalculator().setRootAcceleration(zeroGravitationalAcceleration);
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
