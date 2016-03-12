package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;

public class FullyConstrainedState extends AbstractFootControlState
{
   private final FrameVector fullyConstrainedNormalContactVector;
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   private final FramePoint2d cop = new FramePoint2d();
   private final FramePoint2d desiredCoP = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FootSwitchInterface footSwitch;

   public FullyConstrainedState(FootControlHelper footControlHelper, YoVariableRegistry registry)
   {
      super(ConstraintType.FULL, footControlHelper, registry);

      fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);
      spatialAccelerationCommand.setWeight(10.0);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableFoot, fullyConstrainedNormalContactVector);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
   }

   @Override
   public void doSpecificAction()
   {
      footSwitch.computeAndPackCoP(cop);
      momentumBasedController.getDesiredCenterOfPressure(contactableFoot, desiredCoP);
      partialFootholdControlModule.compute(desiredCoP, cop);
      YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
      partialFootholdControlModule.applyShrunkPolygon(contactState);

      footAcceleration.setToZero(contactableFoot.getFrameAfterParentJoint(), rootBody.getBodyFixedFrame(), contactableFoot.getFrameAfterParentJoint());

      footControlHelper.submitTaskspaceConstraint(footAcceleration, spatialAccelerationCommand);
   }

   @Override
   public SpatialAccelerationCommand getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }
}
