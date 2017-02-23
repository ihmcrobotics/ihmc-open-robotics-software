package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class FullyConstrainedState extends AbstractFootControlState
{
   private final FrameVector fullyConstrainedNormalContactVector;

   private final InverseDynamicsCommandList inverseDymamicsCommandsList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   private final FramePoint2d cop = new FramePoint2d();
   private final FramePoint2d desiredCoP = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FootSwitchInterface footSwitch;

   public FullyConstrainedState(FootControlHelper footControlHelper, YoVariableRegistry registry)
   {
      super(ConstraintType.FULL, footControlHelper);

      fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);
      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialAccelerationCommand.setPrimaryBase(pelvis);
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
   }

   public void setWeight(double weight)
   {
      spatialAccelerationCommand.setWeight(weight);
   }

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      spatialAccelerationCommand.setWeights(angular, linear);
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
      if (partialFootholdControlModule != null)
      {
         footSwitch.computeAndPackCoP(cop);
         momentumBasedController.getDesiredCenterOfPressure(contactableFoot, desiredCoP);
         partialFootholdControlModule.compute(desiredCoP, cop);
         YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
         boolean contactStateHasChanged = partialFootholdControlModule.applyShrunkPolygon(contactState);
         if (contactStateHasChanged)
            contactState.notifyContactStateHasChanged();
      }

      footAcceleration.setToZero(contactableFoot.getFrameAfterParentJoint(), rootBody.getBodyFixedFrame(), contactableFoot.getFrameAfterParentJoint());

      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(footAcceleration);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDymamicsCommandsList.clear();
      inverseDymamicsCommandsList.addCommand(spatialAccelerationCommand);

      if (attemptToStraightenLegs)
         inverseDymamicsCommandsList.addCommand(straightLegsPrivilegedConfigurationCommand);
      else
         inverseDymamicsCommandsList.addCommand(bentLegsPrivilegedConfigurationCommand);

      return inverseDymamicsCommandsList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }
}
