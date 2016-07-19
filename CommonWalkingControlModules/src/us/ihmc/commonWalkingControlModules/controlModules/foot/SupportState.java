package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

/**
 * This foot state will determine which degrees of freedom should be controlled and which should be
 * left unconstrained and holds the position of the ones that are free.
 *
 * E.g. when barely loaded the x, y, and yaw coordinates of the foot should be controlled, while
 * z, roll, and pitch should be unconstrained. If the CoP is close to and edge of the foot polygon
 * rotation towards the outside of the edge should be controlled while other DoFs should be free.
 *
 * This is a combination of the original HoldPositionState and the FullyConstrainedState with smart
 * choice of selection matrices.
 */
public class SupportState extends AbstractFootControlState
{
   private static final double defaultFootLoadThreshold = 0.2;
   private static final int dofs = Twist.SIZE;

   private final YoVariableRegistry registry;

   private final BooleanYoVariable footBarelyLoaded;
   private final BooleanYoVariable copOnEdge;
   private final DoubleYoVariable footLoadThreshold;
   private final boolean[] isDirectionFeedbackControlled = new boolean[dofs];

   private final FootSwitchInterface footSwitch;

   private final ControlFrame controlFrame;
   private final ControlFrame desiredSoleFrame;
   private final YoGraphicReferenceFrame frameViz;

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final DenseMatrix64F accelerationSelectionMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F feedbackSelectionMatrix = new DenseMatrix64F(1, 1);

   private final FramePoint2d cop2d = new FramePoint2d();
   private final FramePoint framePosition = new FramePoint();
   private final FrameOrientation frameOrientation = new FrameOrientation();
   private final FramePose bodyFixedControlledPose = new FramePose();
   private final FramePoint desiredCopPosition = new FramePoint();

   private final FramePoint2d cop = new FramePoint2d();
   private final FramePoint2d desiredCoP = new FramePoint2d();

   public SupportState(FootControlHelper footControlHelper, YoSE3PIDGainsInterface holdPositionGains, YoVariableRegistry parentRegistry)
   {
      super(ConstraintType.FULL, footControlHelper);
      String prefix = footControlHelper.getRobotSide().getLowerCaseName() + "Foot";
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      footSwitch = footControlHelper.getMomentumBasedController().getFootSwitches().get(robotSide);
      controlFrame = new ControlFrame(prefix + "HoldPositionFrame", contactableFoot.getSoleFrame());
      desiredSoleFrame = new ControlFrame(prefix + "DesiredSoleFrame", worldFrame);

      footBarelyLoaded = new BooleanYoVariable(prefix + "BarelyLoaded", registry);
      copOnEdge = new BooleanYoVariable(prefix + "CopOnEdge", registry);
      footLoadThreshold = new DoubleYoVariable(prefix + "LoadThreshold", registry);
      footLoadThreshold.set(defaultFootLoadThreshold);

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());

      spatialFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialFeedbackControlCommand.setGains(holdPositionGains);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);
      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);

      YoGraphicsListRegistry graphicsListRegistry = footControlHelper.getMomentumBasedController().getDynamicGraphicObjectsListRegistry();
      frameViz = new YoGraphicReferenceFrame(controlFrame, registry, 0.2);
      if (graphicsListRegistry != null)
         graphicsListRegistry.registerYoGraphic(prefix + getClass().getSimpleName(), frameViz);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      FrameVector fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableFoot, fullyConstrainedNormalContactVector);

      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;

      footBarelyLoaded.set(false);
      copOnEdge.set(false);
      updateHoldPositionSetpoints();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      footBarelyLoaded.set(false);
      copOnEdge.set(false);
   }

   @Override
   public void doSpecificAction()
   {
      // handle partial foothold detection
      PartialFootholdControlModule partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
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

      // determine foot state
      copOnEdge.set(footControlHelper.isCoPOnEdge());
      footBarelyLoaded.set(footSwitch.computeFootLoadPercentage() < footLoadThreshold.getDoubleValue());
      updateHoldPositionSetpoints();

      // update the control frame
      footSwitch.computeAndPackCoP(cop2d);
      if (cop2d.containsNaN())
         cop2d.setToZero(contactableFoot.getSoleFrame());
      framePosition.setXYIncludingFrame(cop2d);
      frameOrientation.setToZero(contactableFoot.getSoleFrame());
      controlFrame.setTransformToParent(framePosition, frameOrientation);
      controlFrame.update();

      // assemble acceleration command
      footAcceleration.setToZero(controlFrame, rootBody.getBodyFixedFrame(), controlFrame);
      ReferenceFrame bodyFixedFrame = contactableFoot.getRigidBody().getBodyFixedFrame();
      footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
      footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(footAcceleration);

      // assemble feedback command
      bodyFixedControlledPose.setToZero(controlFrame);
      bodyFixedControlledPose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      desiredCopPosition.setXYIncludingFrame(cop2d);
      desiredCopPosition.setIncludingFrame(desiredSoleFrame, desiredCopPosition.getPoint());
      desiredCopPosition.changeFrame(worldFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
      spatialFeedbackControlCommand.set(desiredCopPosition, desiredLinearVelocity, desiredLinearAcceleration);
      spatialFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      // set selection matrices
      accelerationSelectionMatrix.reshape(6, 6);
      CommonOps.setIdentity(accelerationSelectionMatrix);
      feedbackSelectionMatrix.reshape(6, 6);
      CommonOps.setIdentity(feedbackSelectionMatrix);

      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;

      if (footBarelyLoaded.getBooleanValue())
      {
         isDirectionFeedbackControlled[3] = true; // control x position
         isDirectionFeedbackControlled[4] = true; // control y position
         isDirectionFeedbackControlled[2] = true; // control z orientation
      }
      if (copOnEdge.getBooleanValue())
      {
         isDirectionFeedbackControlled[0] = true; // control x orientation
         isDirectionFeedbackControlled[1] = true; // control y orientation
      }

      for (int i = dofs-1; i >= 0; i--)
      {
         if (isDirectionFeedbackControlled[i])
            MatrixTools.removeRow(accelerationSelectionMatrix, i);
         else
            MatrixTools.removeRow(feedbackSelectionMatrix, i);
      }

      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);
      spatialFeedbackControlCommand.setSelectionMatrix(feedbackSelectionMatrix);
      if (accelerationSelectionMatrix.getNumRows() + feedbackSelectionMatrix.getNumRows() != 6)
         throw new RuntimeException("Trying to control too much or too little.");

      // update visualization
      frameViz.setToReferenceFrame(controlFrame);
   }

   private void updateHoldPositionSetpoints()
   {
      if (!footBarelyLoaded.getBooleanValue())
      {
         desiredPosition.setToZero(contactableFoot.getSoleFrame());
         desiredPosition.changeFrame(worldFrame);
      }
      if (!copOnEdge.getBooleanValue())
      {
         desiredOrientation.setToZero(contactableFoot.getSoleFrame());
         desiredOrientation.changeFrame(worldFrame);
      }

      desiredSoleFrame.setTransformToParent(desiredPosition, desiredOrientation);
      desiredSoleFrame.update();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   public void setWeight(double weight)
   {
      spatialAccelerationCommand.setWeight(weight);
      spatialFeedbackControlCommand.setWeightForSolver(weight);
   }

   public void setWeights(Vector3d angular, Vector3d linear)
   {
      spatialAccelerationCommand.setWeights(angular, linear);
      spatialFeedbackControlCommand.setWeightsForSolver(angular, linear);
   }

   private class ControlFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 648565974908207845L;
      private final Quat4d localQuaternion = new Quat4d();
      private final Vector3d localTranslation = new Vector3d();

      public ControlFrame(String frameName, ReferenceFrame parentFrame)
      {
         super(frameName, parentFrame, false, false, false);
      }

      public void setTransformToParent(FramePoint framePosition, FrameOrientation frameOrientation)
      {
         parentFrame.checkReferenceFrameMatch(framePosition.getReferenceFrame());
         parentFrame.checkReferenceFrameMatch(frameOrientation.getReferenceFrame());
         frameOrientation.getQuaternion(localQuaternion);
         framePosition.get(localTranslation);
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(localQuaternion, localTranslation);
      }
   }

}
