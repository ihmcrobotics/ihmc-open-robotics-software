package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class HoldPositionState extends AbstractFootControlState
{
   // During the partial foothold walking this command was split up to control the roll and pitch of
   // the foot separately using only the ankle joints.
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   private static final boolean VISUALIZE = false;
   private static final double EPSILON = 0.010;

   private final FrameVector holdPositionNormalContactVector = new FrameVector();
   private final FrameVector fullyConstrainedNormalContactVector;

   private final FramePoint2d cop = new FramePoint2d();
   private final FramePoint2d desiredCoP = new FramePoint2d();
   private final FramePoint desiredCoP3d = new FramePoint();
   private final FramePoint desiredCoP3dInDesiredSoleFrame = new FramePoint();
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FramePoint desiredSolePosition = new FramePoint();

   private final FootSwitchInterface footSwitch;
   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();
   private final FrameLineSegment2d closestEdgeToCoP = new FrameLineSegment2d();
   private final FrameVector2d edgeVector2d = new FrameVector2d();
   private final FrameVector edgeVector = new FrameVector();
   private final FrameOrientation desiredOrientationCopy = new FrameOrientation();
   private final AxisAngle desiredAxisAngle = new AxisAngle();
   private final Vector3D desiredRotationVector = new Vector3D();

   private final BooleanYoVariable doSmartHoldPosition;
   private final YoFrameOrientation desiredHoldOrientation;
   private final YoFramePoint desiredHoldPosition;

   private final BooleanYoVariable doHoldFootFlatOrientation;

   private final YoSE3PIDGainsInterface gains;
   private final YoFrameVector yoAngularWeight;
   private final YoFrameVector yoLinearWeight;

   private final Vector3D tempAngularWeightVector = new Vector3D();
   private final Vector3D tempLinearWeightVector = new Vector3D();

   private final FramePose bodyFixedControlledPose = new FramePose();
   private final ReferenceFrame soleFrame;
   private final ReferenceFrame desiredSoleFrame;

   /**
    * Determined whether the state is allowed to change the support polygon based on the exploration
    * done in the PartialFootholdControlModule.
    */
   private BooleanYoVariable doFootholdAdjustments;
   private final static boolean defaultDoFootholdAsjustments = true;

   public HoldPositionState(FootControlHelper footControlHelper, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      this(footControlHelper, footControlHelper.getPartialFootholdControlModule(), gains, registry);
   }

   public HoldPositionState(FootControlHelper footControlHelper, PartialFootholdControlModule partialFootholdControlModule,
         YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.HOLD_POSITION, footControlHelper);
      this.gains = gains;

      soleFrame = contactableFoot.getSoleFrame();
      fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      this.partialFootholdControlModule = partialFootholdControlModule;
      footSwitch = controllerToolbox.getFootSwitches().get(robotSide);
      footPolygon.setIncludingFrameAndUpdate(contactableFoot.getContactPoints2d());
      String namePrefix = contactableFoot.getName();
      desiredHoldOrientation = new YoFrameOrientation(namePrefix + "DesiredHoldOrientation", worldFrame, registry);
      desiredHoldPosition = new YoFramePoint(namePrefix + "DesiredHoldPosition", worldFrame, registry);
      doSmartHoldPosition = new BooleanYoVariable(namePrefix + "DoSmartHoldPosition", registry);

      doHoldFootFlatOrientation = new BooleanYoVariable(namePrefix + "DoHoldFootFlatOrientation", registry);

      doFootholdAdjustments = new BooleanYoVariable(namePrefix + "DoFootholdAdjustments", registry);
      doFootholdAdjustments.set(defaultDoFootholdAsjustments);

      yoAngularWeight = new YoFrameVector(namePrefix + "HoldAngularWeight", null, registry);
      yoLinearWeight = new YoFrameVector(namePrefix + "HoldLinearWeight", null, registry);
      yoAngularWeight.set(1.0, 1.0, 1.0);
      yoAngularWeight.scale(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      yoLinearWeight.set(1.0, 1.0, 1.0);
      yoLinearWeight.scale(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);

      doSmartHoldPosition.set(true);
      doHoldFootFlatOrientation.set(false);

      spatialFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);
      bodyFixedControlledPose.setToZero(soleFrame);
      bodyFixedControlledPose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);

      desiredSoleFrame = new ReferenceFrame(namePrefix + "DesiredSoleFrame", worldFrame)
      {
         private static final long serialVersionUID = -6502583726296859305L;

         private final Quaternion localQuaternion = new Quaternion();
         private final Vector3D localTranslation = new Vector3D();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            desiredHoldOrientation.getQuaternion(localQuaternion);
            desiredSolePosition.get(localTranslation);
            transformToParent.set(localQuaternion, localTranslation);
         }
      };

      if (VISUALIZE)
      {
         YoGraphicsListRegistry YoGraphicsListRegistry = footControlHelper.getHighLevelHumanoidControllerToolbox().getYoGraphicsListRegistry();
         YoGraphicPosition desiredPositionViz = new YoGraphicPosition(namePrefix + "DesiredHoldPosition", desiredHoldPosition, 0.005, YoAppearance.DarkGray(), GraphicType.CROSS);
         YoGraphicsListRegistry.registerYoGraphic("HoldPosition", desiredPositionViz);
         YoGraphicsListRegistry.registerArtifact("HoldPosition", desiredPositionViz.createArtifact());
      }
   }

   public void setWeight(double weight)
   {
      yoAngularWeight.set(1.0, 1.0, 1.0);
      yoAngularWeight.scale(weight);
      yoLinearWeight.set(1.0, 1.0, 1.0);
      yoLinearWeight.scale(weight);
   }

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      yoAngularWeight.set(angular);
      yoLinearWeight.set(linear);
   }

   public void setDoSmartHoldPosition(boolean doSmartHold)
   {
      doSmartHoldPosition.set(doSmartHold);
   }

   public void doFootholdAdjustments(boolean doAdjustments)
   {
      doFootholdAdjustments.set(doAdjustments);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      // Remember the previous contact normal, in case the foot leaves the ground and rotates
      holdPositionNormalContactVector.setIncludingFrame(fullyConstrainedNormalContactVector);
      holdPositionNormalContactVector.changeFrame(worldFrame);
      controllerToolbox.setFootContactStateNormalContactVector(robotSide, holdPositionNormalContactVector);

      desiredSolePosition.setToZero(soleFrame);
      desiredSolePosition.changeFrame(worldFrame);

      desiredOrientation.setToZero(soleFrame);
      desiredOrientation.changeFrame(worldFrame);

      if (doHoldFootFlatOrientation.getBooleanValue())
      {
         desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), 0.0, 0.0);
      }

      desiredHoldOrientation.set(desiredOrientation);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);
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
      controllerToolbox.getDesiredCenterOfPressure(contactableFoot, desiredCoP);

      if (desiredCoP.containsNaN())
      {
         if (!cop.containsNaN())
         {
            desiredCoP.setIncludingFrame(cop);
         }
         else
         {
            desiredCoP.setToZero(soleFrame);
         }
      }

      if (cop.containsNaN())
      {
         cop.setToZero(soleFrame);
      }

      desiredCoP.changeFrame(soleFrame);

      correctDesiredOrientationForSmartHoldPosition();

      if (partialFootholdControlModule != null)
      {
         partialFootholdControlModule.compute(desiredCoP, cop);

         if (doFootholdAdjustments.getBooleanValue())
         {
            YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
            boolean contactStateHasChanged = partialFootholdControlModule.applyShrunkPolygon(contactState);
            if (contactStateHasChanged)
               contactState.notifyContactStateHasChanged();
         }
      }

      // Update the control frame to be at the desired center of pressure
      desiredCoP3d.setXYIncludingFrame(cop); // used to be desiredCoP
      desiredCoP3d.changeFrame(bodyFixedControlledPose.getReferenceFrame());
      bodyFixedControlledPose.setPosition(desiredCoP3d);

      // Compute the desired position
      desiredSoleFrame.update();
      desiredCoP3d.changeFrame(soleFrame);
      desiredCoP3dInDesiredSoleFrame.setIncludingFrame(desiredSoleFrame, desiredCoP3d.getPoint());
      desiredCoP3dInDesiredSoleFrame.changeFrame(worldFrame);
      desiredPosition.setIncludingFrame(desiredCoP3dInDesiredSoleFrame);
      desiredHoldPosition.set(desiredPosition);

      yoAngularWeight.get(tempAngularWeightVector);
      yoLinearWeight.get(tempLinearWeightVector);

      spatialFeedbackControlCommand.set(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      spatialFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      spatialFeedbackControlCommand.setGains(gains);
      spatialFeedbackControlCommand.setWeightsForSolver(tempAngularWeightVector, tempLinearWeightVector);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
   }

   /**
    * Correct the desired orientation such that if the foot landed on the edge, the foot is still able to rotate towards the ground and eventually be in full support.
    */
   private void correctDesiredOrientationForSmartHoldPosition()
   {
      desiredHoldOrientation.getFrameOrientationIncludingFrame(desiredOrientation);

      if (!doSmartHoldPosition.getBooleanValue())
         return;

      if (cop.containsNaN())
         return;

      boolean isCoPOnEdge = !footPolygon.isPointInside(cop, -EPSILON);

      if (!isCoPOnEdge)
         return;

      footPolygon.getClosestEdge(closestEdgeToCoP, cop);
      closestEdgeToCoP.getFrameVector(edgeVector2d);
      edgeVector.setXYIncludingFrame(edgeVector2d);
      edgeVector.normalize();
      desiredOrientationCopy.setIncludingFrame(desiredOrientation);
      desiredOrientationCopy.changeFrame(footPolygon.getReferenceFrame());
      desiredOrientationCopy.getAxisAngle(desiredAxisAngle);
      desiredRotationVector.set(desiredAxisAngle.getX(), desiredAxisAngle.getY(), desiredAxisAngle.getZ());
      desiredRotationVector.scale(desiredAxisAngle.getAngle());

      boolean holdRotationAroundEdge = true;
      double rotationOnEdge = edgeVector.dot(desiredRotationVector);

      if (closestEdgeToCoP.isPointOnLeftSideOfLineSegment(footPolygon.getCentroid()))
      {
         if (rotationOnEdge > 0.0)
            holdRotationAroundEdge = false;
      }
      else
      {
         if (rotationOnEdge < 0.0)
            holdRotationAroundEdge = false;
      }

      if (holdRotationAroundEdge)
         return;

      edgeVector.scale(rotationOnEdge);
      desiredRotationVector.sub(edgeVector.getVector());
      double angle = desiredRotationVector.length();
      desiredRotationVector.scale(1.0 / angle);
      desiredAxisAngle.set(desiredRotationVector, angle);
      desiredOrientationCopy.set(desiredAxisAngle);
      desiredOrientationCopy.changeFrame(worldFrame);
      desiredHoldOrientation.set(desiredOrientationCopy);
      desiredHoldOrientation.getFrameOrientationIncludingFrame(desiredOrientation);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (attemptToStraightenLegs)
         return straightLegsPrivilegedConfigurationCommand;
      else
         return bentLegsPrivilegedConfigurationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }
}
