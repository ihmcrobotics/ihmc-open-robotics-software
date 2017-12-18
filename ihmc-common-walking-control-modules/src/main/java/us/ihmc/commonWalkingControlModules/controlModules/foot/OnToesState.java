package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class OnToesState extends AbstractFootControlState
{
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialAccelerationCommand zeroAccelerationCommand = new SpatialAccelerationCommand();

   private final FramePoint3D desiredContactPointPosition = new FramePoint3D();

   private final ToeOffCalculator toeOffCalculator;

   private final Twist footTwist = new Twist();

   private final FrameQuaternion startOrientation = new FrameQuaternion();
   private final double[] tempYawPitchRoll = new double[3];

   private final FramePoint3D contactPointPosition = new FramePoint3D();

   private final YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();
   private final List<YoContactPoint> contactPointsInContact = new ArrayList<>();

   private final YoBoolean usePointContact;
   private final YoDouble toeOffDesiredPitchAngle, toeOffDesiredPitchVelocity, toeOffDesiredPitchAcceleration;
   private final YoDouble toeOffCurrentPitchAngle, toeOffCurrentPitchVelocity;

   private final FramePoint2D toeOffContactPoint2d = new FramePoint2D();
   private final FrameLineSegment2d toeOffContactLine2d = new FrameLineSegment2d();

   private final TranslationReferenceFrame toeOffFrame;

   private final ReferenceFrame soleZUpFrame;

   public OnToesState(FootControlHelper footControlHelper, ToeOffCalculator toeOffCalculator, YoPIDSE3Gains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, footControlHelper);

      this.toeOffCalculator = toeOffCalculator;

      String namePrefix = contactableFoot.getName();

      contactableFoot.getToeOffContactPoint(toeOffContactPoint2d);
      contactableFoot.getToeOffContactLine(toeOffContactLine2d);

      usePointContact = new YoBoolean(namePrefix + "UsePointContact", registry);

      toeOffDesiredPitchAngle = new YoDouble(namePrefix + "ToeOffDesiredPitchAngle", registry);
      toeOffDesiredPitchVelocity = new YoDouble(namePrefix + "ToeOffDesiredPitchVelocity", registry);
      toeOffDesiredPitchAcceleration = new YoDouble(namePrefix + "ToeOffDesiredPitchAcceleration", registry);

      toeOffCurrentPitchAngle = new YoDouble(namePrefix + "ToeOffCurrentPitchAngle", registry);
      toeOffCurrentPitchVelocity = new YoDouble(namePrefix + "ToeOffCurrentPitchVelocity", registry);

      toeOffDesiredPitchAngle.set(Double.NaN);
      toeOffDesiredPitchVelocity.set(Double.NaN);
      toeOffDesiredPitchAcceleration.set(Double.NaN);

      toeOffCurrentPitchAngle.set(Double.NaN);
      toeOffCurrentPitchVelocity.set(Double.NaN);

      toeOffFrame = new TranslationReferenceFrame(namePrefix + "ToeOffFrame", contactableFoot.getRigidBody().getBodyFixedFrame());

      soleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide);

      feedbackControlCommand.setWeightForSolver(SolverWeightLevels.HIGH);
      feedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      feedbackControlCommand.setPrimaryBase(pelvis);
      feedbackControlCommand.setGains(gains);

      zeroAccelerationCommand.setWeight(SolverWeightLevels.HIGH);
      zeroAccelerationCommand.set(rootBody, contactableFoot.getRigidBody());
      zeroAccelerationCommand.setPrimaryBase(pelvis);

      SelectionMatrix6D feedbackControlSelectionMatrix = new SelectionMatrix6D();
      feedbackControlSelectionMatrix.setSelectionFrames(contactableFoot.getSoleFrame(), worldFrame);
      feedbackControlSelectionMatrix.selectLinearZ(false); // We want to do zero acceleration along z-world.
      feedbackControlSelectionMatrix.selectAngularY(false); // Remove pitch
      feedbackControlCommand.setSelectionMatrix(feedbackControlSelectionMatrix);

      SelectionMatrix6D zeroAccelerationSelectionMatrix = new SelectionMatrix6D();
      zeroAccelerationSelectionMatrix.clearSelection();
      zeroAccelerationSelectionMatrix.setSelectionFrames(worldFrame, worldFrame);
      zeroAccelerationSelectionMatrix.selectLinearZ(true);
      zeroAccelerationCommand.setSelectionMatrix(zeroAccelerationSelectionMatrix);
   }

   public void setWeight(double weight)
   {
      feedbackControlCommand.setWeightForSolver(weight);
      zeroAccelerationCommand.setWeight(weight);
   }

   public void setWeights(Vector3DReadOnly angular, Vector3DReadOnly linear)
   {
      feedbackControlCommand.setWeightsForSolver(angular, linear);
      zeroAccelerationCommand.setWeights(angular, linear);
   }

   public void setUsePointContact(boolean usePointContact)
   {
      this.usePointContact.set(usePointContact);
   }

   @Override
   public void doSpecificAction()
   {
      updateCurrentYoVariables();
      updateToeSlippingDetector();

      desiredOrientation.setIncludingFrame(startOrientation);
      desiredPosition.setIncludingFrame(desiredContactPointPosition);

      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);

      desiredPosition.changeFrame(worldFrame);
      desiredLinearVelocity.setToZero(worldFrame);
      desiredLinearAcceleration.setToZero(worldFrame);

      feedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      feedbackControlCommand.set(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      zeroAccelerationCommand.setSpatialAccelerationToZero(toeOffFrame);

      if (usePointContact.getBooleanValue())
      {
         setupSingleContactPoint();
      }
      else
      {
         setupContactLine();
      }
   }

   private void updateCurrentYoVariables()
   {
      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(soleZUpFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);
      // the current pitch can become NaN when it approaches pi/2
      double currentPitch = tempYawPitchRoll[1];
      if (!Double.isNaN(currentPitch))
      {
         toeOffCurrentPitchAngle.set(tempYawPitchRoll[1]);
      }
      contactableFoot.getFrameAfterParentJoint().getTwistOfFrame(footTwist);
      toeOffCurrentPitchVelocity.set(footTwist.getAngularPartY());
   }

   private void updateToeSlippingDetector()
   {
      ToeSlippingDetector toeSlippingDetector = footControlHelper.getToeSlippingDetector();
      if (toeSlippingDetector != null)
         toeSlippingDetector.update();
   }

   public void getDesireds(FrameQuaternion desiredOrientationToPack, FrameVector3D desiredAngularVelocityToPack)
   {
      desiredOrientationToPack.setIncludingFrame(desiredOrientation);
      desiredAngularVelocityToPack.setIncludingFrame(desiredAngularVelocity);
   }

   private void setupSingleContactPoint()
   {

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).setPosition(toeOffContactPoint2d);

      }
   }

   private final FrameVector3D direction = new FrameVector3D();
   private final FramePoint2D tmpPoint2d = new FramePoint2D();
   private void setupContactLine()
   {
      direction.setToZero(contactableFoot.getSoleFrame());
      direction.setX(1.0);

      contactState.getContactPointsInContact(contactPointsInContact);
      int pointsInContact = contactPointsInContact.size();

      for (int i = 0; i < pointsInContact / 2; i++)
      {
         toeOffContactLine2d.getFirstEndpoint(tmpPoint2d);
         contactPointsInContact.get(i).setPosition(tmpPoint2d);
      }
      for (int i = pointsInContact / 2; i < pointsInContact; i++)
      {
         toeOffContactLine2d.getSecondEndpoint(tmpPoint2d);
         contactPointsInContact.get(i).setPosition(tmpPoint2d);
      }
   }

   private void setControlPointPositionFromContactPoint()
   {
      toeOffCalculator.getToeOffContactPoint(toeOffContactPoint2d, robotSide);

      contactPointPosition.setIncludingFrame(toeOffContactPoint2d, 0.0);
      contactPointPosition.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      feedbackControlCommand.setControlFrameFixedInEndEffector(contactPointPosition);
      toeOffFrame.updateTranslation(contactPointPosition);

      desiredContactPointPosition.setIncludingFrame(toeOffContactPoint2d, 0.0);
      desiredContactPointPosition.changeFrame(worldFrame);
   }

   private void setControlPointPositionFromContactLine()
   {
      toeOffCalculator.getToeOffContactLine(toeOffContactLine2d, robotSide);
      toeOffContactLine2d.midpoint(toeOffContactPoint2d);

      contactPointPosition.setIncludingFrame(toeOffContactPoint2d, 0.0);
      contactPointPosition.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      toeOffFrame.updateTranslation(contactPointPosition);
      feedbackControlCommand.setControlFrameFixedInEndEffector(contactPointPosition);

      desiredContactPointPosition.setIncludingFrame(toeOffContactPoint2d, 0.0);
      desiredContactPointPosition.changeFrame(worldFrame);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      if (usePointContact.getBooleanValue())
         setControlPointPositionFromContactPoint();
      else
         setControlPointPositionFromContactLine();

      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
      contactState.notifyContactStateHasChanged();

      startOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      startOrientation.changeFrame(worldFrame);

      ToeSlippingDetector toeSlippingDetector = footControlHelper.getToeSlippingDetector();
      if (toeSlippingDetector != null)
         toeSlippingDetector.initialize(contactPointPosition);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      toeOffDesiredPitchAngle.set(Double.NaN);
      toeOffDesiredPitchVelocity.set(Double.NaN);
      toeOffDesiredPitchAcceleration.set(Double.NaN);

      toeOffCurrentPitchAngle.set(Double.NaN);
      toeOffCurrentPitchVelocity.set(Double.NaN);

      toeOffCalculator.clear();

      ToeSlippingDetector toeSlippingDetector = footControlHelper.getToeSlippingDetector();
      if (toeSlippingDetector != null)
         toeSlippingDetector.clear();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return zeroAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }
}
