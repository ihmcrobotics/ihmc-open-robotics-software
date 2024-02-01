package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePoint2D;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
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
   private final YawPitchRoll tempYawPitchRoll = new YawPitchRoll();

   private final FramePoint3D contactPointPosition = new FramePoint3D();

   private final List<YoContactPoint> contactPoints = controllerToolbox.getFootContactState(robotSide).getContactPoints();
   private final List<RateLimitedYoFramePoint2D> contactPointLocations = new ArrayList<>();
   private final DoubleParameter maxContactPointRate;

   private final YoBoolean usePointContact;
   private final YoDouble toeOffDesiredPitchAngle, toeOffDesiredPitchVelocity, toeOffDesiredPitchAcceleration;
   private final YoDouble toeOffCurrentPitchAngle, toeOffCurrentPitchVelocity;

   private final FramePoint2D toeOffContactPoint2d = new FramePoint2D();
   private final FrameLineSegment2D toeOffContactLine2d = new FrameLineSegment2D();

   private final TranslationReferenceFrame toeOffFrame;

   private final ReferenceFrame soleZUpFrame;

   private Vector3DReadOnly angularWeight;
   private Vector3DReadOnly linearWeight;

   private final PIDSE3GainsReadOnly gains;

   public OnToesState(FootControlHelper footControlHelper, ToeOffCalculator toeOffCalculator, PIDSE3GainsReadOnly gains, YoRegistry registry)
   {
      super(footControlHelper);

      this.toeOffCalculator = toeOffCalculator;
      this.gains = gains;

      String namePrefix = contactableFoot.getName();

      contactableFoot.getToeOffContactPoint(toeOffContactPoint2d);
      contactableFoot.getToeOffContactLine(toeOffContactLine2d);

      usePointContact = new YoBoolean(namePrefix + "UsePointContact", registry);

      toeOffDesiredPitchAngle = new YoDouble(namePrefix + "ToeOffDesiredPitchAngle", registry);
      toeOffDesiredPitchVelocity = new YoDouble(namePrefix + "ToeOffDesiredPitchVelocity", registry);
      toeOffDesiredPitchAcceleration = new YoDouble(namePrefix + "ToeOffDesiredPitchAcceleration", registry);

      toeOffCurrentPitchAngle = new YoDouble(namePrefix + "ToeOffCurrentPitchAngle", registry);
      toeOffCurrentPitchVelocity = new YoDouble(namePrefix + "ToeOffCurrentPitchVelocity", registry);

      maxContactPointRate = new DoubleParameter("maxContactPointRate", registry, Double.POSITIVE_INFINITY);
      for (YoContactPoint contactPoint : contactPoints)
      {
         contactPointLocations.add(new RateLimitedYoFramePoint2D(contactPoint.getNamePrefix(),
                                                                 contactPoint.getNameSuffix() + "Limited",
                                                                 registry,
                                                                 maxContactPointRate,
                                                                 controllerToolbox.getControlDT(),
                                                                 contactPoint.getReferenceFrame()));
      }

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

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.angularWeight = angularWeight;
      this.linearWeight = linearWeight;
   }

   public void setUsePointContact(boolean usePointContact)
   {
      this.usePointContact.set(usePointContact);
   }

   public boolean isUsingPointContact()
   {
      return usePointContact.getBooleanValue();
   }

   @Override
   public void doSpecificAction(double timeInState)
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

      feedbackControlCommand.setInverseDynamics(desiredOrientation,
                                                desiredPosition,
                                                desiredAngularVelocity,
                                                desiredLinearVelocity,
                                                desiredAngularAcceleration,
                                                desiredLinearAcceleration);
      zeroAccelerationCommand.setSpatialAccelerationToZero(toeOffFrame);

      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightsForSolver(angularWeight, linearWeight);
      zeroAccelerationCommand.setWeights(angularWeight, linearWeight);

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
      tempYawPitchRoll.set(desiredOrientation);
      // the current pitch can become NaN when it approaches pi/2
      double currentPitch = tempYawPitchRoll.getPitch();
      if (!Double.isNaN(currentPitch))
      {
         toeOffCurrentPitchAngle.set(tempYawPitchRoll.getPitch());
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
         contactPointLocations.get(i).update(toeOffContactPoint2d);
         contactPoints.get(i).set(contactPointLocations.get(i));
      }
   }

   private final FramePoint2D tmpCurrentLocation = new FramePoint2D();

   private void setupContactLine()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint contactPointInContact = contactPoints.get(i);
         RateLimitedYoFramePoint2D contactPointLocation = contactPointLocations.get(i);
         tmpCurrentLocation.setIncludingFrame(contactPointInContact);

         if (toeOffContactLine2d.getFirstEndpoint().distance(tmpCurrentLocation) < toeOffContactLine2d.getSecondEndpoint().distance(tmpCurrentLocation))
         {
            contactPointLocation.update(toeOffContactLine2d.getFirstEndpoint());
         }
         else
         {
            contactPointLocation.update(toeOffContactLine2d.getSecondEndpoint());
         }
         contactPointInContact.set(contactPointLocation);
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
   public void onEntry()
   {
      super.onEntry();

      for (int contactIndex = 0; contactIndex < contactPoints.size(); contactIndex++)
      {
         contactPointLocations.get(contactIndex).setAndUpdate(contactPoints.get(contactIndex));
      }

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
   public void onExit(double timeInState)
   {
      super.onExit(timeInState);

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
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }

   @Override
   public boolean isLoadBearing()
   {
      return true;
   }
}
