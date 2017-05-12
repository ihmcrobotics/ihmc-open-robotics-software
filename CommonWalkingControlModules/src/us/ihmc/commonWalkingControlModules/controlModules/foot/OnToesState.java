package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class OnToesState extends AbstractFootControlState
{
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialAccelerationCommand zeroAccelerationCommand = new SpatialAccelerationCommand();

   private final FramePoint desiredContactPointPosition = new FramePoint();
   private final YoVariableDoubleProvider maximumToeOffAngleProvider;

   private final ToeOffHelper toeOffHelper;

   private final Twist footTwist = new Twist();

   private double desiredYawToHold = 0.0;
   private double desiredRollToHold = 0.0;
   private final double[] tempYawPitchRoll = new double[3];

   private final FramePoint contactPointPosition = new FramePoint();

   private final YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();

   private final DoubleYoVariable toeOffDesiredPitchAngle, toeOffDesiredPitchVelocity, toeOffDesiredPitchAcceleration;
   private final DoubleYoVariable toeOffCurrentPitchAngle, toeOffCurrentPitchVelocity;

   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();

   private final TwistCalculator twistCalculator;
   private final TranslationReferenceFrame toeOffFrame;

   public OnToesState(FootControlHelper footControlHelper, ToeOffHelper toeOffHelper, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, footControlHelper);

      this.toeOffHelper = toeOffHelper;

      twistCalculator = controllerToolbox.getTwistCalculator();

      String namePrefix = contactableFoot.getName();

      maximumToeOffAngleProvider = new YoVariableDoubleProvider(namePrefix + "MaximumToeOffAngle", registry);
      maximumToeOffAngleProvider.set(footControlHelper.getWalkingControllerParameters().getMaximumToeOffAngle());

      contactableFoot.getToeOffContactPoint(toeOffContactPoint2d);

      toeOffDesiredPitchAngle = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchAngle", registry);
      toeOffDesiredPitchVelocity = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchVelocity", registry);
      toeOffDesiredPitchAcceleration = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchAcceleration", registry);

      toeOffCurrentPitchAngle = new DoubleYoVariable(namePrefix + "ToeOffCurrentPitchAngle", registry);
      toeOffCurrentPitchVelocity = new DoubleYoVariable(namePrefix + "ToeOffCurrentPitchVelocity", registry);

      toeOffDesiredPitchAngle.set(Double.NaN);
      toeOffDesiredPitchVelocity.set(Double.NaN);
      toeOffDesiredPitchAcceleration.set(Double.NaN);

      toeOffCurrentPitchAngle.set(Double.NaN);
      toeOffCurrentPitchVelocity.set(Double.NaN);

      toeOffFrame = new TranslationReferenceFrame(namePrefix + "ToeOffFrame", contactableFoot.getRigidBody().getBodyFixedFrame());

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

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      feedbackControlCommand.setWeightsForSolver(angular, linear);
      zeroAccelerationCommand.setWeights(angular, linear);
   }

   @Override
   public void doSpecificAction()
   {
      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      twistCalculator.getRelativeTwist(rootBody, contactableFoot.getRigidBody(), footTwist);
      footTwist.changeFrame(contactableFoot.getFrameAfterParentJoint());

      toeOffCurrentPitchAngle.set(tempYawPitchRoll[1]);
      toeOffCurrentPitchVelocity.set(footTwist.getAngularPartY());

      desiredPosition.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      computeDesiredsForFreeMotion();

      desiredOrientation.setYawPitchRoll(desiredYawToHold, toeOffDesiredPitchAngle.getDoubleValue(), desiredRollToHold);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setIncludingFrame(contactableFoot.getFrameAfterParentJoint(), 0.0, toeOffDesiredPitchVelocity.getDoubleValue(), 0.0);
      desiredAngularVelocity.changeFrame(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setIncludingFrame(contactableFoot.getFrameAfterParentJoint(), 0.0, toeOffDesiredPitchAcceleration.getDoubleValue(), 0.0);
      desiredAngularAcceleration.changeFrame(worldFrame);

      feedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      feedbackControlCommand.set(desiredContactPointPosition, desiredLinearVelocity, desiredLinearAcceleration);
      zeroAccelerationCommand.setSpatialAccelerationToZero(toeOffFrame);

      setupSingleContactPoint();
   }

   private void computeDesiredsForFreeMotion()
   {
      boolean blockToMaximumPitch = tempYawPitchRoll[1] > maximumToeOffAngleProvider.getValue();

      if (blockToMaximumPitch)
      {
         toeOffDesiredPitchAngle.set(maximumToeOffAngleProvider.getValue());
         toeOffDesiredPitchVelocity.set(0.0);
      }
      else
      {
         toeOffDesiredPitchAngle.set(desiredOrientation.getPitch());
         toeOffDesiredPitchVelocity.set(footTwist.getAngularPartY());
      }

      toeOffDesiredPitchAcceleration.set(0.0);

      ToeSlippingDetector toeSlippingDetector = footControlHelper.getToeSlippingDetector();
      if (toeSlippingDetector != null)
         toeSlippingDetector.update();
   }

   public void getDesireds(FrameOrientation desiredOrientationToPack, FrameVector desiredAngularVelocityToPack)
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

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      toeOffHelper.getToeOffContactPoint(toeOffContactPoint2d, robotSide);

      contactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      contactPointPosition.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      toeOffFrame.updateTranslation(contactPointPosition);
      feedbackControlCommand.setControlFrameFixedInEndEffector(contactPointPosition);

      desiredContactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      desiredContactPointPosition.changeFrame(worldFrame);

      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredYawToHold = desiredOrientation.getYaw();
      desiredRollToHold = desiredOrientation.getRoll();

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

      toeOffHelper.clear();

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
