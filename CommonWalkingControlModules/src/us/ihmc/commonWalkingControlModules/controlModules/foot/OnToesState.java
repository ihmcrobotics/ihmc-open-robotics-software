package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class OnToesState extends AbstractFootControlState
{
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();

   private final FramePoint desiredContactPointPosition = new FramePoint();
   private final YoVariableDoubleProvider maximumToeOffAngleProvider;

   private final ToeOffCalculator toeOffCalculator;

   private final Twist footTwist = new Twist();

   private double desiredYawToHold = 0.0;
   private double desiredRollToHold = 0.0;
   private final double[] tempYawPitchRoll = new double[3];

   private final FramePoint contactPointPosition = new FramePoint();

   private final YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();
   private final List<YoContactPoint> contactPointsInContact = new ArrayList<>();

   private final BooleanYoVariable usePointContact;
   private final DoubleYoVariable toeOffDesiredPitchAngle, toeOffDesiredPitchVelocity, toeOffDesiredPitchAcceleration;
   private final DoubleYoVariable toeOffCurrentPitchAngle, toeOffCurrentPitchVelocity;

   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();
   private final FrameLineSegment2d toeOffContactLine2d = new FrameLineSegment2d();

   private final TwistCalculator twistCalculator;

   public OnToesState(FootControlHelper footControlHelper, ToeOffCalculator toeOffCalculator, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, footControlHelper);

      this.toeOffCalculator = toeOffCalculator;

      twistCalculator = controllerToolbox.getTwistCalculator();

      String namePrefix = contactableFoot.getName();

      maximumToeOffAngleProvider = new YoVariableDoubleProvider(namePrefix + "MaximumToeOffAngle", registry);
      maximumToeOffAngleProvider.set(footControlHelper.getWalkingControllerParameters().getMaximumToeOffAngle());

      contactableFoot.getToeOffContactPoint(toeOffContactPoint2d);
      contactableFoot.getToeOffContactLine(toeOffContactLine2d);

      usePointContact = new BooleanYoVariable(namePrefix + "UsePointContact", registry);

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

      feedbackControlCommand.setWeightForSolver(SolverWeightLevels.HIGH);
      feedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      feedbackControlCommand.setPrimaryBase(pelvis);
      feedbackControlCommand.setGains(gains);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.selectAngularY(false); // Remove pitch
      feedbackControlCommand.setSelectionMatrix(selectionMatrix);
   }

   public void setWeight(double weight)
   {
      feedbackControlCommand.setWeightForSolver(weight);
   }

   public void setWeights(Vector3D angular, Vector3D linear)
   {
      feedbackControlCommand.setWeightsForSolver(angular, linear);
   }

   public void setUsePointContact(boolean usePointContact)
   {
      this.usePointContact.set(usePointContact);
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

      if (usePointContact.getBooleanValue())
      {
         setupSingleContactPoint();
         setControlPointPositionFromContactPoint();
      }
      else
      {
         setupContactLine();
         setControlPointPositionFromContactLine();
      }
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

   private final FrameVector direction = new FrameVector();
   private final FramePoint2d tmpPoint2d = new FramePoint2d();
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

      contactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      contactPointPosition.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      feedbackControlCommand.setControlFrameFixedInEndEffector(contactPointPosition);

      desiredContactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      desiredContactPointPosition.changeFrame(worldFrame);
   }

   private void setControlPointPositionFromContactLine()
   {
      toeOffCalculator.getToeOffContactLine(toeOffContactLine2d, robotSide);
      toeOffContactLine2d.midpoint(toeOffContactPoint2d);

      contactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      contactPointPosition.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      feedbackControlCommand.setControlFrameFixedInEndEffector(contactPointPosition);

      desiredContactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
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

      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredYawToHold = desiredOrientation.getYaw();
      desiredRollToHold = desiredOrientation.getRoll();
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
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }
}
