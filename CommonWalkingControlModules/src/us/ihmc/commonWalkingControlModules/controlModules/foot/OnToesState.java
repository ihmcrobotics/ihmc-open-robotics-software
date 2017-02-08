package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class OnToesState extends AbstractFootControlState
{
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final FramePoint desiredContactPointPosition = new FramePoint();
   private final YoVariableDoubleProvider maximumToeOffAngleProvider;

   private final Twist footTwist = new Twist();

   private double desiredYawToHold = 0.0;
   private double desiredRollToHold = 0.0;
   private final double[] tempYawPitchRoll = new double[3];

   private final FramePoint contactPointPosition = new FramePoint();

   private final YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();

   private final DenseMatrix64F selectionMatrix = CommonOps.identity(6);

   private final DoubleYoVariable toeOffDesiredPitchAngle, toeOffDesiredPitchVelocity, toeOffDesiredPitchAcceleration;
   private final DoubleYoVariable toeOffCurrentPitchAngle, toeOffCurrentPitchVelocity;

   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();
   private final FramePoint exitCMP = new FramePoint();
   private final FramePoint2d exitCMP2d = new FramePoint2d();
   private final FrameVector2d exitCMPRayDirection2d = new FrameVector2d();
   private final FrameLine2d rayThroughExitCMP = new FrameLine2d();
   private final FrameConvexPolygonWithLineIntersector2d frameConvexPolygonWithRayIntersector2d;

   private final TwistCalculator twistCalculator;

   private final ReferenceFrame soleFrame;
   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();

   public OnToesState(FootControlHelper footControlHelper, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, footControlHelper);

      twistCalculator = momentumBasedController.getTwistCalculator();

      String namePrefix = contactableFoot.getName();
      soleFrame = contactableFoot.getSoleFrame();

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

      orientationFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.HIGH);
      orientationFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      orientationFeedbackControlCommand.setGains(gains.getOrientationGains());

      pointFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.HIGH);
      pointFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      pointFeedbackControlCommand.setGains(gains.getPositionGains());

      feedbackControlCommandList.addCommand(orientationFeedbackControlCommand);
      feedbackControlCommandList.addCommand(pointFeedbackControlCommand);

      for (int i = 0; i < 3; i++)
         MatrixTools.removeRow(selectionMatrix, 3); // Remove linear part
      MatrixTools.removeRow(selectionMatrix, 1); // Remove pitch
      orientationFeedbackControlCommand.setSelectionMatrix(selectionMatrix);

      exitCMP2d.setToNaN(soleFrame);
      exitCMPRayDirection2d.setIncludingFrame(soleFrame, 1.0, 0.0);
      rayThroughExitCMP.setToNaN(soleFrame);
      frameConvexPolygonWithRayIntersector2d = new FrameConvexPolygonWithLineIntersector2d();
   }

   public void setWeight(double weight)
   {
      pointFeedbackControlCommand.setWeightForSolver(weight);
      orientationFeedbackControlCommand.setWeightForSolver(weight);
   }

   public void setWeights(Vector3d angular, Vector3d linear)
   {
      pointFeedbackControlCommand.setWeightsForSolver(linear);
      orientationFeedbackControlCommand.setWeightsForSolver(angular);
   }

   @Override
   public void doSpecificAction()
   {
      feedbackControlCommandList.clear();

      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      twistCalculator.getRelativeTwist(footTwist, rootBody, contactableFoot.getRigidBody());
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

      orientationFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      pointFeedbackControlCommand.set(desiredContactPointPosition, desiredLinearVelocity, desiredLinearAcceleration);

      setupSingleContactPoint();

      feedbackControlCommandList.addCommand(orientationFeedbackControlCommand);
      feedbackControlCommandList.addCommand(pointFeedbackControlCommand);
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

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      footPolygon.clear(soleFrame);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).getPosition2d(toeOffContactPoint2d);
         footPolygon.addVertex(toeOffContactPoint2d);
      }

      footPolygon.update();

      FramePoint2d rayOrigin;

      if (!exitCMP2d.containsNaN() && footPolygon.isPointInside(exitCMP2d))
         rayOrigin = exitCMP2d;
      else
         rayOrigin = footPolygon.getCentroid();

      rayThroughExitCMP.set(rayOrigin, exitCMPRayDirection2d);
      frameConvexPolygonWithRayIntersector2d.intersectWithRay(footPolygon, rayThroughExitCMP);
      toeOffContactPoint2d.set(frameConvexPolygonWithRayIntersector2d.getIntersectionPointOne());

      contactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      contactPointPosition.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      pointFeedbackControlCommand.setBodyFixedPointToControl(contactPointPosition);

      desiredContactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      desiredContactPointPosition.changeFrame(worldFrame);

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

      exitCMP2d.setToNaN();
   }

   public void setExitCMP(FramePoint exitCMP)
   {
      this.exitCMP.setIncludingFrame(exitCMP);
      this.exitCMP.changeFrame(soleFrame);
      exitCMP2d.setByProjectionOntoXYPlaneIncludingFrame(this.exitCMP);
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
      return feedbackControlCommandList;
   }
}
