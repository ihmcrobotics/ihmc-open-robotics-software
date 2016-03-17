package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.ThirdOrderPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class OnToesState extends AbstractFootControlState
{
   private static final boolean USE_TOEOFF_TRAJECTORY = false;
   private static final double MIN_TRAJECTORY_TIME = 0.1;

   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();
   private final JointspaceAccelerationCommand kneeJointCommand = new JointspaceAccelerationCommand();

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

   private final DoubleYoVariable toeOffInitialAngle;
   private final DoubleYoVariable toeOffInitialVelocity;
   private final DoubleYoVariable toeOffFinalAngle;
   private final DoubleYoVariable toeOffTrajectoryTime;
   private final ThirdOrderPolynomialTrajectoryGenerator toeOffTrajectory;

   private final FramePoint2d userDefinedContactPoint = new FramePoint2d();
   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();

   private final TwistCalculator twistCalculator;

   private final OneDoFJoint kneeJoint;

   public OnToesState(FootControlHelper footControlHelper, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, footControlHelper, registry);

      kneeJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE);
      twistCalculator = momentumBasedController.getTwistCalculator();

      String namePrefix = contactableFoot.getName();
      maximumToeOffAngleProvider = new YoVariableDoubleProvider(namePrefix + "MaximumToeOffAngle", registry);
      maximumToeOffAngleProvider.set(footControlHelper.getWalkingControllerParameters().getMaximumToeOffAngle());

      userDefinedContactPoint.setToNaN();
      contactableFoot.getToeOffContactPoint(toeOffContactPoint2d);

      toeOffDesiredPitchAngle = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchAngle", registry);
      toeOffDesiredPitchVelocity = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchVelocity", registry);
      toeOffDesiredPitchAcceleration = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchAcceleration", registry);

      toeOffCurrentPitchAngle = new DoubleYoVariable(namePrefix + "ToeOffCurrentPitchAngle", registry);
      toeOffCurrentPitchVelocity = new DoubleYoVariable(namePrefix + "ToeOffCurrentPitchVelocity", registry);

      toeOffInitialAngle = new DoubleYoVariable(namePrefix + "ToeOffInitialAngle", registry);
      toeOffInitialVelocity = new DoubleYoVariable(namePrefix + "ToeOffInitialVelocity", registry);
      toeOffFinalAngle = new DoubleYoVariable(namePrefix + "ToeOffFinalAngle", registry);
      toeOffFinalAngle.set(footControlHelper.getWalkingControllerParameters().getMaximumToeOffAngle());
      toeOffTrajectoryTime = new DoubleYoVariable(namePrefix + "ToeOffTrajectoryTime", registry);
      toeOffTrajectoryTime.set(Double.NaN);
      DoubleProvider initialPositionProvider = new YoVariableDoubleProvider(toeOffInitialAngle);
      DoubleProvider initialVelocityProvider = new YoVariableDoubleProvider(toeOffInitialVelocity);
      DoubleProvider finalPositionProvider = new YoVariableDoubleProvider(toeOffFinalAngle);
      DoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider(toeOffTrajectoryTime);
      toeOffTrajectory = new ThirdOrderPolynomialTrajectoryGenerator(namePrefix + "ToeOffTrajectory", initialPositionProvider, initialVelocityProvider,
            finalPositionProvider, trajectoryTimeProvider, registry);

      toeOffInitialAngle.set(Double.NaN);
      toeOffInitialVelocity.set(Double.NaN);
      toeOffTrajectoryTime.set(Double.NaN);

      toeOffDesiredPitchAngle.set(Double.NaN);
      toeOffDesiredPitchVelocity.set(Double.NaN);
      toeOffDesiredPitchAcceleration.set(Double.NaN);

      toeOffCurrentPitchAngle.set(Double.NaN);
      toeOffCurrentPitchVelocity.set(Double.NaN);

      kneeJointCommand.setWeight(10.0);
      kneeJointCommand.addJoint(kneeJoint, Double.NaN);
      commandList.addCommand(kneeJointCommand);

      orientationFeedbackControlCommand.setWeightForSolver(10.0);
      orientationFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      orientationFeedbackControlCommand.setGains(gains.getOrientationGains());

      pointFeedbackControlCommand.setWeightForSolver(10.0);
      pointFeedbackControlCommand.set(rootBody, contactableFoot.getRigidBody());
      pointFeedbackControlCommand.setGains(gains.getPositionGains());

      feedbackControlCommandList.addCommand(orientationFeedbackControlCommand);
      feedbackControlCommandList.addCommand(pointFeedbackControlCommand);

      for (int i = 0; i < 3; i++)
         MatrixTools.removeRow(selectionMatrix, 3); // Remove linear part
      MatrixTools.removeRow(selectionMatrix, 1); // Remove pitch
      orientationFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
   }

   public void setWeight(double weight)
   {
      pointFeedbackControlCommand.setWeightForSolver(weight);
      orientationFeedbackControlCommand.setWeightForSolver(weight);
   }

   @Override
   public void doSpecificAction()
   {
      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      twistCalculator.getRelativeTwist(footTwist, rootBody, contactableFoot.getRigidBody());
      footTwist.changeFrame(contactableFoot.getFrameAfterParentJoint());

      toeOffCurrentPitchAngle.set(tempYawPitchRoll[1]);
      toeOffCurrentPitchVelocity.set(footTwist.getAngularPartY());

      desiredPosition.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      if (USE_TOEOFF_TRAJECTORY)
         computeDesiredsForTrajectoryBasedMotion();
      else
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
      
      if (!USE_TOEOFF_TRAJECTORY)
         kneeJointCommand.setOneDoFJointDesiredAcceleration(0, 0.0);

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
   }

   private void computeDesiredsForTrajectoryBasedMotion()
   {
      if (toeOffTrajectoryTime.isNaN() || toeOffTrajectoryTime.getDoubleValue() < MIN_TRAJECTORY_TIME)
      {
         computeDesiredsForFreeMotion();
         return;
      }

      double time = MathTools.clipToMinMax(getTimeInCurrentState(), 0.0, toeOffTrajectoryTime.getDoubleValue());
      toeOffTrajectory.compute(time);

      boolean isToeOffAngleBehindTrajectory = tempYawPitchRoll[1] < toeOffTrajectory.getValue();
      if (isToeOffAngleBehindTrajectory)
         toeOffDesiredPitchAngle.set(toeOffTrajectory.getValue());
      else
         toeOffDesiredPitchAngle.set(desiredOrientation.getPitch());

      boolean isToeOffTooSlow = footTwist.getAngularPartY() < toeOffTrajectory.getVelocity();
      if (isToeOffTooSlow)
         toeOffDesiredPitchVelocity.set(toeOffTrajectory.getVelocity());
      else
         toeOffDesiredPitchVelocity.set(footTwist.getAngularPartY());

      toeOffDesiredPitchAcceleration.set(Math.max(0.0, toeOffTrajectory.getAcceleration()));
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

      if (userDefinedContactPoint.containsNaN())
         contactableFoot.getToeOffContactPoint(toeOffContactPoint2d);
      else
         toeOffContactPoint2d.setIncludingFrame(userDefinedContactPoint);

      contactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      contactPointPosition.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      pointFeedbackControlCommand.setBodyFixedPointToControl(contactPointPosition);

      desiredContactPointPosition.setXYIncludingFrame(toeOffContactPoint2d);
      desiredContactPointPosition.changeFrame(worldFrame);

      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredYawToHold = desiredOrientation.getYaw();
      desiredRollToHold = desiredOrientation.getRoll();

      if (toeOffTrajectoryTime.getDoubleValue() > MIN_TRAJECTORY_TIME) // Returns false if the trajectory time is NaN
      {
         twistCalculator.getRelativeTwist(footTwist, rootBody, contactableFoot.getRigidBody());
         footTwist.changeFrame(contactableFoot.getFrameAfterParentJoint());

         toeOffInitialAngle.set(desiredOrientation.getPitch());
         toeOffInitialVelocity.set(footTwist.getAngularPartY());

         toeOffTrajectory.initialize();
      }
      else
      {
         toeOffTrajectoryTime.set(Double.NaN);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      toeOffInitialAngle.set(Double.NaN);
      toeOffInitialVelocity.set(Double.NaN);
      toeOffTrajectoryTime.set(Double.NaN);

      toeOffDesiredPitchAngle.set(Double.NaN);
      toeOffDesiredPitchVelocity.set(Double.NaN);
      toeOffDesiredPitchAcceleration.set(Double.NaN);

      toeOffCurrentPitchAngle.set(Double.NaN);
      toeOffCurrentPitchVelocity.set(Double.NaN);

      // TODO: kind of a hack
      footControlHelper.resetSelectionMatrix();
   }

   public void setPredictedToeOffDuration(double predictedToeOffDuration)
   {
      toeOffTrajectoryTime.set(predictedToeOffDuration);
   }

   public void setDesiredToeOffContactPoint(FramePoint2d toeOffContactPoint)
   {
      toeOffContactPoint.checkReferenceFrameMatch(contactableFoot.getSoleFrame());
      userDefinedContactPoint.setIncludingFrame(toeOffContactPoint);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommandList;
   }
}
