package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsBlendedPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class JumpingSwingFootState implements JumpingFootControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotSide robotSide;
   private final ContactableFoot contactableFoot;

   private final FramePoint3D desiredPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D(worldFrame);
   private final FrameQuaternion desiredOrientation = new FrameQuaternion(worldFrame);
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D desiredAngularAcceleration = new FrameVector3D(worldFrame);

   private final JumpingControllerToolbox controllerToolbox;

   private final YoDouble currentTime;
   private final YoBoolean replanTrajectory;
   private final YoBoolean footstepWasAdjusted;

   private final MovingReferenceFrame centerOfMassFrame;

   private final PoseReferenceFrame desiredSoleFrame = new PoseReferenceFrame("desiredSoleFrame", worldFrame);
   private final PoseReferenceFrame desiredControlFrame = new PoseReferenceFrame("desiredControlFrame", desiredSoleFrame);
   private final YoGraphicReferenceFrame desiredFrameGraphic;
   private final YoGraphicReferenceFrame controlFrameGraphic;

   private final FramePose3D desiredPose = new FramePose3D();
   private final Twist desiredTwist = new Twist();
   private final Twist relativeTwist = new Twist();
   private final SpatialAcceleration desiredSpatialAcceleration = new SpatialAcceleration();

   private final JumpingSwingTrajectoryCalculator swingTrajectoryCalculator;
   private final MultipleWaypointsBlendedPoseTrajectoryGenerator blendedSwingTrajectory;

   // for unit testing and debugging:
   private final YoFramePoint3D yoDesiredSolePosition;
   private final YoFrameVector3D yoDesiredSoleLinearVelocity;
   private final YoFrameVector3D yoDesiredSoleLinearAcceleration;

   private final YoFramePoint3D yoDesiredSolePositionInCoMFrame;
   private final YoFrameVector3D yoDesiredSoleLinearVelocityInCoMFrame;
   private final YoFrameVector3D yoDesiredSoleLinearAccelerationInCoMFrame;

   private final YoFrameQuaternion yoDesiredSoleOrientationInCoMFrame;
   private final YoFrameVector3D yoDesiredSoleAngularVelocityInCoMFrame;
   private final YoFrameVector3D yoDesiredSoleAngularAccelerationInCoMFrame;

   private final YoFrameQuaternion yoDesiredSoleOrientation;
   private final YoFrameVector3D yoDesiredSoleAngularVelocity;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private Vector3DReadOnly nominalAngularWeight;
   private Vector3DReadOnly nominalLinearWeight;
   private final PoseReferenceFrame controlFrame;
   private final PIDSE3GainsReadOnly gains;

   private final FramePose3D adjustedFootstepPose = new FramePose3D();

   private final double gravityZ;

   public JumpingSwingFootState(JumpingFootControlHelper footControlHelper,
                                double gravityZ,
                                PIDSE3GainsReadOnly gains,
                                YoRegistry registry)
   {
      contactableFoot = footControlHelper.getContactableFoot();

      controllerToolbox = footControlHelper.getJumpingControllerToolbox();
      centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();

      robotSide = footControlHelper.getRobotSide();
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "FootSwing";
      swingTrajectoryCalculator = footControlHelper.getSwingTrajectoryCalculator();
      blendedSwingTrajectory = new MultipleWaypointsBlendedPoseTrajectoryGenerator(namePrefix, swingTrajectoryCalculator.getSwingTrajectory(), centerOfMassFrame, registry);

      FullHumanoidRobotModel fullRobotModel = footControlHelper.getJumpingControllerToolbox().getFullRobotModel();

      this.gravityZ = gravityZ;
      this.gains = gains;

      RigidBodyBasics foot = contactableFoot.getRigidBody();

      spatialFeedbackControlCommand.set(fullRobotModel.getElevator(), foot);
      spatialFeedbackControlCommand.setPrimaryBase(fullRobotModel.getPelvis());
      ReferenceFrame linearGainsFrame = footControlHelper.getJumpingControllerToolbox().getPelvisZUpFrame();
      spatialFeedbackControlCommand.setGainsFrames(null, linearGainsFrame);
      spatialFeedbackControlCommand.setControlBaseFrame(centerOfMassFrame);

      controlFrame = new PoseReferenceFrame("controlFrame", contactableFoot.getRigidBody().getBodyFixedFrame());
      FramePose3D controlFramePose = new FramePose3D(controlFrame);
      controlFramePose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(controlFramePose);

      ReferenceFrame soleFrame = footControlHelper.getJumpingControllerToolbox().getReferenceFrames().getSoleFrame(robotSide);
      ReferenceFrame footFrame = contactableFoot.getFrameAfterParentJoint();
      RigidBodyTransform soleToControlFrameTransform = new RigidBodyTransform();
      footFrame.getTransformToDesiredFrame(soleToControlFrameTransform, soleFrame);
      desiredControlFrame.setPoseAndUpdate(soleToControlFrameTransform);

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      replanTrajectory = new YoBoolean(namePrefix + "ReplanTrajectory", registry);
      footstepWasAdjusted = new YoBoolean(namePrefix + "FootstepWasAdjusted", registry);

      currentTime = new YoDouble(namePrefix + "CurrentTime", registry);

      yoDesiredSolePosition = new YoFramePoint3D(namePrefix + "DesiredSolePositionInWorld", worldFrame, registry);
      yoDesiredSoleLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInWorld", worldFrame, registry);
      yoDesiredSoleLinearAcceleration = new YoFrameVector3D(namePrefix + "DesiredSoleLinearAccelerationInWorld", worldFrame, registry);
      yoDesiredSolePositionInCoMFrame = new YoFramePoint3D(namePrefix + "DesiredSolePositionInCoMFrame", centerOfMassFrame, registry);
      yoDesiredSoleLinearVelocityInCoMFrame = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInCoMFrame", centerOfMassFrame, registry);
      yoDesiredSoleLinearAccelerationInCoMFrame = new YoFrameVector3D(namePrefix + "DesiredSoleLinearAccelerationInCoMFrame", centerOfMassFrame, registry);
      yoDesiredSoleOrientation = new YoFrameQuaternion(namePrefix + "DesiredSoleOrientationInWorld", worldFrame, registry);
      yoDesiredSoleAngularVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleAngularVelocityInWorld", worldFrame, registry);
      yoDesiredSoleOrientationInCoMFrame = new YoFrameQuaternion(namePrefix + "DesiredSoleOrientationInCoMFrame", centerOfMassFrame, registry);
      yoDesiredSoleAngularVelocityInCoMFrame = new YoFrameVector3D(namePrefix + "DesiredSoleAngularVelocityInCoMFrame", centerOfMassFrame, registry);
      yoDesiredSoleAngularAccelerationInCoMFrame = new YoFrameVector3D(namePrefix + "DesiredSoleAngularAccelerationInCoMFrame", centerOfMassFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition desiredPosition = new YoGraphicPosition(namePrefix + "DesiredPosition", yoDesiredSolePosition, 0.015, YoAppearance.Green());
         yoGraphicsListRegistry.registerYoGraphic("Swing Foot", desiredPosition);

         desiredFrameGraphic = new YoGraphicReferenceFrame(namePrefix, desiredSoleFrame, registry, false, 0.1, YoAppearance.Gray());
         controlFrameGraphic = new YoGraphicReferenceFrame(namePrefix, desiredControlFrame, registry, false, 0.1, YoAppearance.Gray());

         yoGraphicsListRegistry.registerYoGraphic("Swing Foot", desiredFrameGraphic);
         yoGraphicsListRegistry.registerYoGraphic("Swing Foot", controlFrameGraphic);
      }
      else
      {
         desiredFrameGraphic = null;
         controlFrameGraphic = null;
      }
   }

   private void initializeTrajectory()
   {
      swingTrajectoryCalculator.setInitialConditionsToCurrent();

      fillAndInitializeTrajectories(true);
   }

   @Override
   public void onEntry()
   {
      currentTime.set(0.0);

      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
      contactState.notifyContactStateHasChanged();

      initializeTrajectory();
   }

   @Override
   public void onExit()
   {
      currentTime.set(0.0);

      swingTrajectoryCalculator.informDone();

      yoDesiredSolePosition.setToNaN();
      yoDesiredSoleOrientation.setToNaN();
      yoDesiredSoleLinearVelocity.setToNaN();
      yoDesiredSoleLinearAcceleration.setToNaN();
      yoDesiredSoleAngularVelocity.setToNaN();
   }

   @Override
   public void doAction(double timeInState)
   {
      computeAndPackTrajectory(timeInState);

      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation,
                                                       desiredPosition,
                                                       desiredAngularVelocity,
                                                       desiredLinearVelocity,
                                                       desiredAngularAcceleration,
                                                       desiredLinearAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(nominalAngularWeight, nominalLinearWeight);
//      spatialFeedbackControlCommand.setScaleSecondaryTaskJointWeight(true, 0.0);
      spatialFeedbackControlCommand.setGains(gains);
   }

   private void computeAndPackTrajectory(double timeInState)
   {
      currentTime.set(Math.min(timeInState, swingTrajectoryCalculator.getSwingDuration()));

      double time = currentTime.getDoubleValue();

      if (swingTrajectoryCalculator.doOptimizationUpdate()) // haven't finished original planning
      {
         fillAndInitializeTrajectories(false);
      }
      else if (replanTrajectory.getBooleanValue())
      {
         fillAndInitializeBlendedTrajectories();
      }

      replanTrajectory.set(false);

      FixedFramePoseTrajectoryGenerator swingTrajectory = swingTrajectoryCalculator.getSwingTrajectory();
      swingTrajectory.compute(time);
      swingTrajectory.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      swingTrajectory.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      if (timeInState > swingTrajectoryCalculator.getSwingDuration())
      {
         desiredLinearVelocity.setToZero(centerOfMassFrame);
         desiredLinearAcceleration.setToZero(centerOfMassFrame);

         desiredAngularVelocity.setToZero(centerOfMassFrame);
         desiredAngularAcceleration.setToZero(centerOfMassFrame);
      }

      yoDesiredSolePositionInCoMFrame.set(desiredPosition);
      yoDesiredSoleLinearVelocityInCoMFrame.set(desiredLinearVelocity);
      yoDesiredSoleLinearAccelerationInCoMFrame.set(desiredLinearAcceleration);

      yoDesiredSoleOrientationInCoMFrame.set(desiredOrientation);
      yoDesiredSoleAngularVelocityInCoMFrame.set(desiredAngularVelocity);
      yoDesiredSoleAngularAccelerationInCoMFrame.set(desiredAngularAcceleration);

      yoDesiredSolePosition.setMatchingFrame(desiredPosition);
      yoDesiredSoleOrientation.setMatchingFrame(desiredOrientation);

      transformDesiredVelocitiesAndAccelerationsToFrame(worldFrame);
      yoDesiredSoleLinearVelocity.setMatchingFrame(desiredLinearVelocity);
      yoDesiredSoleLinearAcceleration.setMatchingFrame(desiredLinearAcceleration);

      yoDesiredSoleAngularVelocity.setMatchingFrame(desiredAngularVelocity);

      transformDesiredsFromCoMFrameToControlFrame();
   }

   public void setFootstep(FramePose3DReadOnly footstepPoseRelativeToTouchdownCoM, double swingHeight, double swingTime)
   {
      swingTrajectoryCalculator.setFootstep(footstepPoseRelativeToTouchdownCoM, swingHeight, swingTime);

      adjustedFootstepPose.setIncludingFrame(centerOfMassFrame, footstepPoseRelativeToTouchdownCoM);
   }

   public void setAdjustedFootstep(FramePose3DReadOnly adjustedFootstep)
   {
      replanTrajectory.set(true);
      footstepWasAdjusted.set(true);

      adjustedFootstepPose.setIncludingFrame(centerOfMassFrame, adjustedFootstep);
   }

   private void fillAndInitializeTrajectories(boolean initializeOptimizer)
   {
      swingTrajectoryCalculator.initializeTrajectoryWaypoints(initializeOptimizer);

      fillAndInitializeBlendedTrajectories();
   }

   private void fillAndInitializeBlendedTrajectories()
   {
      double swingDuration = swingTrajectoryCalculator.getSwingDuration();
      blendedSwingTrajectory.clear();
         // If there is a swing waypoint at the end of swing we want to preserve its transform to the footstep pose to not blend out
         // any touchdown trajectory when doing step adjustment.
      blendedSwingTrajectory.blendFinalConstraint(adjustedFootstepPose, swingDuration, swingDuration);
      blendedSwingTrajectory.initialize();
   }

   private void transformDesiredsFromCoMFrameToControlFrame()
   {
      desiredPosition.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredSoleFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);

      desiredControlFrame.update();

      if (desiredFrameGraphic != null)
         desiredFrameGraphic.update();
      if (controlFrameGraphic != null)
         controlFrameGraphic.update();

      // change pose
      desiredPose.setToZero(desiredControlFrame);
      desiredPose.changeFrame(worldFrame);
      desiredPosition.setIncludingFrame(desiredPose.getPosition());
      desiredOrientation.setIncludingFrame(desiredPose.getOrientation());

      transformDesiredVelocitiesAndAccelerationsToFrame(desiredControlFrame);
   }

   private void transformDesiredVelocitiesAndAccelerationsToFrame(ReferenceFrame desiredFrame)
   {
      // change twist
      desiredTwist.setIncludingFrame(centerOfMassFrame, worldFrame, centerOfMassFrame, yoDesiredSoleAngularVelocityInCoMFrame, yoDesiredSoleLinearVelocityInCoMFrame);
      desiredTwist.changeFrame(desiredFrame);

      centerOfMassFrame.getTwistRelativeToOther(worldFrame, relativeTwist);
      relativeTwist.changeFrame(worldFrame);

      desiredLinearVelocity.setIncludingFrame(desiredTwist.getLinearPart());
      desiredAngularVelocity.setIncludingFrame(desiredTwist.getAngularPart());
      desiredLinearVelocity.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      desiredLinearVelocity.add(relativeTwist.getLinearPart());
      desiredAngularVelocity.add(relativeTwist.getAngularPart());

      // change spatial acceleration
      desiredSpatialAcceleration.setIncludingFrame(centerOfMassFrame, worldFrame, centerOfMassFrame, yoDesiredSoleAngularAccelerationInCoMFrame, yoDesiredSoleLinearAccelerationInCoMFrame);
      desiredSpatialAcceleration.changeFrame(desiredFrame);
      desiredLinearAcceleration.setIncludingFrame(desiredSpatialAcceleration.getLinearPart());
      desiredAngularAcceleration.setIncludingFrame(desiredSpatialAcceleration.getAngularPart());
      desiredLinearAcceleration.changeFrame(worldFrame);
      desiredAngularAcceleration.changeFrame(worldFrame);
      desiredLinearAcceleration.addZ(-gravityZ);
   }

   private void changeControlFrame(FramePose3DReadOnly controlFramePoseInEndEffector)
   {
      controlFramePoseInEndEffector.checkReferenceFrameMatch(contactableFoot.getRigidBody().getBodyFixedFrame());
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePoseInEndEffector);
      controlFrame.setPoseAndUpdate(controlFramePoseInEndEffector);
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.nominalAngularWeight = angularWeight;
      this.nominalLinearWeight = linearWeight;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }
}
