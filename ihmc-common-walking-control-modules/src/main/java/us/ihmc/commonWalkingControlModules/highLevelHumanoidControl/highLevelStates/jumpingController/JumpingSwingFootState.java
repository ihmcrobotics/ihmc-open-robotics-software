package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
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

   private final TwoWaypointSwingGenerator swingTrajectoryOptimizer;
   private final MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   private final FrameVector3DReadOnly touchdownVelocity;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();
   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();
   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();

   private final double[] waypointProportions = new double[2];
   private final List<DoubleProvider> defaultWaypointProportions = new ArrayList<>();

   private final YoDouble swingDuration;
   private final YoDouble swingHeight;
   private final YoDouble currentTime;

   private final ReferenceFrame centerOfMassFrame;
   private final MovingReferenceFrame soleFrame;

   private final PoseReferenceFrame desiredSoleFrame = new PoseReferenceFrame("desiredSoleFrame", worldFrame);
   private final PoseReferenceFrame desiredControlFrame = new PoseReferenceFrame("desiredControlFrame", desiredSoleFrame);
   private final FramePose3D desiredPose = new FramePose3D();
   private final Twist desiredTwist = new Twist();
   private final SpatialAcceleration desiredSpatialAcceleration = new SpatialAcceleration();

   private final RigidBodyTransform transformFromToeToAnkle = new RigidBodyTransform();

   private final FramePose3D footstepPose = new FramePose3D();

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   // for unit testing and debugging:
   private final YoFramePoint3D yoDesiredSolePosition;
   private final YoFrameVector3D yoDesiredSoleLinearVelocity;
   private final YoFramePoint3D yoDesiredSolePositionInCoMFrame;
   private final YoFrameVector3D yoDesiredSoleLinearVelocityInCoMFrame;
   private final YoFrameQuaternion yoDesiredSoleOrientation;
   private final YoFrameVector3D yoDesiredSoleAngularVelocity;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private Vector3DReadOnly nominalAngularWeight;
   private Vector3DReadOnly nominalLinearWeight;
   private final PoseReferenceFrame controlFrame;
   private final PIDSE3GainsReadOnly gains;

   private final double gravityZ;

   public JumpingSwingFootState(JumpingFootControlHelper footControlHelper,
                                FrameVector3DReadOnly touchdownVelocity,
                                double gravityZ,
                                PIDSE3GainsReadOnly gains,
                                YoRegistry registry)
   {
      contactableFoot = footControlHelper.getContactableFoot();

      controllerToolbox = footControlHelper.getJumpingControllerToolbox();
      centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();

      robotSide = footControlHelper.getRobotSide();
      FullHumanoidRobotModel fullRobotModel = footControlHelper.getJumpingControllerToolbox().getFullRobotModel();

      this.gravityZ = gravityZ;
      this.gains = gains;

      RigidBodyBasics foot = contactableFoot.getRigidBody();
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "FootSwing";

      spatialFeedbackControlCommand.set(fullRobotModel.getElevator(), foot);
      spatialFeedbackControlCommand.setPrimaryBase(fullRobotModel.getPelvis());
      ReferenceFrame linearGainsFrame = footControlHelper.getJumpingControllerToolbox().getPelvisZUpFrame();
      spatialFeedbackControlCommand.setGainsFrames(null, linearGainsFrame);
      spatialFeedbackControlCommand.setControlBaseFrame(centerOfMassFrame);

      controlFrame = new PoseReferenceFrame("controlFrame", contactableFoot.getRigidBody().getBodyFixedFrame());
      FramePose3D controlFramePose = new FramePose3D(controlFrame);
      controlFramePose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(controlFramePose);

      this.touchdownVelocity = new FrameVector3D(centerOfMassFrame);//touchdownVelocity;

      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();

      int numberWaypoints = 2;
      double[] defaultWaypointProportions = swingTrajectoryParameters.getSwingWaypointProportions();

      for (int i = 0; i < numberWaypoints; i++)
      {
         DoubleParameter waypointProportion = new DoubleParameter(namePrefix + "WaypointProportion" + i, registry, defaultWaypointProportions[i]);
         this.defaultWaypointProportions.add(waypointProportion);
      }

      soleFrame = footControlHelper.getJumpingControllerToolbox().getReferenceFrames().getSoleFrame(robotSide);
      ReferenceFrame footFrame = contactableFoot.getFrameAfterParentJoint();
      RigidBodyTransform soleToControlFrameTransform = new RigidBodyTransform();
      footFrame.getTransformToDesiredFrame(soleToControlFrameTransform, soleFrame);
      desiredControlFrame.setPoseAndUpdate(soleToControlFrameTransform);

      double maxSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot();
      double minSwingHeightFromStanceFoot = 0.0;
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix,
                                                               minSwingHeightFromStanceFoot,
                                                               maxSwingHeightFromStanceFoot,
                                                               defaultSwingHeightFromStanceFoot,
                                                               centerOfMassFrame,
                                                               registry,
                                                               yoGraphicsListRegistry);

      swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix, Footstep.maxNumberOfSwingWaypoints + 2, registry);

      swingDuration = new YoDouble(namePrefix + "Duration", registry);
      swingHeight = new YoDouble(namePrefix + "Height", registry);

      currentTime = new YoDouble(namePrefix + "CurrentTime", registry);

      footstepPose.setToNaN();

      yoDesiredSolePosition = new YoFramePoint3D(namePrefix + "DesiredSolePositionInWorld", worldFrame, registry);
      yoDesiredSoleOrientation = new YoFrameQuaternion(namePrefix + "DesiredSoleOrientationInWorld", worldFrame, registry);
      yoDesiredSoleLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInWorld", worldFrame, registry);
      yoDesiredSolePositionInCoMFrame = new YoFramePoint3D(namePrefix + "DesiredSolePositionInCoMFrame", centerOfMassFrame, registry);
      yoDesiredSoleLinearVelocityInCoMFrame = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInCoMFrame", centerOfMassFrame, registry);
      yoDesiredSoleAngularVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleAngularVelocityInWorld", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition desiredPosition = new YoGraphicPosition(namePrefix + "DesiredPosition", yoDesiredSolePosition, 0.015, YoAppearance.Green());
         yoGraphicsListRegistry.registerYoGraphic("Swing Foot", desiredPosition);
      }
   }

   private void initializeTrajectory()
   {
      initialPosition.setToZero(soleFrame);
      initialOrientation.setToZero(soleFrame);
      initialPosition.changeFrame(centerOfMassFrame);
      initialOrientation.changeFrame(centerOfMassFrame);
      initialAngularVelocity.setToZero(centerOfMassFrame);
      initialLinearVelocity.setToZero(centerOfMassFrame);

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

      swingTrajectoryOptimizer.informDone();

      yoDesiredSolePosition.setToNaN();
      yoDesiredSoleOrientation.setToNaN();
      yoDesiredSoleLinearVelocity.setToNaN();
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
      currentTime.set(Math.min(timeInState, swingDuration.getDoubleValue()));

      double time = currentTime.getDoubleValue();

      if (swingTrajectoryOptimizer.doOptimizationUpdate()) // haven't finished original planning
      {
         fillAndInitializeTrajectories(false);
      }

      swingTrajectory.compute(time);
      swingTrajectory.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      swingTrajectory.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      if (timeInState > swingDuration.getDoubleValue())
      {
         desiredLinearVelocity.setToZero(centerOfMassFrame);
         desiredLinearAcceleration.setToZero(worldFrame);
         desiredLinearAcceleration.setZ(-controllerToolbox.getGravityZ());

         desiredAngularVelocity.setToZero(worldFrame);
         desiredAngularAcceleration.setToZero(worldFrame);
      }

      yoDesiredSolePosition.setMatchingFrame(desiredPosition);
      yoDesiredSoleOrientation.setMatchingFrame(desiredOrientation);
      yoDesiredSoleLinearVelocity.setMatchingFrame(desiredLinearVelocity);
      yoDesiredSoleAngularVelocity.setMatchingFrame(desiredAngularVelocity);
      yoDesiredSolePositionInCoMFrame.setMatchingFrame(desiredPosition);
      yoDesiredSoleLinearVelocityInCoMFrame.setMatchingFrame(desiredLinearVelocity);

      transformDesiredsFromCoMFrameToControlFrame();
   }

   public void setFootstep(FramePose3DReadOnly footstepPoseRelativeToTouchdownCoM, double swingHeight, double swingTime)
   {
      this.footstepPose.setIncludingFrame(centerOfMassFrame, footstepPoseRelativeToTouchdownCoM);
      this.swingHeight.set(swingHeight);

      List<DoubleProvider> waypointProportions = defaultWaypointProportions;
      this.waypointProportions[0] = waypointProportions.get(0).getValue();
      this.waypointProportions[1] = waypointProportions.get(1).getValue();

      swingDuration.set(swingTime);
   }

   private void fillAndInitializeTrajectories(boolean initializeOptimizer)
   {
      finalPosition.setIncludingFrame(footstepPose.getPosition());
      finalOrientation.setIncludingFrame(footstepPose.getOrientation());

      finalLinearVelocity.setIncludingFrame(centerOfMassFrame, touchdownVelocity);
      finalAngularVelocity.setToZero(worldFrame);
      finalAngularVelocity.changeFrame(centerOfMassFrame);

      // append current pose as initial trajectory point
      swingTrajectory.clear(centerOfMassFrame);

      swingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);
      swingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, initialAngularVelocity);

      // TODO: initialize optimizer somewhere else
      if (initializeOptimizer)
         initializeOptimizer();

      for (int i = 0; i < swingTrajectoryOptimizer.getNumberOfWaypoints(); i++)
      {
         swingTrajectoryOptimizer.getWaypointData(i, tempPositionTrajectoryPoint);
         swingTrajectory.appendPositionWaypoint(tempPositionTrajectoryPoint);
      }

      // append footstep pose if not provided in the waypoints
      swingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);
      swingTrajectory.appendOrientationWaypoint(swingDuration.getDoubleValue(), finalOrientation, finalAngularVelocity);

      swingTrajectory.initialize();
   }

   private void initializeOptimizer()
   {
      swingTrajectoryOptimizer.setInitialConditions(initialPosition, initialLinearVelocity);
      swingTrajectoryOptimizer.setFinalConditions(finalPosition, finalLinearVelocity);
      swingTrajectoryOptimizer.setStepTime(swingDuration.getDoubleValue());
      swingTrajectoryOptimizer.setTrajectoryType(TrajectoryType.DEFAULT, null);
      swingTrajectoryOptimizer.setSwingHeight(swingHeight.getDoubleValue());
      swingTrajectoryOptimizer.setWaypointProportions(waypointProportions);
      swingTrajectoryOptimizer.initialize();
   }

   private void transformDesiredsFromCoMFrameToControlFrame()
   {
      desiredPosition.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredSoleFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);

      // change pose
      desiredPose.setToZero(desiredControlFrame);
      desiredPose.changeFrame(worldFrame);
      desiredPosition.setIncludingFrame(desiredPose.getPosition());
      desiredOrientation.setIncludingFrame(desiredPose.getOrientation());

      // change twist
      desiredLinearVelocity.changeFrame(centerOfMassFrame);
      desiredAngularVelocity.changeFrame(centerOfMassFrame);
      desiredTwist.setIncludingFrame(centerOfMassFrame, worldFrame, centerOfMassFrame, desiredAngularVelocity, desiredLinearVelocity);
      desiredTwist.changeFrame(desiredControlFrame);
      desiredLinearVelocity.setIncludingFrame(desiredTwist.getLinearPart());
      desiredAngularVelocity.setIncludingFrame(desiredTwist.getAngularPart());
      desiredLinearVelocity.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);

      // change spatial acceleration
      desiredLinearAcceleration.changeFrame(centerOfMassFrame);
      desiredAngularAcceleration.changeFrame(centerOfMassFrame);
      desiredSpatialAcceleration.setIncludingFrame(centerOfMassFrame, worldFrame, centerOfMassFrame, desiredAngularAcceleration, desiredLinearAcceleration);
      desiredSpatialAcceleration.changeFrame(desiredControlFrame);
      desiredLinearAcceleration.setIncludingFrame(desiredSpatialAcceleration.getLinearPart());
      desiredAngularAcceleration.setIncludingFrame(desiredSpatialAcceleration.getAngularPart());
      desiredLinearAcceleration.changeFrame(worldFrame);
      desiredAngularAcceleration.changeFrame(worldFrame);
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
