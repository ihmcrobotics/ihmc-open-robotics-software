package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.controlModules.SwingTrajectoryCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith.FootLeapOfFaithModule;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPoseTrajectoryGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePose;
import us.ihmc.robotics.math.trajectories.*;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SwingState extends AbstractFootControlState
{
   private static final boolean visualizeAdjustedSwing = true;
   private static final boolean USE_ALL_LEG_JOINT_SWING_CORRECTOR = false;

   private final YoBoolean replanTrajectory;
   private final YoBoolean footstepWasAdjusted;

   private static final double maxScalingFactor = 1.5;
   private static final double minScalingFactor = 0.1;
   private static final double exponentialScalingRate = 5.0;

   private final SwingTrajectoryCalculator swingTrajectoryCalculator;
   private final MultipleWaypointsBlendedPoseTrajectoryGenerator blendedSwingTrajectory;
   private final BlendedPositionTrajectoryGeneratorVisualizer swingVisualizer;
   private final SoftTouchdownPoseTrajectoryGenerator touchdownTrajectory;
   private final C1ContinuousTrajectorySmoother swingTrajectorySmoother;
   private double swingTrajectoryBlendDuration = 0.0;

   private final List<FixedFramePoint3DBasics> swingWaypointsForViz = new ArrayList<>();

   private final ReferenceFrame soleFrame;
   private final YoSwingTrajectoryParameters swingTrajectoryParameters;

   private final FootLeapOfFaithModule leapOfFaithModule;

   private final YoFrameVector3D touchdownDesiredLinearVelocity;

   private final FramePose3D initialPose = new FramePose3D();
   private final FrameVector3DReadOnly finalAngularVelocity = new FrameVector3D(worldFrame);

   private final YoDouble swingDuration;
   private final YoDouble swingTimeSpeedUpFactor;
   private final YoDouble maxSwingTimeSpeedUpFactor;
   private final YoDouble minSwingTimeForDisturbanceRecovery;
   private final YoBoolean isSwingSpeedUpEnabled;
   private final YoDouble currentTime;
   private final YoDouble currentTimeWithSwingSpeedUp;
   private final YoDouble timeWhenAdjusted;
   private final YoFramePoint3D desiredPositionWhenAdjusted;
   private final YoFrameVector3D desiredVelocityWhenAdjusted;

   private final double controlDT;

   private final PoseReferenceFrame desiredSoleFrame = new PoseReferenceFrame("desiredSoleFrame", worldFrame);
   private final PoseReferenceFrame desiredControlFrame = new PoseReferenceFrame("desiredControlFrame", desiredSoleFrame);
   private final FramePose3D desiredPose = new FramePose3D();
   private final Twist desiredTwist = new Twist();
   private final SpatialAcceleration desiredSpatialAcceleration = new SpatialAcceleration();

   private final RigidBodyTransform transformFromToeToAnkle = new RigidBodyTransform();


   private final FramePose3D adjustedFootstepPose = new FramePose3D();
   private final RateLimitedYoFramePose rateLimitedAdjustedPose;

   private final FramePose3D footstepPose = new FramePose3D();

   // for unit testing and debugging:
   private final YoInteger currentTrajectoryWaypoint;
   private final YoFramePoint3D yoReferenceSolePosition;
   private final YoFrameVector3D yoReferenceSoleLinearVelocity;
   private final YoFramePoint3D yoDesiredSolePosition;
   private final YoFrameQuaternion yoDesiredSoleOrientation;
   private final YoFrameVector3D yoDesiredSoleLinearVelocity;
   private final YoFrameVector3D yoDesiredSoleAngularVelocity;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final WorkspaceLimiterControlModule workspaceLimiterControlModule;
   private final LegJointLimitAvoidanceControlModule legJointLimitAvoidanceControlModule;
   private final YoFramePoint3D yoDesiredPosition;
   private final YoFrameVector3D yoDesiredLinearVelocity;
   private final YoBoolean yoSetDesiredAccelerationToZero;
   private final YoBoolean yoSetDesiredVelocityToZero;
   private final YoBoolean scaleSecondaryJointWeights;
   private final YoDouble secondaryJointWeightScale;
   private Vector3DReadOnly nominalAngularWeight;
   private Vector3DReadOnly nominalLinearWeight;
   private final YoFrameVector3D currentAngularWeight;
   private final YoFrameVector3D currentLinearWeight;
   private final ReferenceFrame ankleFrame;
   private final PoseReferenceFrame controlFrame;
   private final PIDSE3GainsReadOnly gains;
   private final FramePoint3D desiredAnklePosition = new FramePoint3D();
   private final RigidBodyTransform oldBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform newBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewBodyFrameToOldBodyFrame = new RigidBodyTransform();

   private final YoFrameVector3D touchdownDesiredLinearAcceleration;

   private final OneDoFJointBasics kneeJoint;
   private final YoDouble kickOffVelocity;
   private final YoDouble kickOffDuration;
   private final YoPDGains kickOffGains;

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final JointspaceAccelerationCommand jointspaceAccelerationCommand = new JointspaceAccelerationCommand();

   public SwingState(FootControlHelper footControlHelper, PIDSE3GainsReadOnly gains, YoRegistry registry)
   {
      super(footControlHelper);
      this.gains = gains;

      this.workspaceLimiterControlModule = footControlHelper.getWorkspaceLimiterControlModule();

      swingTrajectoryParameters = footControlHelper.getSwingTrajectoryParameters();
      RigidBodyBasics foot = contactableFoot.getRigidBody();
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "FootSwing";
      yoDesiredLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      yoDesiredLinearVelocity.setToNaN();
      yoDesiredPosition = new YoFramePoint3D(namePrefix + "DesiredPosition", worldFrame, registry);
      yoDesiredPosition.setToNaN();
      yoSetDesiredAccelerationToZero = new YoBoolean(namePrefix + "SetDesiredAccelerationToZero", registry);
      yoSetDesiredVelocityToZero = new YoBoolean(namePrefix + "SetDesiredVelocityToZero", registry);

      scaleSecondaryJointWeights = new YoBoolean(namePrefix + "ScaleSecondaryJointWeights", registry);
      secondaryJointWeightScale = new YoDouble(namePrefix + "SecondaryJointWeightScale", registry);
      secondaryJointWeightScale.set(1.0);

      currentAngularWeight = new YoFrameVector3D(namePrefix + "CurrentAngularWeight", worldFrame, registry);
      currentLinearWeight = new YoFrameVector3D(namePrefix + "CurrentLinearWeight", worldFrame, registry);

      touchdownDesiredLinearAcceleration = new YoFrameVector3D(namePrefix + "DesiredTouchdownAcceleration", worldFrame, registry);

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
         legJointLimitAvoidanceControlModule = new LegJointLimitAvoidanceControlModule(namePrefix, registry, controllerToolbox, robotSide);
      else
         legJointLimitAvoidanceControlModule = null;

      ankleFrame = contactableFoot.getFrameAfterParentJoint();
      controlFrame = new PoseReferenceFrame("controlFrame", contactableFoot.getRigidBody().getBodyFixedFrame());

      spatialFeedbackControlCommand.set(rootBody, foot);
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);
      ReferenceFrame linearGainsFrame = footControlHelper.getHighLevelHumanoidControllerToolbox().getPelvisZUpFrame();
      spatialFeedbackControlCommand.setGainsFrames(null, linearGainsFrame);
      FramePose3D anklePoseInFoot = new FramePose3D(ankleFrame);
      anklePoseInFoot.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(anklePoseInFoot);

      kickOffVelocity = new YoDouble(namePrefix + "KickOffVelocity", registry);
      kickOffGains = new YoPDGains(namePrefix + "KickOffGains", registry);
      kickOffDuration = new YoDouble(namePrefix + "KickOffDuration", registry);
      kickOffGains.setKd(50);
      kickOffVelocity.set(3.0);
      kickOffDuration.set(0.15);
      FullHumanoidRobotModel fullRobotModel = footControlHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel();
      kneeJoint = fullRobotModel.getLegJoint(footControlHelper.getRobotSide(), LegJointName.KNEE_PITCH);

      touchdownDesiredLinearVelocity = new YoFrameVector3D(namePrefix + "TouchdownDesiredLinearVelocity", worldFrame, registry);

      controlDT = footControlHelper.getHighLevelHumanoidControllerToolbox().getControlDT();

      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();

      replanTrajectory = new YoBoolean(namePrefix + "ReplanTrajectory", registry);
      footstepWasAdjusted = new YoBoolean(namePrefix + "FootstepWasAdjusted", registry);

      rateLimitedAdjustedPose = new RateLimitedYoFramePose(namePrefix + "AdjustedFootstepPose", "", registry, 10.0, controlDT, worldFrame);

      soleFrame = footControlHelper.getHighLevelHumanoidControllerToolbox().getReferenceFrames().getSoleFrame(robotSide);
      ReferenceFrame footFrame = contactableFoot.getFrameAfterParentJoint();
      ReferenceFrame toeFrame = createToeFrame(robotSide);
      ReferenceFrame controlFrame = walkingControllerParameters.controlToeDuringSwing() ? toeFrame : footFrame;
      RigidBodyTransform soleToControlFrameTransform = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(soleToControlFrameTransform, soleFrame);
      desiredControlFrame.setPoseAndUpdate(soleToControlFrameTransform);

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      swingDuration = new YoDouble(namePrefix + "Duration", registry);
      desiredPositionWhenAdjusted = new YoFramePoint3D(namePrefix + "DesiredPositionWhenAdjusted", worldFrame, registry);
      desiredVelocityWhenAdjusted = new YoFrameVector3D(namePrefix + "DesiredVelocityWhenAdjusted", worldFrame, registry);
      timeWhenAdjusted = new YoDouble(namePrefix + "TimeWhenStepAdjusted", registry);
      swingTrajectoryCalculator = footControlHelper.getSwingTrajectoryCalculator();
      blendedSwingTrajectory = new MultipleWaypointsBlendedPoseTrajectoryGenerator(namePrefix, swingTrajectoryCalculator.getSwingTrajectory(), worldFrame, registry);
      swingTrajectorySmoother = new C1ContinuousTrajectorySmoother(namePrefix, blendedSwingTrajectory, registry);
      touchdownTrajectory = new SoftTouchdownPoseTrajectoryGenerator(namePrefix + "Touchdown", registry);

      if (visualizeAdjustedSwing)
         swingVisualizer = new BlendedPositionTrajectoryGeneratorVisualizer(namePrefix, blendedSwingTrajectory, swingDuration, registry, yoGraphicsListRegistry);
      else
         swingVisualizer = null;

      swingTimeSpeedUpFactor = new YoDouble(namePrefix + "TimeSpeedUpFactor", registry);
      minSwingTimeForDisturbanceRecovery = new YoDouble(namePrefix + "MinTimeForDisturbanceRecovery", registry);
      minSwingTimeForDisturbanceRecovery.set(walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery());
      maxSwingTimeSpeedUpFactor = new YoDouble(namePrefix + "MaxTimeSpeedUpFactor", registry);
      currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      currentTimeWithSwingSpeedUp = new YoDouble(namePrefix + "CurrentTimeWithSpeedUp", registry);
      isSwingSpeedUpEnabled = new YoBoolean(namePrefix + "IsSpeedUpEnabled", registry);
      isSwingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());

      swingTimeSpeedUpFactor.setToNaN();

      scaleSecondaryJointWeights.set(walkingControllerParameters.applySecondaryJointScaleDuringSwing());

      LeapOfFaithParameters leapOfFaithParameters = walkingControllerParameters.getLeapOfFaithParameters();
      leapOfFaithModule = new FootLeapOfFaithModule(swingDuration, leapOfFaithParameters, registry);

      FramePose3D controlFramePose = new FramePose3D(controlFrame);
      controlFramePose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(controlFramePose);

      footstepPose.setToNaN();
      footstepWasAdjusted.set(false);

      currentTrajectoryWaypoint = new YoInteger(namePrefix + "CurrentTrajectoryWaypoint", registry);
      yoReferenceSolePosition = new YoFramePoint3D(namePrefix + "ReferenceSolePositionInWorld", worldFrame, registry);
      yoReferenceSoleLinearVelocity = new YoFrameVector3D(namePrefix + "ReferenceSoleLinearVelocityInWorld", worldFrame, registry);
      yoDesiredSolePosition = new YoFramePoint3D(namePrefix + "DesiredSolePositionInWorld", worldFrame, registry);
      yoDesiredSoleOrientation = new YoFrameQuaternion(namePrefix + "DesiredSoleOrientationInWorld", worldFrame, registry);
      yoDesiredSoleLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInWorld", worldFrame, registry);
      yoDesiredSoleAngularVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleAngularVelocityInWorld", worldFrame, registry);

      setupViz(yoGraphicsListRegistry, registry);
   }

   private void setupViz(YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry registry)
   {
      if (yoGraphicsListRegistry == null)
      {
         return;
      }

      for (int i = 0; i < Footstep.maxNumberOfSwingWaypoints; i++)
      {
         YoFramePoint3D yoWaypoint = new YoFramePoint3D("SwingWaypoint" + robotSide.getPascalCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition waypointViz = new YoGraphicPosition("SwingWaypoint" + robotSide.getPascalCaseName() + i, yoWaypoint , 0.01, YoAppearance.GreenYellow());
         yoWaypoint.setToNaN();
         yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), waypointViz);
         swingWaypointsForViz.add(yoWaypoint);
      }
   }

   private ReferenceFrame createToeFrame(RobotSide robotSide)
   {
      ContactableFoot contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      ReferenceFrame footFrame = controllerToolbox.getReferenceFrames().getFootFrame(robotSide);
      FramePoint2D toeContactPoint2d = new FramePoint2D();
      contactableFoot.getToeOffContactPoint(toeContactPoint2d);
      FramePoint3D toeContactPoint = new FramePoint3D();
      toeContactPoint.setIncludingFrame(toeContactPoint2d, 0.0);
      toeContactPoint.changeFrame(footFrame);

      transformFromToeToAnkle.getTranslation().set(toeContactPoint);
      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() + "ToeFrame", footFrame,
                                                                               transformFromToeToAnkle);
   }

   private void initializeTrajectory()
   {
      initialPose.setToZero(soleFrame);

      swingTrajectoryCalculator.setInitialConditionsToCurrent();

      swingTrajectorySmoother.initialize();
      fillAndInitializeTrajectories(true);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();
      swingTrajectoryCalculator.setShouldVisualize(true);
      currentTime.set(0.0);
      swingTimeSpeedUpFactor.set(1.0);
      currentTimeWithSwingSpeedUp.set(Double.NaN);
      replanTrajectory.set(false);

      if (workspaceLimiterControlModule != null)
         workspaceLimiterControlModule.setCheckVelocityForSwingSingularityAvoidance(true);

      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
      contactState.notifyContactStateHasChanged();

      spatialFeedbackControlCommand.resetSecondaryTaskJointWeightScale();

      initializeTrajectory();
   }

   @Override
   public void onExit(double timeInState)
   {
      super.onExit(timeInState);
      currentTime.set(0.0);
      swingTimeSpeedUpFactor.set(Double.NaN);
      currentTimeWithSwingSpeedUp.set(Double.NaN);

      swingTrajectoryCalculator.informDone();
      for (int i = 0; i < swingWaypointsForViz.size(); i++)
      {
         swingWaypointsForViz.get(i).setToNaN();
      }

      yoReferenceSolePosition.setToNaN();
      yoReferenceSoleLinearVelocity.setToNaN();
      yoDesiredSolePosition.setToNaN();
      yoDesiredSoleOrientation.setToNaN();
      yoDesiredSoleLinearVelocity.setToNaN();
      yoDesiredSoleAngularVelocity.setToNaN();
      yoDesiredPosition.setToNaN();
      yoDesiredLinearVelocity.setToNaN();

      footstepWasAdjusted.set(false);
      desiredPositionWhenAdjusted.setToNaN();
      desiredVelocityWhenAdjusted.setToNaN();
      timeWhenAdjusted.setToNaN();;
   }

   @Override
   public void doSpecificAction(double timeInState)
   {
      computeAndPackTrajectory(timeInState);

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
      {
         legJointLimitAvoidanceControlModule.correctSwingFootTrajectory(desiredPosition,
                                                                        desiredOrientation,
                                                                        desiredLinearVelocity,
                                                                        desiredAngularVelocity,
                                                                        desiredLinearAcceleration,
                                                                        desiredAngularAcceleration);
      }

      if (workspaceLimiterControlModule != null)
      {
         desiredPose.setIncludingFrame(desiredPosition, desiredOrientation);
         changeDesiredPoseBodyFrame(controlFrame, ankleFrame, desiredPose);
         desiredAnklePosition.setIncludingFrame(desiredPose.getPosition());

         workspaceLimiterControlModule.correctSwingFootTrajectory(desiredAnklePosition, desiredLinearVelocity, desiredLinearAcceleration);

         desiredPose.getPosition().set(desiredAnklePosition);
         changeDesiredPoseBodyFrame(ankleFrame, controlFrame, desiredPose);
         desiredPosition.setIncludingFrame(desiredPose.getPosition());
      }

      if (yoSetDesiredVelocityToZero.getBooleanValue())
      {
         desiredLinearVelocity.setToZero();
      }

      if (yoSetDesiredAccelerationToZero.getBooleanValue())
      {
         desiredLinearAcceleration.setToZero();
      }

      computeCurrentWeights(nominalAngularWeight, nominalLinearWeight, currentAngularWeight, currentLinearWeight);

      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation,
                                                       desiredPosition,
                                                       desiredAngularVelocity,
                                                       desiredLinearVelocity,
                                                       desiredAngularAcceleration,
                                                       desiredLinearAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(currentAngularWeight, currentLinearWeight);
      spatialFeedbackControlCommand.setScaleSecondaryTaskJointWeight(scaleSecondaryJointWeights.getBooleanValue(), secondaryJointWeightScale.getDoubleValue());
      spatialFeedbackControlCommand.setGains(gains);

      double desiredAcceleration = kickOffGains.getKd() * (kickOffVelocity.getDoubleValue() - kneeJoint.getQd());

      jointspaceAccelerationCommand.clear();
      jointspaceAccelerationCommand.addJoint(kneeJoint, desiredAcceleration);
      jointspaceAccelerationCommand.setConstraintType(ConstraintType.GEQ_INEQUALITY);

      yoDesiredPosition.setMatchingFrame(desiredPosition);
      yoDesiredLinearVelocity.setMatchingFrame(desiredLinearVelocity);
   }

   private void computeAndPackTrajectory(double timeInState)
   {
      currentTime.set(timeInState);

      if (footstepWasAdjusted.getBooleanValue())
      {
         if (!rateLimitedAdjustedPose.geometricallyEquals(adjustedFootstepPose, 1.0e-7))
            replanTrajectory.set(true); // As long as the rate-limited pose has not reached the adjusted pose, we'll have to replan the swing.

         rateLimitedAdjustedPose.update(adjustedFootstepPose);
      }

      double time;
      if (!isSwingSpeedUpEnabled.getBooleanValue() || currentTimeWithSwingSpeedUp.isNaN())
      {
         time = currentTime.getDoubleValue();
      }
      else
      {
         currentTimeWithSwingSpeedUp.add(swingTimeSpeedUpFactor.getDoubleValue() * controlDT);
         time = currentTimeWithSwingSpeedUp.getDoubleValue();
      }

      FixedFramePoseTrajectoryGenerator activeTrajectory;
      boolean isInTouchdown = false;
      if (time > swingDuration.getDoubleValue())
      {
         activeTrajectory = touchdownTrajectory;
         isInTouchdown = true;
      }
      else
         activeTrajectory = blendedSwingTrajectory;

      if (swingTrajectoryCalculator.getActiveTrajectoryType() != TrajectoryType.WAYPOINTS && swingTrajectoryCalculator.doOptimizationUpdate())
      { // haven't finished original planning
         fillAndInitializeTrajectories(false);

      }
      else if (replanTrajectory.getBooleanValue()) // need to update the swing trajectory to account for the end position moving
         fillAndInitializeBlendedTrajectories();

      replanTrajectory.set(false);

      activeTrajectory.compute(time);
      activeTrajectory.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      activeTrajectory.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      leapOfFaithModule.compute(time);

      // set the reference values to get the ones coming from the trajectory
      yoReferenceSolePosition.setMatchingFrame(desiredPosition);
      yoReferenceSoleLinearVelocity.setMatchingFrame(desiredLinearVelocity);

      if (!isInTouchdown)
      { // we're still in swing, so update the desired setpoints using the smoother
         swingTrajectorySmoother.compute(time);
         swingTrajectorySmoother.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      }

      if (isSwingSpeedUpEnabled.getBooleanValue() && !currentTimeWithSwingSpeedUp.isNaN())
      {
         desiredLinearVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());
         yoReferenceSoleLinearVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());
         desiredAngularVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());

         double speedUpFactorSquared = swingTimeSpeedUpFactor.getDoubleValue() * swingTimeSpeedUpFactor.getDoubleValue();
         desiredLinearAcceleration.scale(speedUpFactorSquared);
         desiredAngularAcceleration.scale(speedUpFactorSquared);
      }

      yoDesiredSolePosition.setMatchingFrame(desiredPosition);
      yoDesiredSoleOrientation.setMatchingFrame(desiredOrientation);
      yoDesiredSoleLinearVelocity.setMatchingFrame(desiredLinearVelocity);
      yoDesiredSoleAngularVelocity.setMatchingFrame(desiredAngularVelocity);
      currentTrajectoryWaypoint.set(blendedSwingTrajectory.getCurrentPositionWaypointIndex());

      transformDesiredsFromSoleFrameToControlFrame();
      secondaryJointWeightScale.set(computeSecondaryJointWeightScale(time));

      if (swingVisualizer != null)
         swingVisualizer.visualize();
   }

   public void setFootstep(Footstep footstep, double swingTime)
   {
      setFootstep(footstep, swingTime, null, null);
   }

   public void setFootstep(Footstep footstep, double swingTime, FrameVector3DReadOnly finalCoMVelocity, FrameVector3DReadOnly finalCoMAcceleration)
   {
      setFootstepDurationInternal(swingTime);

      swingTrajectoryCalculator.setFootstep(footstep);

      touchdownDesiredLinearVelocity.set(swingTrajectoryParameters.getDesiredTouchdownVelocity());
      touchdownDesiredLinearVelocity.scale(1.0 / Math.min(swingDuration.getDoubleValue(), 1.0));
      if (finalCoMVelocity != null)
      {
         touchdownDesiredLinearVelocity.checkReferenceFrameMatch(finalCoMVelocity);
         double injectionRatio = swingTrajectoryParameters.getFinalCoMVelocityInjectionRatio();
         touchdownDesiredLinearVelocity.setX(injectionRatio * finalCoMVelocity.getX());
         touchdownDesiredLinearVelocity.setY(injectionRatio * finalCoMVelocity.getY());
      }
      swingTrajectoryCalculator.setFinalLinearVelocity(touchdownDesiredLinearVelocity);

      touchdownDesiredLinearAcceleration.set(swingTrajectoryParameters.getDesiredTouchdownAcceleration());
      if (finalCoMAcceleration != null)
      {
         touchdownDesiredLinearAcceleration.checkReferenceFrameMatch(finalCoMAcceleration);
         double injectionRatio = swingTrajectoryParameters.getFinalCoMAccelerationInjectionRatio();
         touchdownDesiredLinearAcceleration.addX(injectionRatio * finalCoMAcceleration.getX());
         touchdownDesiredLinearAcceleration.addY(injectionRatio * finalCoMAcceleration.getY());
      }

      setFootstepInternal(footstep);

      adjustedFootstepPose.set(footstepPose);
      rateLimitedAdjustedPose.set(footstepPose);
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpFactor)
   {
      if (isSwingSpeedUpEnabled.getBooleanValue() && (speedUpFactor > 1.001 && speedUpFactor > swingTimeSpeedUpFactor.getDoubleValue()))
      {
         speedUpFactor = MathTools.clamp(speedUpFactor, swingTimeSpeedUpFactor.getDoubleValue(), maxSwingTimeSpeedUpFactor.getDoubleValue());

         swingTimeSpeedUpFactor.set(speedUpFactor);
         if (currentTimeWithSwingSpeedUp.isNaN())
            currentTimeWithSwingSpeedUp.set(currentTime.getDoubleValue());
      }

      return computeSwingTimeRemaining(currentTime.getDoubleValue());
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, double swingTime)
   {
      setAdjustedFootstepAndTime(adjustedFootstep, null, null, swingTime);
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, FrameVector3DReadOnly finalCoMVelocity, FrameVector3DReadOnly finalCoMAcceleration, double swingTime)
   {
      replanTrajectory.set(true);
      footstepWasAdjusted.set(true);

      timeWhenAdjusted.set(currentTime.getValue());
      swingTrajectorySmoother.compute(currentTime.getDoubleValue());

      desiredPositionWhenAdjusted.set(swingTrajectorySmoother.getPosition());
      desiredVelocityWhenAdjusted.set(swingTrajectorySmoother.getVelocity());

      adjustedFootstepPose.setIncludingFrame(adjustedFootstep.getFootstepPose());

      if (finalCoMVelocity != null)
      {
         touchdownDesiredLinearVelocity.checkReferenceFrameMatch(finalCoMVelocity);
         double injectionRatio = swingTrajectoryParameters.getFinalCoMVelocityInjectionRatio();
         touchdownDesiredLinearVelocity.setX(injectionRatio * finalCoMVelocity.getX());
         touchdownDesiredLinearVelocity.setY(injectionRatio * finalCoMVelocity.getY());
      }

      if (finalCoMAcceleration != null)
      {
         touchdownDesiredLinearAcceleration.checkReferenceFrameMatch(finalCoMAcceleration);
         double injectionRatio = swingTrajectoryParameters.getFinalCoMAccelerationInjectionRatio();
         touchdownDesiredLinearAcceleration.addX(injectionRatio * finalCoMAcceleration.getX());
         touchdownDesiredLinearAcceleration.addY(injectionRatio * finalCoMAcceleration.getY());
      }

      setFootstepDurationInternal(swingTime);
   }

   private void fillAndInitializeTrajectories(boolean initializeOptimizer)
   {
      swingTrajectoryCalculator.initializeTrajectoryWaypoints(initializeOptimizer);

      // Setup touchdown trajectory.
      touchdownDesiredLinearVelocity.set(swingTrajectoryCalculator.getFinalLinearVelocity());

      touchdownTrajectory.setLinearTrajectory(swingDuration.getDoubleValue(),
                                              footstepPose.getPosition(),
                                              touchdownDesiredLinearVelocity,
                                              touchdownDesiredLinearAcceleration);
      touchdownTrajectory.setOrientation(footstepPose.getOrientation(), finalAngularVelocity);

      fillAndInitializeBlendedTrajectories();
   }

   private final PoseReferenceFrame footstepFrame = new PoseReferenceFrame("FootstepFrame", worldFrame);
   private final PoseReferenceFrame adjustedFootstepFrame = new PoseReferenceFrame("AdjustedFootstepFrame", worldFrame);
   private final FramePose3D adjustedWaypoint = new FramePose3D();

   private void fillAndInitializeBlendedTrajectories()
   {
      double swingDuration = this.swingDuration.getDoubleValue();
      blendedSwingTrajectory.clear();
      if (swingVisualizer != null)
         swingVisualizer.hideVisualization();
      if (swingTrajectoryBlendDuration > 0.0)
      {
         initialPose.changeFrame(worldFrame);
         blendedSwingTrajectory.blendInitialConstraint(initialPose, 0.0, swingTrajectoryBlendDuration);
      }
      if (footstepWasAdjusted.getBooleanValue())
      {
         // If there is a swing waypoint at the end of swing we want to preserve its transform to the footstep pose to not blend out
         // any touchdown trajectory when doing step adjustment.
         FrameSE3TrajectoryPointBasics lastSwingWaypoint = swingTrajectoryCalculator.getLastSwingWaypoint();
         if (swingTrajectoryCalculator.getActiveTrajectoryType() == TrajectoryType.WAYPOINTS && Precision.equals(lastSwingWaypoint.getTime(), swingDuration))
         {
            footstepFrame.setPoseAndUpdate(footstepPose);
            adjustedFootstepFrame.setPoseAndUpdate(rateLimitedAdjustedPose);
            lastSwingWaypoint.getPose(adjustedWaypoint);
            adjustedWaypoint.changeFrame(footstepFrame);
            adjustedWaypoint.setReferenceFrame(adjustedFootstepFrame);
            adjustedWaypoint.changeFrame(worldFrame);
            blendedSwingTrajectory.blendFinalConstraint(adjustedWaypoint, swingDuration, swingDuration - timeWhenAdjusted.getValue());
            touchdownTrajectory.setLinearTrajectory(swingDuration,
                                                    adjustedWaypoint.getPosition(), touchdownDesiredLinearVelocity,
                                                    touchdownDesiredLinearAcceleration);
            touchdownTrajectory.setOrientation(adjustedWaypoint.getOrientation());
         }
         else
         {
            BlendedOrientationTrajectoryGenerator orientationTrajectory = blendedSwingTrajectory.getOrientationTrajectoryGenerator();
            BlendedWaypointPositionTrajectoryGenerator positionTrajectory = blendedSwingTrajectory.getPositionTrajectoryGenerator();
            orientationTrajectory.blendFinalConstraint(rateLimitedAdjustedPose.getOrientation(), swingDuration, swingDuration);

            positionTrajectory.clear();
            positionTrajectory.addBlendWaypoint(swingTrajectoryCalculator.getSwingTrajectory().getPositionTrajectory().getWaypoint(0).getPosition(), 0.0);
            //            positionTrajectory.addBlendWaypoint(desiredPositionWhenAdjusted, swingDuration - timeRemainingWhenAdjusted.getValue());
            positionTrajectory.addBlendWaypoint(rateLimitedAdjustedPose.getPosition(), touchdownDesiredLinearVelocity, swingDuration);
            positionTrajectory.initializeBlendingTrajectory();
            swingTrajectorySmoother.updateErrorDynamicsAtTime(timeWhenAdjusted.getDoubleValue(), desiredPositionWhenAdjusted, desiredVelocityWhenAdjusted);

            swingTrajectorySmoother.updateErrorDynamicsAtTime(timeWhenAdjusted.getDoubleValue(), desiredPositionWhenAdjusted, desiredVelocityWhenAdjusted);

            touchdownTrajectory.setLinearTrajectory(swingDuration,
                                                    rateLimitedAdjustedPose.getPosition(),
                                                    touchdownDesiredLinearVelocity,
                                                    touchdownDesiredLinearAcceleration);
            touchdownTrajectory.setOrientation(rateLimitedAdjustedPose.getOrientation());
         }

      }

      blendedSwingTrajectory.initialize();
      touchdownTrajectory.initialize();
      if (swingVisualizer != null)
         swingVisualizer.visualize();

      if (!swingWaypointsForViz.isEmpty() && swingTrajectoryCalculator.getActiveTrajectoryType() == TrajectoryType.WAYPOINTS)
      {
         for (int i = 0; i < swingTrajectoryCalculator.getNumberOfSwingWaypoints(); i++)
         {
            blendedSwingTrajectory.compute(swingTrajectoryCalculator.getSwingWaypoint(i).getTime());
            swingWaypointsForViz.get(i).setMatchingFrame(blendedSwingTrajectory.getPosition());
         }
      }
   }

   private void computeCurrentWeights(Vector3DReadOnly nominalAngularWeight, Vector3DReadOnly nominalLinearWeight, Vector3DBasics currentAngularWeightToPack,
                                      Vector3DBasics currentLinearWeightToPack)
   {
      double percentThroughSwing = currentTime.getDoubleValue() / swingDuration.getValue();
      double minAlpha = 0.25;
      double alpha;
      if (percentThroughSwing > 0.75)
         alpha = 1.0;
      else
         alpha = minAlpha + (1.0 - minAlpha) * percentThroughSwing / 0.75;

      currentAngularWeightToPack.setAndScale(alpha, nominalAngularWeight);
      leapOfFaithModule.scaleFootWeight(nominalLinearWeight, currentLinearWeightToPack);
   }

   private void transformDesiredsFromSoleFrameToControlFrame()
   {
      desiredSoleFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);

      // change pose
      desiredPose.setToZero(desiredControlFrame);
      desiredPose.changeFrame(worldFrame);
      desiredPosition.setIncludingFrame(desiredPose.getPosition());
      desiredOrientation.setIncludingFrame(desiredPose.getOrientation());

      // change twist
      desiredLinearVelocity.changeFrame(desiredSoleFrame);
      desiredAngularVelocity.changeFrame(desiredSoleFrame);
      desiredTwist.setIncludingFrame(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredAngularVelocity, desiredLinearVelocity);
      desiredTwist.changeFrame(desiredControlFrame);
      desiredLinearVelocity.setIncludingFrame(desiredTwist.getLinearPart());
      desiredAngularVelocity.setIncludingFrame(desiredTwist.getAngularPart());
      desiredLinearVelocity.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);

      // change spatial acceleration
      desiredLinearAcceleration.changeFrame(desiredSoleFrame);
      desiredAngularAcceleration.changeFrame(desiredSoleFrame);
      desiredSpatialAcceleration.setIncludingFrame(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredAngularAcceleration, desiredLinearAcceleration);
      desiredSpatialAcceleration.changeFrame(desiredControlFrame);
      desiredLinearAcceleration.setIncludingFrame(desiredSpatialAcceleration.getLinearPart());
      desiredAngularAcceleration.setIncludingFrame(desiredSpatialAcceleration.getAngularPart());
      desiredLinearAcceleration.changeFrame(worldFrame);
      desiredAngularAcceleration.changeFrame(worldFrame);
   }

   private void setFootstepDurationInternal(double swingTime)
   {
      swingDuration.set(swingTime);
      swingTrajectoryCalculator.setSwingDuration(swingTime);
      maxSwingTimeSpeedUpFactor.set(Math.max(swingTime / minSwingTimeForDisturbanceRecovery.getDoubleValue(), 1.0));
   }

   private void setFootstepInternal(Footstep footstep)
   {
      footstep.getPose(footstepPose);
      footstepPose.changeFrame(worldFrame);
      footstepPose.setZ(footstepPose.getZ() + swingTrajectoryParameters.getDesiredTouchdownHeightOffset());

      if (swingTrajectoryCalculator.getActiveTrajectoryType() == TrajectoryType.WAYPOINTS)
         swingTrajectoryBlendDuration = footstep.getSwingTrajectoryBlendDuration();
      else
         swingTrajectoryBlendDuration = 0.0;
   }

   public double getSwingTimeRemaining()
   {
      return computeSwingTimeRemaining(currentTime.getValue());
   }

   public double getFractionThroughSwing()
   {
      if (!currentTimeWithSwingSpeedUp.isNaN())
         return currentTimeWithSwingSpeedUp.getDoubleValue() / swingDuration.getValue();
      else
         return currentTime.getDoubleValue() / swingDuration.getValue();
   }

   private double computeSwingTimeRemaining(double timeInState)
   {
      double swingDuration = this.swingDuration.getDoubleValue();
      if (!currentTimeWithSwingSpeedUp.isNaN())
      {
         double swingTimeRemaining = (swingDuration - currentTimeWithSwingSpeedUp.getDoubleValue()) / swingTimeSpeedUpFactor.getDoubleValue();
         return swingTimeRemaining;
      }
      else
      {
         return swingDuration - timeInState;
      }
   }

   private double computeSecondaryJointWeightScale(double timeInState)
   {
      double phaseInSwingState = timeInState / swingDuration.getDoubleValue();

      double scaleFactor;
      if (timeInState < swingDuration.getDoubleValue())
         scaleFactor = (maxScalingFactor - minScalingFactor) * (1.0 - Math.exp(-exponentialScalingRate * phaseInSwingState)) + minScalingFactor;
      else
         scaleFactor = (1.0 + 0.1 * (timeInState - swingDuration.getDoubleValue())) * maxScalingFactor;

      return scaleFactor;
   }

   private void changeControlFrame(FramePose3D controlFramePoseInEndEffector)
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

   private void changeDesiredPoseBodyFrame(ReferenceFrame oldBodyFrame, ReferenceFrame newBodyFrame, FramePose3D framePoseToModify)
   {
      if (oldBodyFrame == newBodyFrame)
         return;

      framePoseToModify.get(oldBodyFrameDesiredTransform);
      newBodyFrame.getTransformToDesiredFrame(transformFromNewBodyFrameToOldBodyFrame, oldBodyFrame);
      newBodyFrameDesiredTransform.set(oldBodyFrameDesiredTransform);
      newBodyFrameDesiredTransform.multiply(transformFromNewBodyFrameToOldBodyFrame);
      framePoseToModify.set(newBodyFrameDesiredTransform);
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory()
   {
      return swingTrajectoryCalculator.getSwingTrajectory();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (currentTime.getDoubleValue() > kickOffDuration.getDoubleValue())
         return null;
      if (jointspaceAccelerationCommand.getDesiredAcceleration(0).get(0) < 0.0)
         return null;

      return jointspaceAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<FeedbackControlCommandList> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
      return feedbackControlCommandList;
   }

}
