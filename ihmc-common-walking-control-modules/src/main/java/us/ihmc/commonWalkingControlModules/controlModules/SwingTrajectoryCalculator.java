package us.ihmc.commonWalkingControlModules.controlModules;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class SwingTrajectoryCalculator
{
   private final TwoWaypointSwingGenerator swingTrajectoryOptimizer;

   private final ReferenceFrame oppositeSoleFrame;
   private final ReferenceFrame oppositeSoleZUpFrame;

   private final CurrentRigidBodyStateProvider currentStateProvider;

   private final YoEnum<TrajectoryType> activeTrajectoryType;

   private final RecyclingArrayList<FramePoint3D> positionWaypointsForSole = new RecyclingArrayList<>(2, FramePoint3D.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingWaypoints = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints, FrameSE3TrajectoryPoint.class);
   private final List<FixedFramePoint3DBasics> swingWaypointsForViz = new ArrayList<>();

   private final YoDouble swingDuration;
   private final YoDouble swingHeight;

   private final YoSwingTrajectoryParameters swingTrajectoryParameters;

   private final FramePose3D initialPose = new FramePose3D();
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();
   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();

   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();
   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();
   private final FramePoint3D stanceFootPosition = new FramePoint3D();

   private final FrameQuaternion tmpOrientation = new FrameQuaternion();
   private final FrameVector3D tmpVector = new FrameVector3D();

   private final FramePose3D footstepPose = new FramePose3D();
   private final FramePose3D lastFootstepPose = new FramePose3D();

   private final MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   private final double[] waypointProportions = new double[2];

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   private final RobotSide robotSide;

   private final YoBoolean ignoreInitialAngularVelocityZ;
   private final YoDouble maxInitialLinearVelocityMagnitude;
   private final YoDouble maxInitialAngularVelocityMagnitude;

   public SwingTrajectoryCalculator(String namePrefix,
                                    RobotSide robotSide,
                                    HighLevelHumanoidControllerToolbox controllerToolbox,
                                    WalkingControllerParameters walkingControllerParameters,
                                    YoSwingTrajectoryParameters swingTrajectoryParameters,
                                    YoRegistry registry)
   {
      this.robotSide = robotSide;
      this.swingTrajectoryParameters = swingTrajectoryParameters;
      double maxSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot();
      double minSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMinSwingHeightFromStanceFoot();
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();

      oppositeSoleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotSide.getOppositeSide());
      oppositeSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide.getOppositeSide());
      currentStateProvider = new CurrentRigidBodyStateProvider(controllerToolbox.getReferenceFrames().getSoleFrame(robotSide));

      activeTrajectoryType = new YoEnum<>(namePrefix + TrajectoryType.class.getSimpleName(), registry, TrajectoryType.class);
      swingHeight = new YoDouble(namePrefix + "Height", registry);
      swingDuration = new YoDouble(namePrefix + "Duration", registry);


      swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix, Footstep.maxNumberOfSwingWaypoints + 2, registry);

      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix,
                                                               minSwingHeightFromStanceFoot,
                                                               maxSwingHeightFromStanceFoot,
                                                               defaultSwingHeightFromStanceFoot,
                                                               registry,
                                                               controllerToolbox.getYoGraphicsListRegistry());
      double minDistanceToStance = walkingControllerParameters.getMinSwingTrajectoryClearanceFromStanceFoot();
      swingTrajectoryOptimizer.enableStanceCollisionAvoidance(robotSide, oppositeSoleZUpFrame, minDistanceToStance);

      ignoreInitialAngularVelocityZ = new YoBoolean(namePrefix + "IgnoreInitialAngularVelocityZ", registry);
      maxInitialLinearVelocityMagnitude = new YoDouble(namePrefix + "MaxInitialLinearVelocityMagnitude", registry);
      maxInitialAngularVelocityMagnitude = new YoDouble(namePrefix + "MaxInitialAngularVelocityMagnitude", registry);
      ignoreInitialAngularVelocityZ.set(walkingControllerParameters.ignoreSwingInitialAngularVelocityZ());
      maxInitialLinearVelocityMagnitude.set(walkingControllerParameters.getMaxSwingInitialLinearVelocityMagnitude());
      maxInitialAngularVelocityMagnitude.set(walkingControllerParameters.getMaxSwingInitialAngularVelocityMagnitude());

      lastFootstepPose.setToNaN();
      footstepPose.setToNaN();

      setupViz(controllerToolbox.getYoGraphicsListRegistry(), registry);
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

   public void informDone()
   {
      swingTrajectoryOptimizer.informDone();
      for (int i = 0; i < swingWaypointsForViz.size(); i++)
      {
         swingWaypointsForViz.get(i).setToNaN();
      }
   }

   public TrajectoryType getActiveTrajectoryType()
   {
      return activeTrajectoryType.getEnumValue();
   }

   public boolean doOptimizationUpdate()
   {
      return swingTrajectoryOptimizer.doOptimizationUpdate();
   }

   public void setFootstep(Footstep footstep)
   {
      footstepPose.setIncludingFrame(footstep.getFootstepPose());
      footstepPose.changeFrame(worldFrame);
      footstepPose.setZ(footstepPose.getZ() + swingTrajectoryParameters.getDesiredTouchdownHeightOffset());

      if (footstep.getTrajectoryType() == null)
      {
         activeTrajectoryType.set(TrajectoryType.DEFAULT);
      }
      else
      {
         activeTrajectoryType.set(footstep.getTrajectoryType());
      }
      this.positionWaypointsForSole.clear();
      this.swingWaypoints.clear();
      lastFootstepPose.changeFrame(worldFrame);

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.CUSTOM)
      {
         List<FramePoint3D> positionWaypointsForSole = footstep.getCustomPositionWaypoints();
         for (int i = 0; i < positionWaypointsForSole.size(); i++)
            this.positionWaypointsForSole.add().setIncludingFrame(positionWaypointsForSole.get(i));
      }
      else if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         List<FrameSE3TrajectoryPoint> swingWaypoints = footstep.getSwingTrajectory();
         for (int i = 0; i < swingWaypoints.size(); i++)
         {
            this.swingWaypoints.add().set(swingWaypoints.get(i));
         }
      }
      else
      {
         swingHeight.set(footstep.getSwingHeight());

         if (checkStepUpOrDown(footstepPose))
            activeTrajectoryType.set(TrajectoryType.OBSTACLE_CLEARANCE);

         RecyclingArrayList<MutableDouble> customWaypointProportions = footstep.getCustomWaypointProportions();
         if (customWaypointProportions.size() != 2)
         {
            if (!customWaypointProportions.isEmpty())
            {
               LogTools.warn("Ignoring custom waypoint proportions. Expected 2, got: " + customWaypointProportions.size());
            }

            List<DoubleProvider> waypointProportions = activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE ?
                  swingTrajectoryParameters.getObstacleClearanceProportions() :
                  swingTrajectoryParameters.getSwingWaypointProportions();
            this.waypointProportions[0] = waypointProportions.get(0).getValue();
            this.waypointProportions[1] = waypointProportions.get(1).getValue();
         }
         else
         {
            waypointProportions[0] = customWaypointProportions.get(0).getValue();
            waypointProportions[1] = customWaypointProportions.get(1).getValue();
         }
      }
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.set(swingDuration);
   }

   public void initializeTrajectoryStartConditions()
   {
      currentStateProvider.getPosition(initialPosition);
      currentStateProvider.getLinearVelocity(initialLinearVelocity);
      currentStateProvider.getOrientation(initialOrientation);
      currentStateProvider.getAngularVelocity(initialAngularVelocity);

      initialPose.getPosition().setMatchingFrame(initialPosition);
      initialPose.getOrientation().setMatchingFrame(initialOrientation);

      if (ignoreInitialAngularVelocityZ.getBooleanValue())
      {
         initialAngularVelocity.changeFrame(worldFrame);
         initialAngularVelocity.setZ(0.0);
      }
      initialLinearVelocity.clipToMaxLength(maxInitialLinearVelocityMagnitude.getDoubleValue());
      initialAngularVelocity.clipToMaxLength(maxInitialAngularVelocityMagnitude.getDoubleValue());
      stanceFootPosition.setToZero(oppositeSoleFrame);
   }

   public void initializeTrajectoryWaypoints(boolean initializeOptimizer)
   {
      footstepPose.get(finalPosition, finalOrientation);
      finalLinearVelocity.setIncludingFrame(swingTrajectoryParameters.getDesiredTouchdownVelocity());
      finalAngularVelocity.setToZero(worldFrame);

      // append current pose as initial trajectory point
      swingTrajectory.clear(worldFrame);
      boolean appendFootstepPose = true;
      double swingDuration = this.swingDuration.getValue();

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         if (swingWaypoints.get(0).getTime() > 1.0e-5)
         {
            swingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);
            swingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, initialAngularVelocity);
         }

         for (int i = 0; i < swingWaypoints.size(); i++)
         {
            swingTrajectory.appendPoseWaypoint(swingWaypoints.get(i));
         }

         appendFootstepPose = !Precision.equals(swingWaypoints.getLast().getTime(), swingDuration);
      }
      else
      {
         swingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);
         swingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, initialAngularVelocity);

         // TODO: initialize optimizer somewhere else
         if (initializeOptimizer)
         {
            initializeTrajectoryOptimizer();
         }

         for (int i = 0; i < swingTrajectoryOptimizer.getNumberOfWaypoints(); i++)
         {
            swingTrajectoryOptimizer.getWaypointData(i, tempPositionTrajectoryPoint);
            swingTrajectory.appendPositionWaypoint(tempPositionTrajectoryPoint);
         }

         // make the foot orientation better for avoidance
         if (swingTrajectoryParameters.addOrientationMidpointForObstacleClearance() && activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE)
         {
            tmpOrientation.setToZero(worldFrame);
            tmpVector.setToZero(worldFrame);
            tmpOrientation.interpolate(initialOrientation, finalOrientation, swingTrajectoryParameters.getMidpointOrientationInterpolationForObstacleClearance());
            swingTrajectory.appendOrientationWaypoint(0.5 * swingDuration, tmpOrientation, tmpVector);
         }
      }

      // append footstep pose if not provided in the waypoints
      if (appendFootstepPose)
      {
         modifyFinalOrientationForTouchdown(finalOrientation);
         if (activeTrajectoryType.getEnumValue() != TrajectoryType.WAYPOINTS && initializeOptimizer)
            swingTrajectoryOptimizer.getFinalVelocity(finalLinearVelocity);
         swingTrajectory.appendPositionWaypoint(swingDuration, finalPosition, finalLinearVelocity);
         swingTrajectory.appendOrientationWaypoint(swingDuration, finalOrientation, finalAngularVelocity);
      }
      else
      {
         // In this case our swing trajectory contains the touchdown so we should use those values to be continuous.
         FrameSE3TrajectoryPoint lastPoint = swingWaypoints.getLast();
         lastPoint.getPositionIncludingFrame(finalPosition);
         lastPoint.getLinearVelocityIncludingFrame(finalLinearVelocity);
         lastPoint.getOrientationIncludingFrame(finalOrientation);
         lastPoint.getAngularVelocity(finalAngularVelocity);
      }

      swingTrajectory.initialize();

      if (!swingWaypointsForViz.isEmpty() && activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         for (int i = 0; i < swingWaypoints.size(); i++)
         {
            swingWaypointsForViz.get(i).setMatchingFrame(swingWaypoints.get(i).getPosition());
         }
      }
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public FrameSE3TrajectoryPointBasics getSwingWaypoint(int i)
   {
      return swingWaypoints.get(i);
   }

   public FrameSE3TrajectoryPointBasics getLastSwingWaypoint()
   {
      return swingWaypoints.getLast();
   }

   private void initializeTrajectoryOptimizer()
   {
      swingTrajectoryOptimizer.setInitialConditions(initialPosition, initialLinearVelocity);
      swingTrajectoryOptimizer.setFinalConditions(finalPosition, finalLinearVelocity);
      swingTrajectoryOptimizer.setFinalConditionWeights(null, swingTrajectoryParameters.getTouchdownVelocityWeight());
      swingTrajectoryOptimizer.setStepTime(swingDuration.getValue());
      swingTrajectoryOptimizer.setTrajectoryType(activeTrajectoryType.getEnumValue(), positionWaypointsForSole);
      swingTrajectoryOptimizer.setSwingHeight(swingHeight.getDoubleValue());
      swingTrajectoryOptimizer.setStanceFootPosition(stanceFootPosition);
      swingTrajectoryOptimizer.setWaypointProportions(waypointProportions);
      swingTrajectoryOptimizer.initialize();
   }

   private boolean checkStepUpOrDown(FramePose3D footstepPose)
   {
      double zDifference = Math.abs(footstepPose.getZ() - lastFootstepPose.getZ());
      return zDifference > swingTrajectoryParameters.getMinHeightDifferenceForStepUpOrDown();
   }

   private void modifyFinalOrientationForTouchdown(FrameQuaternion finalOrientationToPack)
   {
      finalPosition.changeFrame(oppositeSoleZUpFrame);
      stanceFootPosition.changeFrame(oppositeSoleZUpFrame);
      double stepHeight = finalPosition.getZ() - stanceFootPosition.getZ();
      double initialFootstepPitch = finalOrientationToPack.getPitch();

      double footstepPitchModification;
      if (MathTools.intervalContains(stepHeight, swingTrajectoryParameters.getStepDownHeightForToeTouchdown(), swingTrajectoryParameters.getMaximumHeightForHeelTouchdown())
          && swingTrajectoryParameters.doHeelTouchdownIfPossible())
      { // not stepping down too far, and not stepping up too far, so do heel strike
         double stepLength = finalPosition.getX() - stanceFootPosition.getX();
         double heelTouchdownAngle = MathTools.clamp(-stepLength * swingTrajectoryParameters.getHeelTouchdownLengthRatio(), -swingTrajectoryParameters.getHeelTouchdownAngle());
         // use the footstep pitch if its greater than the heel strike angle
         footstepPitchModification = Math.max(initialFootstepPitch, heelTouchdownAngle);
         // decrease the foot pitch modification if next step pitches down
         footstepPitchModification = Math.min(footstepPitchModification, heelTouchdownAngle + initialFootstepPitch);
         footstepPitchModification -= initialFootstepPitch;
      }
      else if (stepHeight < swingTrajectoryParameters.getStepDownHeightForToeTouchdown() && swingTrajectoryParameters.doToeTouchdownIfPossible())
      { // stepping down and do toe touchdown
         double toeTouchdownAngle = MathTools.clamp(-swingTrajectoryParameters.getToeTouchdownDepthRatio() * (stepHeight - swingTrajectoryParameters.getStepDownHeightForToeTouchdown()),
                                                    swingTrajectoryParameters.getToeTouchdownAngle());
         footstepPitchModification = Math.max(toeTouchdownAngle, initialFootstepPitch);
         footstepPitchModification -= initialFootstepPitch;
      }
      else
      {
         footstepPitchModification = 0.0;
      }

      finalOrientationToPack.appendPitchRotation(footstepPitchModification);
   }
}
