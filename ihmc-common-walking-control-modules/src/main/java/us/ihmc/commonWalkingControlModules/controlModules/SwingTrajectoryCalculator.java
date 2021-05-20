package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class SwingTrajectoryCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TwoWaypointSwingGenerator swingTrajectoryOptimizer;

   private final MovingReferenceFrame soleFrame;
   private final ReferenceFrame oppositeSoleFrame;
   private final ReferenceFrame oppositeSoleZUpFrame;

   private final CurrentRigidBodyStateProvider currentStateProvider;

   private final YoEnum<TrajectoryType> activeTrajectoryType;

   private final RecyclingArrayList<FramePoint3D> positionWaypointsForSole = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints, FramePoint3D.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingWaypoints = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints,
                                                                                                       FrameSE3TrajectoryPoint.class);

   private final YoDouble swingDuration;
   private final YoDouble swingHeight;

   private final YoSwingTrajectoryParameters swingTrajectoryParameters;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();
   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();

   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();
   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();

   private final FrameVector3D finalCoMVelocity = new FrameVector3D();

   private final FramePoint3D stanceFootPosition = new FramePoint3D();

   private final FrameQuaternion tmpOrientation = new FrameQuaternion();
   private final FrameVector3D tmpVector = new FrameVector3D();

   private final FramePoint3D lastFootstepPosition = new FramePoint3D();

   private final MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   private final double[] waypointProportions = new double[2];

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   public SwingTrajectoryCalculator(String namePrefix, RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox,
                                    WalkingControllerParameters walkingControllerParameters, YoSwingTrajectoryParameters swingTrajectoryParameters,
                                    YoRegistry parentRegistry)
   {
      this.swingTrajectoryParameters = swingTrajectoryParameters;
      double maxSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot();
      double minSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMinSwingHeightFromStanceFoot();
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();
      double customWaypointAngleThreshold = walkingControllerParameters.getSteppingParameters().getCustomWaypointAngleThreshold();

      soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotSide);
      oppositeSoleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotSide.getOppositeSide());
      oppositeSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide.getOppositeSide());
      currentStateProvider = new CurrentRigidBodyStateProvider(soleFrame);

      namePrefix += "FootSwing";

      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      activeTrajectoryType = new YoEnum<>(namePrefix + TrajectoryType.class.getSimpleName(), registry, TrajectoryType.class);

      swingHeight = new YoDouble(namePrefix + "Height", registry);
      swingDuration = new YoDouble(namePrefix + "Duration", registry);

      swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix, Footstep.maxNumberOfSwingWaypoints + 2, registry);

      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix,
                                                               minSwingHeightFromStanceFoot,
                                                               maxSwingHeightFromStanceFoot,
                                                               defaultSwingHeightFromStanceFoot,
                                                               customWaypointAngleThreshold,
                                                               registry,
                                                               controllerToolbox.getYoGraphicsListRegistry());
      double minDistanceToStance = walkingControllerParameters.getMinSwingTrajectoryClearanceFromStanceFoot();
      swingTrajectoryOptimizer.enableStanceCollisionAvoidance(robotSide, oppositeSoleZUpFrame, minDistanceToStance);

      lastFootstepPosition.setToNaN();
      finalPosition.setToNaN();
      finalOrientation.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void setShouldVisualize(boolean visualize)
   {
      swingTrajectoryOptimizer.setShouldVisualize(visualize);
   }

   /**
    * Resets the optimizer and the swing waypoints.
    */
   public void informDone()
   {
      saveFinalPositionAsLastFootstep();
      swingTrajectoryOptimizer.informDone();
   }

   public TrajectoryType getActiveTrajectoryType()
   {
      return activeTrajectoryType.getEnumValue();
   }

   public FrameVector3DReadOnly getFinalLinearVelocity()
   {
      return finalLinearVelocity;
   }

   /**
    * Updates the gradient descent module in the {@link TwoWaypointSwingGenerator}. If that optimizer
    * has already converged, does nothing.
    * 
    * @return true if it did something, false if it's converged
    */
   public boolean doOptimizationUpdate()
   {
      return swingTrajectoryOptimizer.doOptimizationUpdate();
   }

   public void saveFinalPositionAsLastFootstep()
   {
      this.lastFootstepPosition.setIncludingFrame(finalPosition);
      if (this.lastFootstepPosition.containsNaN())
         this.lastFootstepPosition.setToZero(soleFrame);
   }

   public void saveCurrentPositionAsLastFootstepPosition()
   {
      this.lastFootstepPosition.setToZero(soleFrame);
   }

   /**
    * Sets the footstep to be used for this calculator. Also passes in the waypoints to be used for the
    * swing trajectory.
    * 
    * @param footstep
    */
   public void setFootstep(Footstep footstep)
   {
      finalPosition.setIncludingFrame(footstep.getFootstepPose().getPosition());
      finalOrientation.setIncludingFrame(footstep.getFootstepPose().getOrientation());
      finalPosition.changeFrame(worldFrame);
      finalOrientation.changeFrame(worldFrame);
      finalPosition.addZ(swingTrajectoryParameters.getDesiredTouchdownHeightOffset());
      finalCoMVelocity.setToNaN();

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
      lastFootstepPosition.changeFrame(worldFrame);

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.CUSTOM)
      {
         setWaypointsFromCustomMidpoints(footstep.getCustomPositionWaypoints());
      }
      else if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         setWaypointsFromExternal(footstep.getSwingTrajectory());
      }
      else
      {
         setWaypointsFromStepPosition(footstep);
      }
   }

   /**
    * Invoke this setter after {@link #setFootstep(Footstep)} to register the center of mass velocity
    * predicted at the end of swing.
    * <p>
    * The x and y components of the CoM velocity are added to the desired swing final velocity. The
    * objective is to increase robustness to late touchdown.
    * </p>
    * 
    * @param finalCoMVelocity the predicted center of mass velocity at touchdown. Not modified.
    */
   public void setFinalCoMVelocity(FrameVector3DReadOnly finalCoMVelocity)
   {
      this.finalCoMVelocity.set(finalCoMVelocity);
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.set(swingDuration);
   }

   /**
    * Sets the initial conditions that the calculator uses to the calculate the swing trajectory to the
    * current state of the sole frame.
    */
   public void setInitialConditionsToCurrent()
   {
      currentStateProvider.getPosition(initialPosition);
      currentStateProvider.getLinearVelocity(initialLinearVelocity);
      currentStateProvider.getOrientation(initialOrientation);
      currentStateProvider.getAngularVelocity(initialAngularVelocity);

      if (swingTrajectoryParameters.ignoreSwingInitialAngularVelocityZ())
      {
         initialAngularVelocity.changeFrame(worldFrame);
         initialAngularVelocity.setZ(0.0);
      }
      initialLinearVelocity.clipToMaxLength(swingTrajectoryParameters.getMaxSwingInitialLinearVelocityMagnitude());
      initialAngularVelocity.clipToMaxLength(swingTrajectoryParameters.getMaxSwingInitialAngularVelocityMagnitude());
      stanceFootPosition.setToZero(oppositeSoleFrame);
   }

   /**
    * Computes the trajectory waypoints. If the foot is executing a custom waypoint trajectory provided
    * in the footstep (see Footstep#trajectoryType}, those are what are used. Otherwise, it
    * uses the {@link TwoWaypointSwingGenerator} to optimize the waypoint times and velocities. If
    * these waypoint positions have been provided using the custom waypoints interface in the
    * {@link Footstep} class ({see Footstep#customPositionWaypoints}), these are used, otherwise they
    * are computed using the default waypoint proportions or {see Footstep#customWaypointProportions}
    * if provided.
    *
    * @param initializeOptimizer set to true on the first initialization in the state, as it sets the
    *                            constraints for the {@link TwoWaypointSwingGenerator}.
    */
   public void initializeTrajectoryWaypoints(boolean initializeOptimizer)
   {
      finalLinearVelocity.setIncludingFrame(swingTrajectoryParameters.getDesiredTouchdownVelocity());
      finalAngularVelocity.setToZero(worldFrame);

      if (!finalCoMVelocity.containsNaN())
      {
         double injectionRatio = swingTrajectoryParameters.getFinalCoMVelocityInjectionRatio();
         finalLinearVelocity.setX(injectionRatio * finalCoMVelocity.getX());
         finalLinearVelocity.setY(injectionRatio * finalCoMVelocity.getY());
      }

      // append current pose as initial trajectory point
      swingTrajectory.clear(worldFrame);

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         setTrajectoryFromWaypoints();
      }
      else
      {
         setTrajectoryFromOptimizer(initializeOptimizer);
      }

      swingTrajectory.initialize();
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public int getNumberOfSwingWaypoints()
   {
      return swingWaypoints.size();
   }

   public FrameSE3TrajectoryPointBasics getSwingWaypoint(int index)
   {
      return swingWaypoints.get(index);
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

   private boolean checkStepUpOrDown(FramePoint3DReadOnly footstepPosition)
   {
      double zDifference = Math.abs(footstepPosition.getZ() - lastFootstepPosition.getZ());
      return zDifference > swingTrajectoryParameters.getMinHeightDifferenceForStepUpOrDown();
   }

   private void modifyFinalOrientationForTouchdown(FrameQuaternion finalOrientationToPack)
   {
      finalPosition.changeFrame(oppositeSoleZUpFrame);
      stanceFootPosition.changeFrame(oppositeSoleZUpFrame);
      double stepHeight = finalPosition.getZ() - stanceFootPosition.getZ();
      double initialFootstepPitch = finalOrientationToPack.getPitch();

      double footstepPitchModification;
      if (MathTools.intervalContains(stepHeight,
                                     swingTrajectoryParameters.getStepDownHeightForToeTouchdown(),
                                     swingTrajectoryParameters.getMaximumHeightForHeelTouchdown())
            && swingTrajectoryParameters.doHeelTouchdownIfPossible())
      { // not stepping down too far, and not stepping up too far, so do heel strike
         double stepLength = finalPosition.getX() - stanceFootPosition.getX();
         double heelTouchdownAngle = MathTools.clamp(-stepLength * swingTrajectoryParameters.getHeelTouchdownLengthRatio(),
                                                     -swingTrajectoryParameters.getHeelTouchdownAngle());
         // use the footstep pitch if its greater than the heel strike angle
         footstepPitchModification = Math.max(initialFootstepPitch, heelTouchdownAngle);
         // decrease the foot pitch modification if next step pitches down
         footstepPitchModification = Math.min(footstepPitchModification, heelTouchdownAngle + initialFootstepPitch);
         footstepPitchModification -= initialFootstepPitch;
      }
      else if (stepHeight < swingTrajectoryParameters.getStepDownHeightForToeTouchdown() && swingTrajectoryParameters.doToeTouchdownIfPossible())
      { // stepping down and do toe touchdown
         double toeTouchdownAngle = MathTools.clamp(-swingTrajectoryParameters.getToeTouchdownDepthRatio()
               * (stepHeight - swingTrajectoryParameters.getStepDownHeightForToeTouchdown()), swingTrajectoryParameters.getToeTouchdownAngle());
         footstepPitchModification = Math.max(toeTouchdownAngle, initialFootstepPitch);
         footstepPitchModification -= initialFootstepPitch;
      }
      else
      {
         footstepPitchModification = 0.0;
      }

      finalOrientationToPack.appendPitchRotation(footstepPitchModification);
   }

   private void setWaypointsFromStepPosition(Footstep footstep)
   {
      swingHeight.set(footstep.getSwingHeight());

      if (checkStepUpOrDown(finalPosition))
         activeTrajectoryType.set(TrajectoryType.OBSTACLE_CLEARANCE);

      RecyclingArrayList<MutableDouble> customWaypointProportions = footstep.getCustomWaypointProportions();
      if (customWaypointProportions.size() != 2)
      {
         if (!customWaypointProportions.isEmpty())
         {
            LogTools.warn("Ignoring custom waypoint proportions. Expected 2, got: " + customWaypointProportions.size());
         }

         List<DoubleProvider> waypointProportions = activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE
               ? swingTrajectoryParameters.getObstacleClearanceProportions()
               : swingTrajectoryParameters.getSwingWaypointProportions();
         this.waypointProportions[0] = waypointProportions.get(0).getValue();
         this.waypointProportions[1] = waypointProportions.get(1).getValue();
      }
      else
      {
         waypointProportions[0] = customWaypointProportions.get(0).getValue();
         waypointProportions[1] = customWaypointProportions.get(1).getValue();
      }
   }

   private void setWaypointsFromCustomMidpoints(List<FramePoint3D> positionWaypointsForSole)
   {
      for (int i = 0; i < positionWaypointsForSole.size(); i++)
         this.positionWaypointsForSole.add().setIncludingFrame(positionWaypointsForSole.get(i));
   }

   private void setWaypointsFromExternal(List<FrameSE3TrajectoryPoint> swingWaypoints)
   {
      for (int i = 0; i < swingWaypoints.size(); i++)
      {
         this.swingWaypoints.add().set(swingWaypoints.get(i));
      }
   }

   private void setTrajectoryFromWaypoints()
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

      boolean appendFootstepPose = !Precision.equals(swingWaypoints.getLast().getTime(), swingDuration.getDoubleValue());

      // append footstep pose if not provided in the waypoints
      if (appendFootstepPose)
      {
         modifyFinalOrientationForTouchdown(finalOrientation);
         swingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);
         swingTrajectory.appendOrientationWaypoint(swingDuration.getDoubleValue(), finalOrientation, finalAngularVelocity);
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
   }

   private void setTrajectoryFromOptimizer(boolean initializeOptimizer)
   {
      if (initializeOptimizer)
         initializeTrajectoryOptimizer();

      swingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);
      swingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, initialAngularVelocity);

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
         swingTrajectory.appendOrientationWaypoint(0.5 * swingDuration.getDoubleValue(), tmpOrientation, tmpVector);
      }

      modifyFinalOrientationForTouchdown(finalOrientation);
      swingTrajectoryOptimizer.getFinalVelocity(finalLinearVelocity);
      swingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);
      swingTrajectory.appendOrientationWaypoint(swingDuration.getDoubleValue(), finalOrientation, finalAngularVelocity);
   }
}
