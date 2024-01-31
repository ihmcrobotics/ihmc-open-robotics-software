package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
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
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class SwingTrajectoryCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TwoWaypointSwingGenerator swingTrajectoryOptimizer;

   private final MovingReferenceFrame pelvisFrame;
   private final MovingReferenceFrame soleFrame;
   private final ReferenceFrame oppositeSoleFrame;
   private final ReferenceFrame oppositeSoleZUpFrame;

   private final YoEnum<TrajectoryType> activeTrajectoryType;

   private final RecyclingArrayList<FramePoint3D> positionWaypointsForSole = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints, FramePoint3D.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingWaypoints = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints,
                                                                                                       FrameSE3TrajectoryPoint.class);

   private final YoDouble swingDuration;
   private final YoDouble swingHeight;

   private final YoSwingTrajectoryParameters swingTrajectoryParameters;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();
   private final FrameVector3D pelvisVelocity = new FrameVector3D();
   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();

   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();
   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();

   private final FramePoint3D stanceFootPosition = new FramePoint3D();

   private final FrameQuaternion tmpOrientation = new FrameQuaternion();
   private final FrameVector3D tmpVector = new FrameVector3D();

   private final FixedFramePoint3DBasics lastFootstepPosition = new FramePoint3D(worldFrame);

   private final MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   private final double[] waypointProportions = new double[2];

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   private final String namePrefix;

   public SwingTrajectoryCalculator(String namePrefix,
                                    RobotSide robotSide,
                                    HighLevelHumanoidControllerToolbox controllerToolbox,
                                    WalkingControllerParameters walkingControllerParameters,
                                    YoSwingTrajectoryParameters swingTrajectoryParameters,
                                    YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.swingTrajectoryParameters = swingTrajectoryParameters;
      double maxSwingHeightFromStanceFoot = walkingControllerParameters.getSwingTrajectoryParameters().getMaxSwingHeight();
      double minSwingHeightFromStanceFoot = walkingControllerParameters.getSwingTrajectoryParameters().getMinSwingHeight();
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSwingTrajectoryParameters().getDefaultSwingHeight();
      double customWaypointAngleThreshold = walkingControllerParameters.getSwingTrajectoryParameters().getCustomWaypointAngleThreshold();
      double firstWaypointHeightFactorForSteppingUp = walkingControllerParameters.getSwingTrajectoryParameters().getFirstWaypointHeightFactorForSteppingUp();
      double secondWaypointHeightFactorForSteppingDown = walkingControllerParameters.getSwingTrajectoryParameters()
                                                                                    .getSecondWaypointHeightFactorForSteppingDown();

      pelvisFrame = controllerToolbox.getReferenceFrames().getPelvisFrame();
      soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotSide);
      oppositeSoleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotSide.getOppositeSide());
      oppositeSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide.getOppositeSide());

      namePrefix += "FootSwing";

      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      activeTrajectoryType = new YoEnum<>(namePrefix + TrajectoryType.class.getSimpleName(), registry, TrajectoryType.class);

      swingHeight = new YoDouble(namePrefix + "Height", registry);
      swingDuration = new YoDouble(namePrefix + "Duration", registry);

      swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix, Footstep.maxNumberOfSwingWaypoints + 2, registry);

      // Setting the number of waypoints to two, since we only use the swing trajectory generator when there's two waypoints
      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix,
                                                               minSwingHeightFromStanceFoot,
                                                               maxSwingHeightFromStanceFoot,
                                                               defaultSwingHeightFromStanceFoot,
                                                               customWaypointAngleThreshold,
                                                               registry,
                                                               controllerToolbox.getYoGraphicsListRegistry());
      swingTrajectoryOptimizer.setObstacleClearanceWaypointHeightFactors(firstWaypointHeightFactorForSteppingUp, secondWaypointHeightFactorForSteppingDown);
      double minDistanceToStance = walkingControllerParameters.getMinSwingTrajectoryClearanceFromStanceFoot();
      swingTrajectoryOptimizer.enableStanceCollisionAvoidance(robotSide, oppositeSoleZUpFrame, minDistanceToStance);

      if (walkingControllerParameters.getSwingTrajectoryParameters().useInitialToeHeight())
      {
         YoPoint3D toePoint = new YoPoint3D(namePrefix + "InitialToePosition", registry);
         swingTrajectoryOptimizer.setInitialGroundHeightProvider(() ->
         {
            toePoint.set(walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0, 0.0, 0.0);
            initialOrientation.transform(toePoint);
            toePoint.add(initialPosition);
            return toePoint.getZ();
         });
      }

      if (walkingControllerParameters.getSwingTrajectoryParameters().useFinalHeelHeight())
      {
         YoPoint3D heelPoint = new YoPoint3D(namePrefix + "FinalHeelPosition", registry);
         swingTrajectoryOptimizer.setFinalGroundHeightProvider(() ->
         {
            heelPoint.set(-walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0, 0.0, 0.0);
            finalOrientation.transform(heelPoint);
            heelPoint.add(finalPosition);
            return heelPoint.getZ();
         });
      }

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
      this.lastFootstepPosition.setMatchingFrame(finalPosition);
      if (this.lastFootstepPosition.containsNaN())
         saveCurrentPositionAsLastFootstepPosition();
   }

   public void saveCurrentPositionAsLastFootstepPosition()
   {
      this.lastFootstepPosition.setFromReferenceFrame(soleFrame);
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
      finalLinearVelocity.set(swingTrajectoryParameters.getDesiredTouchdownVelocity());
      finalLinearVelocity.scale(1.0 / Math.min(swingDuration.getDoubleValue(), 1.0));
      finalAngularVelocity.setToZero(worldFrame);

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
    * Invoke this setter after {@link #setFootstep(Footstep)} to register what the final velocity is
    * predicted at the end of swing, mainly to account for non-zero com velocity. The objective is to
    * increase robustness to late touchdown.
    *
    * @param finalLinearVelocity the final velocity at touchdown to use. Not modified.
    */
   public void setFinalLinearVelocity(FrameVector3DReadOnly finalLinearVelocity)
   {
      this.finalLinearVelocity.set(finalLinearVelocity);
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.set(swingDuration);
   }

   /**
    * Sets the initial conditions that the calculator uses to calculate the swing trajectory to the
    * current state of the sole frame.
    */
   public void setInitialConditionsToCurrent()
   {
      initialPosition.setFromReferenceFrame(soleFrame);
      initialLinearVelocity.setMatchingFrame(soleFrame.getTwistOfFrame().getLinearPart());
      initialOrientation.setFromReferenceFrame(soleFrame);
      initialAngularVelocity.setMatchingFrame(soleFrame.getTwistOfFrame().getAngularPart());

      pelvisVelocity.setIncludingFrame(pelvisFrame.getTwistOfFrame().getLinearPart());
      pelvisVelocity.changeFrame(worldFrame);

      initialLinearVelocity.scaleAdd(swingTrajectoryParameters.getPelvisVelocityInjectionRatio(), pelvisVelocity, initialLinearVelocity);

      if (swingTrajectoryParameters.ignoreSwingInitialAngularVelocityZ())
      {
         initialAngularVelocity.changeFrame(worldFrame);
         initialAngularVelocity.setZ(0.0);
      }
      initialLinearVelocity.changeFrame(worldFrame);
      double liftOffVelocity = swingTrajectoryParameters.getMinLiftOffVerticalVelocity() / (Math.min(1.0, swingDuration.getDoubleValue()));
      initialLinearVelocity.setZ(Math.max(liftOffVelocity, initialLinearVelocity.getZ()));

      initialLinearVelocity.clipToMaxNorm(swingTrajectoryParameters.getMaxSwingInitialLinearVelocityMagnitude());
      initialAngularVelocity.clipToMaxNorm(swingTrajectoryParameters.getMaxSwingInitialAngularVelocityMagnitude());

      stanceFootPosition.setToZero(oppositeSoleFrame);
   }

   /**
    * Computes the trajectory waypoints. If the foot is executing a custom waypoint trajectory provided
    * in the footstep (see Footstep#trajectoryType}, those are what are used. Otherwise, it uses the
    * {@link TwoWaypointSwingGenerator} to optimize the waypoint times and velocities. If these
    * waypoint positions have been provided using the custom waypoints interface in the
    * {@link Footstep} class ({see Footstep#customPositionWaypoints}), these are used, otherwise they
    * are computed using the default waypoint proportions or {see Footstep#customWaypointProportions}
    * if provided.
    *
    * @param initializeOptimizer set to true on the first initialization in the state, as it sets the
    *                            constraints for the {@link TwoWaypointSwingGenerator}.
    */
   public void initializeTrajectoryWaypoints(boolean initializeOptimizer)
   {
      finalAngularVelocity.setToZero(worldFrame);

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
      swingTrajectoryOptimizer.setSwingHeight(selectSwingHeight());
      swingTrajectoryOptimizer.setStanceFootPosition(stanceFootPosition);
      swingTrajectoryOptimizer.setWaypointProportions(waypointProportions);
      swingTrajectoryOptimizer.initialize();
   }

   private double selectSwingHeight()
   {
      if (activeTrajectoryType.getValue() != TrajectoryType.OBSTACLE_CLEARANCE)
         return swingHeight.getValue();
      if (swingTrajectoryOptimizer.isSwingHeightValid(swingHeight.getValue()))
         return swingHeight.getValue();
      return isSteppingUp(finalPosition) ? swingTrajectoryParameters.getDefaultSwingStepUpHeight() : swingTrajectoryParameters.getDefaultSwingStepDownHeight();
   }

   private boolean isSteppingUpOrDown(FramePoint3DReadOnly footstepPosition)
   {
      double zDifference = Math.abs(footstepPosition.getZ() - lastFootstepPosition.getZ());
      return zDifference > swingTrajectoryParameters.getMinHeightDifferenceForStepUpOrDown();
   }

   private boolean isSteppingUp(FramePoint3DReadOnly footstepPosition)
   {
      return footstepPosition.getZ() - lastFootstepPosition.getZ() > swingTrajectoryParameters.getMinHeightDifferenceForStepUpOrDown();
   }

   private void setWaypointsFromStepPosition(Footstep footstep)
   {
      swingHeight.set(footstep.getSwingHeight());

      if (lastFootstepPosition.containsNaN())
         saveCurrentPositionAsLastFootstepPosition();

      if (isSteppingUpOrDown(finalPosition))
         activeTrajectoryType.set(TrajectoryType.OBSTACLE_CLEARANCE);

      RecyclingArrayList<MutableDouble> customWaypointProportions = footstep.getCustomWaypointProportions();
      if (customWaypointProportions.size() != 2)
      {
         if (!customWaypointProportions.isEmpty())
         {
            LogTools.warn("Ignoring custom waypoint proportions. Expected 2, got: " + customWaypointProportions.size());
         }

         List<DoubleProvider> waypointProportions;

         if (activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE)
         {
            waypointProportions = isSteppingUp(finalPosition) ? swingTrajectoryParameters.getSwingStepUpWaypointProportions()
                                                              : swingTrajectoryParameters.getSwingStepDownWaypointProportions();
         }
         else
         {
            waypointProportions = swingTrajectoryParameters.getSwingWaypointProportions();
         }

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
         swingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);
         swingTrajectory.appendOrientationWaypoint(swingDuration.getDoubleValue(), finalOrientation, finalAngularVelocity);
      }
      else
      {
         // In this case our swing trajectory contains the touchdown so we should use those values to be continuous.
         FrameSE3TrajectoryPoint lastPoint = swingWaypoints.getLast();
         finalPosition.setIncludingFrame(lastPoint.getPosition());
         finalLinearVelocity.setIncludingFrame(lastPoint.getLinearVelocity());
         finalOrientation.setIncludingFrame(lastPoint.getOrientation());
         finalAngularVelocity.set(lastPoint.getAngularVelocity());
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

      boolean isSteppingDown = swingTrajectoryOptimizer.isSteppingDown();
      if (swingTrajectoryParameters.addFootPitchToAvoidHeelStrikeWhenSteppingDown() && activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE && isSteppingDown)
      {
         tmpOrientation.setToZero(worldFrame);
         tmpVector.setToZero(worldFrame);
         double remainingPitch = swingTrajectoryParameters.getFootPitchAngleToAvoidHeelStrike() - initialOrientation.getPitch();
         if (remainingPitch > 0.0)
         {
            // compute the linear velocity to go from A to B, and then zero out the pitch velocity.
            tmpOrientation.difference(initialOrientation, finalOrientation);
            tmpOrientation.getRotationVector(tmpVector);
            tmpVector.scale(1.0 / swingDuration.getDoubleValue());
            tmpVector.setY(0.0);

            tmpOrientation.interpolate(initialOrientation, finalOrientation, swingTrajectoryParameters.getFractionOfSwingToPitchFootDown());
            tmpOrientation.setYawPitchRoll(tmpOrientation.getYaw(), swingTrajectoryParameters.getFootPitchAngleToAvoidHeelStrike(), tmpOrientation.getRoll());

            swingTrajectory.appendOrientationWaypoint(swingTrajectoryParameters.getFractionOfSwingToPitchFootDown() * swingDuration.getDoubleValue(), tmpOrientation, tmpVector);
         }
      }
      // make the foot orientation better for avoidance
      else if (swingTrajectoryParameters.addOrientationMidpointForObstacleClearance() && activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE)
      {
         tmpOrientation.setToZero(worldFrame);
         tmpVector.setToZero(worldFrame);
         tmpOrientation.interpolate(initialOrientation, finalOrientation, swingTrajectoryParameters.getMidpointOrientationInterpolationForObstacleClearance());
         swingTrajectory.appendOrientationWaypoint(0.5 * swingDuration.getDoubleValue(), tmpOrientation, tmpVector);
      }

      swingTrajectoryOptimizer.getFinalVelocity(finalLinearVelocity);
      swingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);
      swingTrajectory.appendOrientationWaypoint(swingDuration.getDoubleValue(), finalOrientation, finalAngularVelocity);
   }

   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(namePrefix + getClass().getSimpleName());
      group.addChild(swingTrajectoryOptimizer.getSCS2YoGraphics());
      return group;
   }
}
