package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.YoCounter;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class SwingOverPlanarRegionsTrajectoryExpander
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final double[] swingWaypointProportions;

   private final TwoWaypointSwingGenerator twoWaypointSwingGenerator;
   private final ConvexPolygon2D footPlanePolygon;

   private final YoBoolean doInitialFastApproximation;
   private final YoInteger numberOfCheckpoints;
   private final YoCounter numberOfTriesCounter;
   private final YoDouble minimumClearance;
   private final YoDouble incrementalAdjustmentDistance;
   private final YoDouble maximumAdjustmentDistance;
   private final YoDouble minimumHeightAboveFloorForCollision;
   private final YoDouble minimumFractionOfSwingForCollisionCheck;
   private final YoDouble maximumFractionOfSwingForCollisionCheck;
   private final YoEnum<SwingOverPlanarRegionsCollisionType> mostSevereCollisionType;
   private final YoEnum<SwingOverPlanarRegionsStatus> status;

   private final YoBoolean wereWaypointsAdjusted;
   private final YoFramePoint3D trajectoryPosition;
   private final PoseReferenceFrame solePoseReferenceFrame;
   private final RecyclingArrayList<FramePoint3D> originalWaypoints;
   private final RecyclingArrayList<FramePoint3D> adjustedWaypoints;
   private final double minimumSwingHeight;
   private final double maximumSwingHeight;
   private final double collisionSphereRadius;

   private final Map<SwingOverPlanarRegionsCollisionType, FramePoint3D> closestPolygonPointMap;
   private final FramePoint3D midGroundPoint;
   private final Vector3D waypointAdjustmentDirection;
   private final Plane3D swingTrajectoryPlane;
   private final Plane3D swingFloorPlane;
   private final PoseReferenceFrame startOfSwingReferenceFrame = new PoseReferenceFrame("startOfSwingFrame", worldFrame);
   private final AxisAngle axisAngle;
   private final RigidBodyTransform rigidBodyTransform;

   private final Vector3D tempPlaneNormal = new Vector3D();
   private final Vector3D stepDirectionVector = new Vector3D();

   // Boilerplate variables
   private final FrameVector3D initialVelocity;
   private final FrameVector3D touchdownVelocity;
   private final FramePoint3D swingStartPosition;
   private final FramePoint3D swingEndPosition;
   private final FramePoint3D stanceFootPosition;
   private final FramePoint3D collisionRelativeToStart;
   private final FramePoint3D stepRelativeToStart;

   // Visualization
   private Optional<Runnable> visualizer;

   public enum SwingOverPlanarRegionsCollisionType
   {
      NO_INTERSECTION, TOO_CLOSE_TO_IGNORE_PLANE, OUTSIDE_TRAJECTORY, COLLISION_INSIDE_TRAJECTORY, COLLISION_BETWEEN_FEET
   }

   public enum SwingOverPlanarRegionsStatus
   {
      INITIALIZED, FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE, SEARCHING_FOR_SOLUTION, SOLUTION_FOUND,
   }

   public SwingOverPlanarRegionsTrajectoryExpander(WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry,
                                                   YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = "trajectoryExpander";
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      twoWaypointSwingGenerator = new TwoWaypointSwingGenerator(namePrefix, steppingParameters.getMinSwingHeightFromStanceFoot(),
                                                                steppingParameters.getMaxSwingHeightFromStanceFoot(),
                                                                steppingParameters.getMinSwingHeightFromStanceFoot(), parentRegistry, graphicsListRegistry);
      minimumSwingHeight = steppingParameters.getMinSwingHeightFromStanceFoot();
      maximumSwingHeight = steppingParameters.getMaxSwingHeightFromStanceFoot();
      collisionSphereRadius = steppingParameters.getActualFootLength() / 2.0;

      swingWaypointProportions = walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions();

      footPlanePolygon = new ConvexPolygon2D();
      footPlanePolygon.addVertex(steppingParameters.getFootForwardOffset(), 0.5 * steppingParameters.getToeWidth());
      footPlanePolygon.addVertex(steppingParameters.getFootForwardOffset(), -0.5 * steppingParameters.getToeWidth());
      footPlanePolygon.addVertex(-steppingParameters.getFootBackwardOffset(), 0.5 * steppingParameters.getFootWidth());
      footPlanePolygon.addVertex(-steppingParameters.getFootBackwardOffset(), -0.5 * steppingParameters.getFootWidth());
      footPlanePolygon.update();

      doInitialFastApproximation = new YoBoolean(namePrefix + "DoInitialFastApproximation", parentRegistry);
      numberOfCheckpoints = new YoInteger(namePrefix + "NumberOfCheckpoints", parentRegistry);
      numberOfTriesCounter = new YoCounter(namePrefix + "NumberOfTriesCounter", parentRegistry);
      minimumClearance = new YoDouble(namePrefix + "MinimumClearance", parentRegistry);
      minimumFractionOfSwingForCollisionCheck = new YoDouble(namePrefix + "MinimumFractionOfSwingForCollisionCheck", parentRegistry);
      maximumFractionOfSwingForCollisionCheck = new YoDouble(namePrefix + "MaximumFractionOfSwingForCollisionCheck", parentRegistry);
      minimumHeightAboveFloorForCollision = new YoDouble(namePrefix + "MinimumHeightAboveFloorForCollision", parentRegistry);
      incrementalAdjustmentDistance = new YoDouble(namePrefix + "IncrementalAdjustmentDistance", parentRegistry);
      maximumAdjustmentDistance = new YoDouble(namePrefix + "MaximumAdjustmentDistance", parentRegistry);
      wereWaypointsAdjusted = new YoBoolean(namePrefix + "WereWaypointsAdjusted", parentRegistry);
      status = new YoEnum<>(namePrefix + "Status", parentRegistry, SwingOverPlanarRegionsStatus.class);
      mostSevereCollisionType = new YoEnum<>(namePrefix + "CollisionType", parentRegistry, SwingOverPlanarRegionsCollisionType.class);

      trajectoryPosition = new YoFramePoint3D(namePrefix + "TrajectoryPosition", worldFrame, parentRegistry);
      solePoseReferenceFrame = new PoseReferenceFrame(namePrefix + "SolePoseReferenceFrame", worldFrame);
      originalWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      originalWaypoints.add();
      originalWaypoints.add();
      adjustedWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      adjustedWaypoints.add();
      adjustedWaypoints.add();

      closestPolygonPointMap = new HashMap<>();
      for (SwingOverPlanarRegionsCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsCollisionType.values())
      {
         closestPolygonPointMap.put(swingOverPlanarRegionsTrajectoryCollisionType, new FramePoint3D());
      }
      midGroundPoint = new FramePoint3D();
      waypointAdjustmentDirection = new Vector3D();
      swingTrajectoryPlane = new Plane3D();
      swingFloorPlane = new Plane3D();
      axisAngle = new AxisAngle();
      rigidBodyTransform = new RigidBodyTransform();

      initialVelocity = new FrameVector3D();
      touchdownVelocity = new FrameVector3D();
      touchdownVelocity.setZ(walkingControllerParameters.getSwingTrajectoryParameters().getDesiredTouchdownVelocity());
      swingStartPosition = new FramePoint3D();
      swingEndPosition = new FramePoint3D();
      stanceFootPosition = new FramePoint3D();
      collisionRelativeToStart = new FramePoint3D();
      stepRelativeToStart = new FramePoint3D();

      visualizer = Optional.empty();

      // Set default values
      doInitialFastApproximation.set(true);
      minimumFractionOfSwingForCollisionCheck.set(0.0);
      maximumFractionOfSwingForCollisionCheck.set(1.0);
      minimumHeightAboveFloorForCollision.set(0.02);
      numberOfCheckpoints.set(100);
      numberOfTriesCounter.setMaxCount(50);
      minimumClearance.set(0.04);
      incrementalAdjustmentDistance.set(0.03);
      maximumAdjustmentDistance.set(maximumSwingHeight - minimumSwingHeight);
   }

   public void setDoInitialFastApproximation(boolean doInitialFastApproximation)
   {
      this.doInitialFastApproximation.set(doInitialFastApproximation);
   }

   public void setNumberOfCheckpoints(int numberOfCheckpoints)
   {
      this.numberOfCheckpoints.set(numberOfCheckpoints);
   }

   public void setMaximumNumberOfTries(int maximumNumberOfTries)
   {
      this.numberOfTriesCounter.setMaxCount(maximumNumberOfTries);
   }

   public void setMinimumSwingFootClearance(double minimumSwingFootClearance)
   {
      minimumClearance.set(minimumSwingFootClearance);
   }

   public void setIncrementalAdjustmentDistance(double incrementalAdjustmentDistance)
   {
      this.incrementalAdjustmentDistance.set(incrementalAdjustmentDistance);
   }

   public void setMaximumAdjustmentDistance(double maximumAdjustmentDistance)
   {
      this.maximumAdjustmentDistance.set(maximumAdjustmentDistance);
   }

   public void setMinimumFractionOfSwingForCollisionCheck(double minimumFractionOfSwingForCollisionCheck)
   {
      this.minimumFractionOfSwingForCollisionCheck.set(minimumFractionOfSwingForCollisionCheck);
   }

   public void setMaximumFractionOfSwingForCollisionCheck(double maximumFractionOfSwingForCollisionCheck)
   {
      this.maximumFractionOfSwingForCollisionCheck.set(maximumFractionOfSwingForCollisionCheck);
   }

   public void setMinimumHeightAboveFloorForCollision(double heightAboveFloorForCollision)
   {
      this.minimumHeightAboveFloorForCollision.set(heightAboveFloorForCollision);
   }

   private final RigidBodyTransform transformToStart = new RigidBodyTransform();
   private final RigidBodyTransform transformFromStart = new RigidBodyTransform();
   private final RigidBodyTransform transformToEnd = new RigidBodyTransform();
   private final RigidBodyTransform transformFromEnd = new RigidBodyTransform();

   public double expandTrajectoryOverPlanarRegions(FramePose3DReadOnly stanceFootPose, FramePose3DReadOnly swingStartPose, FramePose3DReadOnly swingEndPose,
                                                   PlanarRegionsList planarRegionsList)
   {
      stanceFootPosition.setMatchingFrame(stanceFootPose.getPosition());
      twoWaypointSwingGenerator.setStanceFootPosition(stanceFootPosition);

      swingStartPosition.setMatchingFrame(swingStartPose.getPosition());
      twoWaypointSwingGenerator.setInitialConditions(swingStartPosition, initialVelocity);

      swingEndPosition.setMatchingFrame(swingEndPose.getPosition());
      twoWaypointSwingGenerator.setFinalConditions(swingEndPosition, touchdownVelocity);
      twoWaypointSwingGenerator.setStepTime(1.0);

      swingStartPose.get(transformToStart);
      swingEndPose.get(transformToEnd);
      transformToStart.inverseTransform(transformFromStart);
      transformToEnd.inverseTransform(transformFromEnd);

      initializeSwingWaypoints();

      adjustSwingEndIfCoincidentWithSwingStart();

      startOfSwingReferenceFrame.setPoseAndUpdate(swingStartPose);
      stepRelativeToStart.setIncludingFrame(swingEndPosition);
      stepRelativeToStart.changeFrame(startOfSwingReferenceFrame);

      midGroundPoint.interpolate(swingStartPosition, swingEndPosition, 0.5);
      swingTrajectoryPlane.set(swingStartPosition, adjustedWaypoints.get(0), swingEndPosition);

      axisAngle.set(swingTrajectoryPlane.getNormal(), Math.PI / 2.0);
      rigidBodyTransform.setRotation(axisAngle);
      tempPlaneNormal.sub(swingStartPosition, swingEndPosition);
      rigidBodyTransform.transform(tempPlaneNormal);
      tempPlaneNormal.normalize();
      swingFloorPlane.set(swingStartPosition, tempPlaneNormal);

      wereWaypointsAdjusted.set(false);

      double filterDistance = maximumSwingHeight + collisionSphereRadius + 2.0 * minimumClearance.getDoubleValue();
      List<PlanarRegion> filteredRegions = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCapsule(swingStartPosition, swingEndPosition, filterDistance, planarRegionsList.getPlanarRegionsAsList());

      status.set(SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION);
      numberOfTriesCounter.resetCount();

      while (doInitialFastApproximation.getBooleanValue() && status.getEnumValue().equals(SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION)
            && !numberOfTriesCounter.maxCountReached())
      {
         for (SwingOverPlanarRegionsCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsCollisionType.values())
         {
            closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                                  .setIncludingFrame(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         }
         mostSevereCollisionType.set(SwingOverPlanarRegionsCollisionType.NO_INTERSECTION);

         status.set(checkAndAdjustForCollisions(filteredRegions, this::getFastFractionThroughTrajectoryForCollision));
         updateVisualizer();
         numberOfTriesCounter.countOne();
      }

      status.set(SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION);
      while (status.getEnumValue().equals(SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION) && !numberOfTriesCounter.maxCountReached())
      {
         for (SwingOverPlanarRegionsCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsCollisionType.values())
         {
            closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                                  .setIncludingFrame(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         }
         mostSevereCollisionType.set(SwingOverPlanarRegionsCollisionType.NO_INTERSECTION);

         twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
         twoWaypointSwingGenerator.initialize();

         status.set(checkAndAdjustForCollisions(filteredRegions, this::getFractionThroughTrajectoryForCollision));
         updateVisualizer();
         numberOfTriesCounter.countOne();
      }

      return twoWaypointSwingGenerator.computeAndGetMaxSpeed();
   }

   // TODO figure out a better solution for coincident points and replace this
   private void adjustSwingEndIfCoincidentWithSwingStart()
   {
      if (swingStartPosition.distance(swingEndPosition) < 1e-8)
         swingEndPosition.add(1e-4, 1e-4, 1e-4);
   }

   private void initializeSwingWaypoints()
   {
      FramePoint3DBasics firstBaseWaypoint = originalWaypoints.get(0);
      FramePoint3DBasics secondBaseWaypoint = originalWaypoints.get(1);

      firstBaseWaypoint.interpolate(swingStartPosition, swingEndPosition, swingWaypointProportions[0]);
      secondBaseWaypoint.interpolate(swingStartPosition, swingEndPosition, swingWaypointProportions[1]);
      double firstWaypointHeight = Math.max(swingStartPosition.getZ(), firstBaseWaypoint.getZ()) + minimumSwingHeight;
      double secondWaypointHeight = Math.max(swingEndPosition.getZ(), secondBaseWaypoint.getZ()) + minimumSwingHeight;
      firstBaseWaypoint.setZ(firstWaypointHeight);
      secondBaseWaypoint.setZ(secondWaypointHeight);

      adjustedWaypoints.get(0).set(firstBaseWaypoint);
      adjustedWaypoints.get(1).set(secondBaseWaypoint);
   }

   private SwingOverPlanarRegionsStatus checkAndAdjustForCollisions(List<PlanarRegion> planarRegionsList,
                                                                    Function<List<PlanarRegion>, Double> fractionThroughCollisionFinder)
   {
      double maxAdjustmentDistanceSquared = MathTools.square(maximumAdjustmentDistance.getDoubleValue());

      FramePoint3DBasics originalFirstWaypoint = originalWaypoints.get(0);
      FramePoint3DBasics originalSecondWaypoint = originalWaypoints.get(1);
      FramePoint3DBasics adjustedFirstWaypoint = adjustedWaypoints.get(0);
      FramePoint3DBasics adjustedSecondWaypoint = adjustedWaypoints.get(1);

      double fractionForCollision = fractionThroughCollisionFinder.apply(planarRegionsList);

      if (fractionForCollision >= 0.0)
      {
         wereWaypointsAdjusted.set(true);
         computeWaypointAdjustmentDirection(fractionForCollision);

         // TODO scale this with distance to collision?
         double firstWaypointAdjustment = (1.0 - fractionForCollision) * incrementalAdjustmentDistance.getDoubleValue();
         double secondWaypointAdjustment = fractionForCollision * incrementalAdjustmentDistance.getDoubleValue();
         adjustedFirstWaypoint.scaleAdd(firstWaypointAdjustment, waypointAdjustmentDirection, adjustedFirstWaypoint);
         adjustedSecondWaypoint.scaleAdd(secondWaypointAdjustment, waypointAdjustmentDirection, adjustedSecondWaypoint);

         if (adjustedFirstWaypoint.distanceSquared(originalFirstWaypoint) > maxAdjustmentDistanceSquared
               || adjustedSecondWaypoint.distanceSquared(originalSecondWaypoint) > maxAdjustmentDistanceSquared)
         {
            return SwingOverPlanarRegionsStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE;
         }

         return SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION;
      }

      return SwingOverPlanarRegionsStatus.SOLUTION_FOUND;
   }

   private double getFastFractionThroughTrajectoryForCollision(List<PlanarRegion> planarRegions)
   {
      double firstSegmentLength = swingStartPosition.distance(adjustedWaypoints.get(0));
      double secondSegmentLength = adjustedWaypoints.get(0).distance(adjustedWaypoints.get(1));
      double thirdSegmentLength = adjustedWaypoints.get(1).distance(swingEndPosition);
      double totalLength = firstSegmentLength + secondSegmentLength + thirdSegmentLength;

      double fractionThroughSegmentForCollision = checkLineSegmentForCollision(swingStartPosition, adjustedWaypoints.get(0), planarRegions);
      if (fractionThroughSegmentForCollision >= 0.0)
      {
         return fractionThroughSegmentForCollision * firstSegmentLength / totalLength;
      }

      fractionThroughSegmentForCollision = checkLineSegmentForCollision(adjustedWaypoints.get(0), adjustedWaypoints.get(1), planarRegions);
      if (fractionThroughSegmentForCollision >= 0.0)
      {
         return (fractionThroughSegmentForCollision * secondSegmentLength + firstSegmentLength) / totalLength;
      }

      fractionThroughSegmentForCollision = checkLineSegmentForCollision(adjustedWaypoints.get(1), swingEndPosition, planarRegions);
      if (fractionThroughSegmentForCollision >= 0.0)
      {
         return (fractionThroughSegmentForCollision * thirdSegmentLength + secondSegmentLength + firstSegmentLength) / totalLength;
      }

      return -1.0;
   }

   /**
    * Returns the fraction through the segment that the collision occurs at.
    */
   private double checkLineSegmentForCollision(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint, List<PlanarRegion> planarRegions)
   {
      double avoidanceDistance = collisionSphereRadius + minimumClearance.getDoubleValue();

      for (PlanarRegion planarRegion : planarRegions)
      {
         Point3D startInLocal = new Point3D(firstEndpoint);
         Point3D endInLocal = new Point3D(secondEndpoint);
         planarRegion.transformFromWorldToLocal(startInLocal);
         planarRegion.transformFromWorldToLocal(endInLocal);

         Point3D closestPointOnSegment = new Point3D();
         Point3D closestPointInRegion = new Point3D();

         double distance = PlanarRegionTools
               .getDistanceFromLineSegment3DToPlanarRegion(startInLocal, endInLocal, planarRegion, closestPointOnSegment, closestPointInRegion);

         // FIXME double check the frame of the collision point
         updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.NO_INTERSECTION, closestPointInRegion);

         if (distance < avoidanceDistance)
         {
            Point3D closestPointOnRegionInWorld = new Point3D(closestPointInRegion);
            planarRegion.transformFromLocalToWorld(closestPointOnRegionInWorld);

            updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.TOO_CLOSE_TO_IGNORE_PLANE, closestPointInRegion);

            if (!checkIfCollidingWithFloorPlane(closestPointOnRegionInWorld))
            {
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.COLLISION_INSIDE_TRAJECTORY, closestPointInRegion);

               return closestPointOnSegment.distance(startInLocal) / endInLocal.distance(startInLocal);
            }
         }
      }

      return -1.0;
   }

   private double getFractionThroughTrajectoryForCollision(List<PlanarRegion> planarRegions)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();

      double stepAmount = 1.0 / numberOfCheckpoints.getIntegerValue();
      double avoidanceDistance = collisionSphereRadius + minimumClearance.getDoubleValue();
      double avoidanceDistanceSquared = MathTools.square(avoidanceDistance);
      boolean collisionIsOnRising = true;
      for (double fraction = minimumFractionOfSwingForCollisionCheck.getDoubleValue();
           fraction <= maximumFractionOfSwingForCollisionCheck.getDoubleValue(); fraction += stepAmount)
      {
         twoWaypointSwingGenerator.compute(fraction);
         FramePoint3D frameTupleUnsafe = new FramePoint3D(trajectoryPosition);
         twoWaypointSwingGenerator.getPosition(frameTupleUnsafe);
         trajectoryPosition.set(frameTupleUnsafe);
         solePoseReferenceFrame.setPositionAndUpdate(trajectoryPosition);

         twoWaypointSwingGenerator.getWaypointTime(0);
         if (collisionIsOnRising && fraction > twoWaypointSwingGenerator.getWaypointTime(0))
            collisionIsOnRising = false;

         // there's no point in checking if I haven't picked my foot up off the ground enough
         // FIXME easy edge case that fails: there's something right in front of the toe that needs to be avoided.
         boolean shouldSkipCheck = areWeTooEarlyInSwingToLookForCollisions(collisionIsOnRising, avoidanceDistance);
         if (shouldSkipCheck)
            continue;

         for (PlanarRegion planarRegion : planarRegions)
         {
            Point3DReadOnly closestPointOnRegion = PlanarRegionTools.closestPointOnPlane(trajectoryPosition, planarRegion);
            updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.NO_INTERSECTION, closestPointOnRegion);

            if (closestPointOnRegion == null)
               continue;

            double distanceToClosestPoint = closestPointOnRegion.distanceSquared(trajectoryPosition);

            if (distanceToClosestPoint < avoidanceDistanceSquared)
            {
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.TOO_CLOSE_TO_IGNORE_PLANE, closestPointOnRegion);

               if (!checkIfCollidingWithFloorPlane(closestPointOnRegion))
               {
                  updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.OUTSIDE_TRAJECTORY, closestPointOnRegion);

                  boolean isCollisionInsideTheTrajectory =
                        midGroundPoint.distanceSquared(closestPointOnRegion) < midGroundPoint.distanceSquared(trajectoryPosition);

                  if (isCollisionInsideTheTrajectory)
                  {
                     updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.COLLISION_INSIDE_TRAJECTORY, closestPointOnRegion);
                     return fraction;
                  }

                  collisionRelativeToStart.setIncludingFrame(worldFrame, closestPointOnRegion);
                  collisionRelativeToStart.changeFrame(startOfSwingReferenceFrame);

                  double toePoint = collisionSphereRadius;
                  double heelPoint = stepRelativeToStart.getX() - collisionSphereRadius;
                  boolean collisionIsBetweenToeAndHeel = MathTools.intervalContains(collisionRelativeToStart.getX(), Math.min(toePoint, heelPoint),
                                                                                    Math.max(toePoint, heelPoint));

                  if (collisionIsBetweenToeAndHeel)
                  {
                     updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.COLLISION_BETWEEN_FEET, closestPointOnRegion);
                     return fraction;
                  }
               }
            }
         }

         updateVisualizer();
      }

      return -1.0;
   }

   private boolean areWeTooEarlyInSwingToLookForCollisions(boolean currentlyRising, double avoidanceDistance)
   {
      double heightAboveStart = trajectoryPosition.getZ() - swingStartPosition.getZ();
      if (currentlyRising && heightAboveStart < avoidanceDistance)
         return true;

      double distanceToEnd = trajectoryPosition.distance(swingEndPosition);

      return distanceToEnd < avoidanceDistance;
   }

   private void updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType collisionType, Point3DReadOnly collision)
   {
      if (collisionType.ordinal() > this.mostSevereCollisionType.getEnumValue().ordinal())
      {
         this.mostSevereCollisionType.set(collisionType);
      }

      if (collision != null)
      {
         if (trajectoryPosition.distanceSquared(collision) < trajectoryPosition.distanceSquared(closestPolygonPointMap.get(collisionType)))
            closestPolygonPointMap.get(collisionType).set(collision);
      }
   }

   private boolean checkIfCollidingWithFloorPlane(Point3DReadOnly collisionPoint)
   {
      // see if the collision is below the floor
      return swingFloorPlane.distance(collisionPoint) < minimumHeightAboveFloorForCollision.getDoubleValue() + minimumClearance.getDoubleValue();
   }

   private void computeWaypointAdjustmentDirection(double fraction)
   {
      // so this does radially about the midpoint w.r.t. how far through swing we are.
      axisAngle.set(swingTrajectoryPlane.getNormal(), Math.PI * fraction);
      rigidBodyTransform.setRotation(axisAngle);

      waypointAdjustmentDirection.sub(swingStartPosition, swingEndPosition);
      waypointAdjustmentDirection.normalize();
      rigidBodyTransform.transform(waypointAdjustmentDirection);
   }

   public RecyclingArrayList<FramePoint3D> getExpandedWaypoints()
   {
      return adjustedWaypoints;
   }

   public boolean wereWaypointsAdjusted()
   {
      return wereWaypointsAdjusted.getBooleanValue();
   }

   public SwingOverPlanarRegionsStatus getStatus()
   {
      return status.getEnumValue();
   }

   // VISULIZER METHODS

   public void updateVisualizer()
   {
      if (visualizer.isPresent())
      {
         visualizer.get().run();
      }
   }

   public void attachVisualizer(Runnable visualizer)
   {
      this.visualizer = Optional.of(visualizer);
   }

   public PoseReferenceFrame getSolePoseReferenceFrame()
   {
      return solePoseReferenceFrame;
   }

   public FramePoint3D getClosestPolygonPoint(SwingOverPlanarRegionsCollisionType collisionType)
   {
      return closestPolygonPointMap.get(collisionType);
   }

   public SwingOverPlanarRegionsCollisionType getMostSevereCollisionType()
   {
      return mostSevereCollisionType.getEnumValue();
   }

   public double getCollisionSphereRadius()
   {
      return collisionSphereRadius;
   }

   public double getMinimumClearance()
   {
      return minimumClearance.getDoubleValue();
   }
}
