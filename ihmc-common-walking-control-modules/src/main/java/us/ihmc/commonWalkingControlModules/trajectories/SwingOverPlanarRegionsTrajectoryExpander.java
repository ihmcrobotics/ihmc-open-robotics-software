package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
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
   private final ConvexPolygon2D footPolygonShape;
   private final FrameConvexPolygon2D swingStartPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D swingEndPolygon = new FrameConvexPolygon2D();

   private final YoBoolean doInitialFastApproximation;
   private final YoInteger numberOfCheckpoints;
   private final YoCounter numberOfTriesCounter;
   private final YoDouble minimumClearance;
   private final YoDouble fastApproximationLessClearance;
   private final YoDouble minimumAdjustmentIncrementDistance;
   private final YoDouble maximumAdjustmentIncrementDistance;
   private final YoDouble adjustmentIncrementDistanceGain;
   private final YoDouble maximumAdjustmentDistance;
   private final YoDouble minimumHeightAboveFloorForCollision;
   private final YoEnum<SwingOverPlanarRegionsCollisionType> mostSevereCollisionType;
   private final YoEnum<SwingOverPlanarRegionsStatus> status;

   private final YoBoolean wereWaypointsAdjusted;
   private final YoFramePoint3D trajectoryPosition;
   private final PoseReferenceFrame solePoseReferenceFrame = new PoseReferenceFrame("desiredPositionFrame", worldFrame);
   private final PoseReferenceFrame startOfSwingReferenceFrame = new PoseReferenceFrame("startOfSwingFrame", worldFrame);
   private final PoseReferenceFrame endOfSwingReferenceFrame = new PoseReferenceFrame("endOfSwingFrame", worldFrame);

   private final RecyclingArrayList<FramePoint3D> originalWaypoints;
   private final RecyclingArrayList<FramePoint3D> adjustedWaypoints;
   private final double minimumSwingHeight;
   private final double maximumSwingHeight;
   private final double collisionSphereRadius;
   private final double toeLength;
   private final double heelLength;

   private final Map<SwingOverPlanarRegionsCollisionType, FramePoint3D> closestPolygonPointMap;
   private final FramePoint3D midGroundPoint;
   private final Vector3D waypointAdjustment;
   private final Plane3D swingTrajectoryPlane;
   private final Plane3D swingFloorPlane;
   private final AxisAngle axisAngle;
   private final RigidBodyTransform rigidBodyTransform;

   private final Vector3D tempPlaneNormal = new Vector3D();

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
      NO_INTERSECTION, TOO_CLOSE_TO_IGNORE_PLANE, COLLISION_ABOVE_FOOT, OUTSIDE_TRAJECTORY, COLLISION_INSIDE_TRAJECTORY, COLLISION_BETWEEN_FEET
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
      toeLength = steppingParameters.getFootForwardOffset();
      heelLength = steppingParameters.getFootBackwardOffset();
      collisionSphereRadius = Math.max(toeLength, heelLength);

      swingWaypointProportions = walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions();

      footPolygonShape = new ConvexPolygon2D();
      footPolygonShape.addVertex(steppingParameters.getFootForwardOffset(), 0.5 * steppingParameters.getToeWidth());
      footPolygonShape.addVertex(steppingParameters.getFootForwardOffset(), -0.5 * steppingParameters.getToeWidth());
      footPolygonShape.addVertex(-steppingParameters.getFootBackwardOffset(), 0.5 * steppingParameters.getFootWidth());
      footPolygonShape.addVertex(-steppingParameters.getFootBackwardOffset(), -0.5 * steppingParameters.getFootWidth());
      footPolygonShape.update();

      doInitialFastApproximation = new YoBoolean(namePrefix + "DoInitialFastApproximation", parentRegistry);
      numberOfCheckpoints = new YoInteger(namePrefix + "NumberOfCheckpoints", parentRegistry);
      numberOfTriesCounter = new YoCounter(namePrefix + "NumberOfTriesCounter", parentRegistry);
      minimumClearance = new YoDouble(namePrefix + "MinimumClearance", parentRegistry);
      fastApproximationLessClearance = new YoDouble(namePrefix + "FastApproximationLessClearance", parentRegistry);
      minimumHeightAboveFloorForCollision = new YoDouble(namePrefix + "MinimumHeightAboveFloorForCollision", parentRegistry);
      minimumAdjustmentIncrementDistance = new YoDouble(namePrefix + "MinimumAdjustmentIncrementDistance", parentRegistry);
      maximumAdjustmentIncrementDistance = new YoDouble(namePrefix + "MaximumAdjustmentIncrementDistance", parentRegistry);
      adjustmentIncrementDistanceGain = new YoDouble(namePrefix + "AdjustmentIncrementDistanceGain", parentRegistry);
      maximumAdjustmentDistance = new YoDouble(namePrefix + "MaximumAdjustmentDistance", parentRegistry);
      wereWaypointsAdjusted = new YoBoolean(namePrefix + "WereWaypointsAdjusted", parentRegistry);
      status = new YoEnum<>(namePrefix + "Status", parentRegistry, SwingOverPlanarRegionsStatus.class);
      mostSevereCollisionType = new YoEnum<>(namePrefix + "CollisionType", parentRegistry, SwingOverPlanarRegionsCollisionType.class);

      trajectoryPosition = new YoFramePoint3D(namePrefix + "TrajectoryPosition", worldFrame, parentRegistry);
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
      waypointAdjustment = new Vector3D();
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
      fastApproximationLessClearance.set(0.05);
      minimumHeightAboveFloorForCollision.set(0.02);
      numberOfCheckpoints.set(100);
      numberOfTriesCounter.setMaxCount(50);
      minimumClearance.set(0.04);
      minimumAdjustmentIncrementDistance.set(0.03);
      maximumAdjustmentIncrementDistance.set(0.15);
      adjustmentIncrementDistanceGain.set(0.95);
      maximumAdjustmentDistance.set(maximumSwingHeight - minimumSwingHeight);
   }

   public void setDoInitialFastApproximation(boolean doInitialFastApproximation)
   {
      this.doInitialFastApproximation.set(doInitialFastApproximation);
   }

   public void setFastApproximationLessClearance(double fastApproximationLessClearance)
   {
      this.fastApproximationLessClearance.set(fastApproximationLessClearance);
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

   public void setMinimumAdjustmentIncrementDistance(double minimumAdjustmentIncrementDistance)
   {
      this.minimumAdjustmentIncrementDistance.set(minimumAdjustmentIncrementDistance);
   }

   public void setMaximumAdjustmentIncrementDistance(double maximumAdjustmentIncrementDistance)
   {
      this.maximumAdjustmentIncrementDistance.set(maximumAdjustmentIncrementDistance);
   }

   public void setAdjustmentIncrementDistanceGain(double adjustmentIncrementDistanceGain)
   {
      this.adjustmentIncrementDistanceGain.set(adjustmentIncrementDistanceGain);
   }

   public void setMaximumAdjustmentDistance(double maximumAdjustmentDistance)
   {
      this.maximumAdjustmentDistance.set(maximumAdjustmentDistance);
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
      endOfSwingReferenceFrame.setPoseAndUpdate(swingEndPose);
      stepRelativeToStart.setIncludingFrame(swingEndPosition);
      stepRelativeToStart.changeFrame(startOfSwingReferenceFrame);

      swingStartPolygon.setIncludingFrame(startOfSwingReferenceFrame, footPolygonShape);
      swingEndPolygon.setIncludingFrame(endOfSwingReferenceFrame, footPolygonShape);
      swingStartPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      swingEndPolygon.changeFrameAndProjectToXYPlane(worldFrame);

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

      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();

      status.set(SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION);
      while (status.getEnumValue().equals(SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION) && !numberOfTriesCounter.maxCountReached())
      {
         for (SwingOverPlanarRegionsCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsCollisionType.values())
         {
            closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                                  .setIncludingFrame(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         }
         mostSevereCollisionType.set(SwingOverPlanarRegionsCollisionType.NO_INTERSECTION);

         status.set(checkAndAdjustForCollisions(filteredRegions, this::getFractionThroughTrajectoryForCollision));

         if (!status.getEnumValue().equals(SwingOverPlanarRegionsStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE))
         {
            twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
            twoWaypointSwingGenerator.initialize();
         }

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
                                                                    FractionThroughTrajectoryForCollision collisionChecker)
   {
      double maxAdjustmentDistanceSquared = MathTools.square(maximumAdjustmentDistance.getDoubleValue());

      FramePoint3DBasics originalFirstWaypoint = originalWaypoints.get(0);
      FramePoint3DBasics originalSecondWaypoint = originalWaypoints.get(1);
      FramePoint3DBasics adjustedFirstWaypoint = adjustedWaypoints.get(0);
      FramePoint3DBasics adjustedSecondWaypoint = adjustedWaypoints.get(1);

      Point3D pointOnTrajectory = new Point3D();
      Point3D nearestCollision = new Point3D();

      double fractionForCollision = collisionChecker.getFractionThroughTrajectoryForCollision(planarRegionsList, pointOnTrajectory, nearestCollision);

      if (fractionForCollision >= 0.0)
      {
         wereWaypointsAdjusted.set(true);
         waypointAdjustment.sub(pointOnTrajectory, nearestCollision);
         double distanceToCollision = waypointAdjustment.length();

         if (MathTools.epsilonEquals(distanceToCollision, 0.0, 1e-3))
         {
            computeWaypointAdjustmentDirection(fractionForCollision);
            waypointAdjustment.scale(minimumAdjustmentIncrementDistance.getDoubleValue());
         }
         else
         {
            double adjustmentDistance = MathTools
                  .clamp(adjustmentIncrementDistanceGain.getDoubleValue() * distanceToCollision, minimumAdjustmentIncrementDistance.getDoubleValue(),
                         maximumAdjustmentIncrementDistance.getDoubleValue());
            waypointAdjustment.scale(adjustmentDistance / distanceToCollision);
         }

         // TODO clamp the adjustment so that the waypoints remain above the foot
         adjustedFirstWaypoint.scaleAdd(1.0 - fractionForCollision, waypointAdjustment, adjustedFirstWaypoint);
         adjustedSecondWaypoint.scaleAdd(fractionForCollision, waypointAdjustment, adjustedSecondWaypoint);

         if (adjustedFirstWaypoint.distanceSquared(originalFirstWaypoint) > maxAdjustmentDistanceSquared
               || adjustedSecondWaypoint.distanceSquared(originalSecondWaypoint) > maxAdjustmentDistanceSquared)
         {
            return SwingOverPlanarRegionsStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE;
         }

         return SwingOverPlanarRegionsStatus.SEARCHING_FOR_SOLUTION;
      }

      return SwingOverPlanarRegionsStatus.SOLUTION_FOUND;
   }

   private double getFastFractionThroughTrajectoryForCollision(List<PlanarRegion> planarRegions, Point3DBasics collisionOnSegmentToPack,
                                                               Point3DBasics collisionOnRegionToPack)
   {
      double firstSegmentLength = swingStartPosition.distance(adjustedWaypoints.get(0));
      double secondSegmentLength = adjustedWaypoints.get(0).distance(adjustedWaypoints.get(1));
      double thirdSegmentLength = adjustedWaypoints.get(1).distance(swingEndPosition);
      double totalLength = firstSegmentLength + secondSegmentLength + thirdSegmentLength;

      double fractionThroughSegmentForCollision = checkLineSegmentForCollision(swingStartPosition, adjustedWaypoints.get(0), planarRegions,
                                                                               collisionOnSegmentToPack, collisionOnRegionToPack, true);
      if (fractionThroughSegmentForCollision >= 0.0)
      {
         return fractionThroughSegmentForCollision * firstSegmentLength / totalLength;
      }

      fractionThroughSegmentForCollision = checkLineSegmentForCollision(adjustedWaypoints.get(0), adjustedWaypoints.get(1), planarRegions,
                                                                        collisionOnSegmentToPack, collisionOnRegionToPack, false);
      if (fractionThroughSegmentForCollision >= 0.0)
      {
         return (fractionThroughSegmentForCollision * secondSegmentLength + firstSegmentLength) / totalLength;
      }

      fractionThroughSegmentForCollision = checkLineSegmentForCollision(adjustedWaypoints.get(1), swingEndPosition, planarRegions, collisionOnSegmentToPack,
                                                                        collisionOnRegionToPack, false);
      if (fractionThroughSegmentForCollision >= 0.0)
      {
         return (fractionThroughSegmentForCollision * thirdSegmentLength + secondSegmentLength + firstSegmentLength) / totalLength;
      }

      return -1.0;
   }

   /**
    * Returns the fraction through the segment that the collision occurs at.
    */
   private double checkLineSegmentForCollision(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint, List<PlanarRegion> planarRegions,
                                               Point3DBasics collisionOnSegmentToPack, Point3DBasics collisionOnRegionToPack, boolean collisionIsOnRising)
   {
      double avoidanceDistance = collisionSphereRadius + minimumClearance.getDoubleValue() - fastApproximationLessClearance.getDoubleValue();
      double avoidanceDistanceSquared = MathTools.square(avoidanceDistance);
      for (PlanarRegion planarRegion : planarRegions)
      {
         Point3D startInLocal = new Point3D(firstEndpoint);
         Point3D endInLocal = new Point3D(secondEndpoint);
         planarRegion.transformFromWorldToLocal(startInLocal);
         planarRegion.transformFromWorldToLocal(endInLocal);

         Point3D closestPointOnSegment = new Point3D();

         PlanarRegionTools.getDistanceFromLineSegment3DToPlanarRegion(startInLocal, endInLocal, planarRegion, closestPointOnSegment, collisionOnRegionToPack);

         // FIXME double check the frame of the collision point
         planarRegion.transformFromLocalToWorld(collisionOnRegionToPack);
         planarRegion.getTransformToWorld().transform(closestPointOnSegment, collisionOnSegmentToPack);
         trajectoryPosition.set(collisionOnSegmentToPack);

         updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.NO_INTERSECTION, collisionOnRegionToPack);

         if (checkValidityOfCollisionPoint(collisionOnSegmentToPack, collisionOnRegionToPack, avoidanceDistanceSquared, collisionIsOnRising))
            return closestPointOnSegment.distance(startInLocal) / endInLocal.distance(startInLocal);
      }

      collisionOnSegmentToPack.setToNaN();
      collisionOnRegionToPack.setToNaN();
      return -1.0;
   }

   private double getFractionThroughTrajectoryForCollision(List<PlanarRegion> planarRegions, Point3DBasics pointOnTrajectoryToPack,
                                                           Point3DBasics closestPointOnRegionToPack)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();
      adjustedWaypoints.get(0).set(twoWaypointSwingGenerator.getWaypoint(0));
      adjustedWaypoints.get(1).set(twoWaypointSwingGenerator.getWaypoint(1));

      double avoidanceDistance = collisionSphereRadius + minimumClearance.getDoubleValue();
      double avoidanceDistanceSquared = MathTools.square(avoidanceDistance);
      double stepAmount = 1.0 / numberOfCheckpoints.getIntegerValue();

      boolean collisionIsOnRising = true;

      for (double fraction = 0.0; fraction <= 1.0; fraction += stepAmount)
      {
         twoWaypointSwingGenerator.compute(fraction);
         FramePoint3D frameTupleUnsafe = new FramePoint3D(trajectoryPosition);
         twoWaypointSwingGenerator.getPosition(frameTupleUnsafe);
         trajectoryPosition.set(frameTupleUnsafe);
         solePoseReferenceFrame.setPositionAndUpdate(trajectoryPosition);
         pointOnTrajectoryToPack.set(trajectoryPosition);

         twoWaypointSwingGenerator.getWaypointTime(0);
         if (collisionIsOnRising && fraction > twoWaypointSwingGenerator.getWaypointTime(0))
            collisionIsOnRising = false;

         for (PlanarRegion planarRegion : planarRegions)
         {
            Point3DReadOnly closestPointOnRegion = PlanarRegionTools.closestPointOnPlane(trajectoryPosition, planarRegion);
            updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.NO_INTERSECTION, closestPointOnRegion);

            if (closestPointOnRegion == null)
               continue;

            closestPointOnRegionToPack.set(closestPointOnRegion);

            if (checkValidityOfCollisionPoint(pointOnTrajectoryToPack, closestPointOnRegionToPack, avoidanceDistanceSquared, collisionIsOnRising))
               return fraction;
         }

         updateVisualizer();
      }

      pointOnTrajectoryToPack.setToNaN();
      closestPointOnRegionToPack.setToNaN();
      return -1.0;
   }

   private boolean checkValidityOfCollisionPoint(Point3DReadOnly pointOnTrajectory, Point3DReadOnly closestPointOnRegion, double avoidanceDistanceSquared,
                                                 boolean collisionIsOnRising)
   {

      double distanceToClosestPoint = closestPointOnRegion.distanceSquared(pointOnTrajectory);

      if (distanceToClosestPoint < avoidanceDistanceSquared)
      {
         if (!checkIfCollidingWithFloorPlane(closestPointOnRegion))
         {
            updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.TOO_CLOSE_TO_IGNORE_PLANE, closestPointOnRegion);

            boolean isCollisionAboveFootholds =
                  isCollisionAboveStartFoot(closestPointOnRegion, collisionIsOnRising) || isCollisionAboveEndFoot(closestPointOnRegion);
            if (isCollisionAboveFootholds)
            {
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.COLLISION_ABOVE_FOOT, closestPointOnRegion);
            }
            else
            {
               boolean isCollisionInsideTheTrajectory =
                     midGroundPoint.distanceSquared(closestPointOnRegion) < midGroundPoint.distanceSquared(trajectoryPosition);

               if (isCollisionInsideTheTrajectory)
               {
                  updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.COLLISION_INSIDE_TRAJECTORY, closestPointOnRegion);
                  return true;
               }

               collisionRelativeToStart.setIncludingFrame(worldFrame, closestPointOnRegion);
               collisionRelativeToStart.changeFrame(startOfSwingReferenceFrame);

               double toePoint = toeLength;
               double heelPoint = stepRelativeToStart.getX() - heelLength;
               boolean collisionIsBetweenToeAndHeel = MathTools
                     .intervalContains(collisionRelativeToStart.getX(), Math.min(toePoint, heelPoint), Math.max(toePoint, heelPoint));

               if (collisionIsBetweenToeAndHeel)
               {
                  updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.COLLISION_BETWEEN_FEET, closestPointOnRegion);
                  return true;
               }

               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsCollisionType.OUTSIDE_TRAJECTORY, closestPointOnRegion);
            }
         }
      }

      return false;
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
      return swingFloorPlane.distance(collisionPoint) < minimumHeightAboveFloorForCollision.getDoubleValue() + minimumClearance.getDoubleValue();
   }

   private boolean isCollisionAboveEndFoot(Point3DReadOnly collisionPoint)
   {
      if (!swingEndPolygon.isPointInside(collisionPoint.getX(), collisionPoint.getY()))
         return false;

      FramePoint3D collisionInEnd = new FramePoint3D(worldFrame, collisionPoint);
      collisionInEnd.changeFrame(endOfSwingReferenceFrame);

      return Math.abs(endOfSwingReferenceFrame.getZ()) < minimumHeightAboveFloorForCollision.getDoubleValue();
   }

   private boolean isCollisionAboveStartFoot(Point3DReadOnly collisionPoint, boolean collisionIsOnRising)
   {
      return collisionIsOnRising && swingStartPolygon.isPointInside(collisionPoint.getX(), collisionPoint.getY());
   }

   private void computeWaypointAdjustmentDirection(double fraction)
   {
      // so this does radially about the midpoint w.r.t. how far through swing we are.
      axisAngle.set(swingTrajectoryPlane.getNormal(), Math.PI * fraction);
      rigidBodyTransform.setRotation(axisAngle);

      waypointAdjustment.sub(swingStartPosition, swingEndPosition);
      waypointAdjustment.normalize();
      rigidBodyTransform.transform(waypointAdjustment);
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

   public double getCollisionSphereRadius()
   {
      return collisionSphereRadius;
   }

   public double getMinimumClearance()
   {
      return minimumClearance.getDoubleValue();
   }

   @FunctionalInterface
   private interface FractionThroughTrajectoryForCollision
   {
      double getFractionThroughTrajectoryForCollision(List<PlanarRegion> planarRegionList, Point3DBasics pointOnTrajectoryToPack,
                                                      Point3DBasics nearestPointInWorldToPack);
   }
}
