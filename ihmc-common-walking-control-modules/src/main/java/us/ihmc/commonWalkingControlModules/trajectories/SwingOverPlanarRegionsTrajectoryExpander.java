package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.algorithms.SphereWithConvexPolygonIntersector;
import us.ihmc.robotics.geometry.shapes.FrameSphere3d;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.YoCounter;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class SwingOverPlanarRegionsTrajectoryExpander
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double[] swingWaypointProportions = TwoWaypointSwingGenerator.getDefaultWaypointProportions();

   private final TwoWaypointSwingGenerator twoWaypointSwingGenerator;

   private final YoBoolean doInitialFastApproximation;
   private final YoInteger numberOfCheckpoints;
   private final YoCounter numberOfTriesCounter;
   private final YoDouble minimumClearance;
   private final YoDouble incrementalAdjustmentDistance;
   private final YoDouble maximumAdjustmentDistance;
   private final YoDouble minimumHeightAboveFloorForCollision;
   private final YoDouble minimumFractionOfSwingForCollisionCheck;
   private final YoDouble maximumFractionOfSwingForCollisionCheck;
   private final YoEnum<SwingOverPlanarRegionsTrajectoryCollisionType> mostSevereCollisionType;
   private final YoEnum<SwingOverPlanarRegionsTrajectoryExpansionStatus> status;

   private final YoBoolean wereWaypointsAdjusted;
   private final YoFramePoint3D trajectoryPosition;
   private final PoseReferenceFrame solePoseReferenceFrame;
   private final RecyclingArrayList<FramePoint3D> originalWaypoints;
   private final RecyclingArrayList<FramePoint3D> adjustedWaypoints;
   private final double minimumSwingHeight;
   private final double maximumSwingHeight;
   private final double collisionSphereRadius;

   private final SphereWithConvexPolygonIntersector sphereWithConvexPolygonIntersector;
   private final Map<SwingOverPlanarRegionsTrajectoryCollisionType, FramePoint3D> closestPolygonPointMap;
   private final FrameSphere3d footCollisionSphere;
   private final FramePoint3D midGroundPoint;
   private final Vector3D waypointAdjustmentDirection;
   private final Plane3D waypointAdjustmentPlane;
   private final Plane3D swingFloorPlane;
   private final Plane3D swingStartToeFacingSwingEndPlane;
   private final Plane3D swingEndHeelFacingSwingStartPlane;
   private final AxisAngle axisAngle;
   private final RigidBodyTransform rigidBodyTransform;

   private final Vector3D tempPlaneNormal = new Vector3D();
   private final Point3D toeAtStartOfStep = new Point3D();
   private final Point3D heelAtEndOfStep = new Point3D();
   private final Vector3D stepDirectionVector = new Vector3D();

   // Boilerplate variables
   private final FrameVector3D initialVelocity;
   private final FrameVector3D touchdownVelocity;
   private final FramePoint3D swingStartPosition;
   private final FramePoint3D swingEndPosition;
   private final FramePoint3D stanceFootPosition;

   // Visualization
   private Optional<Runnable> visualizer;

   public enum SwingOverPlanarRegionsTrajectoryCollisionType
   {
      NO_INTERSECTION, INTERSECTION_BUT_BELOW_IGNORE_PLANE, INTERSECTION_BUT_OUTSIDE_TRAJECTORY, CRITICAL_INTERSECTION,
   }

   public enum SwingOverPlanarRegionsTrajectoryExpansionStatus
   {
      INITIALIZED, FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE, SEARCHING_FOR_SOLUTION, SOLUTION_FOUND,
   }

   public SwingOverPlanarRegionsTrajectoryExpander(WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry,
                                                   YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = "trajectoryExpander";
      twoWaypointSwingGenerator = new TwoWaypointSwingGenerator(namePrefix,
                                                                walkingControllerParameters.getSteppingParameters().getMinSwingHeightFromStanceFoot(),
                                                                walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot(),
                                                                walkingControllerParameters.getSteppingParameters().getMinSwingHeightFromStanceFoot(),
                                                                parentRegistry, graphicsListRegistry);
      minimumSwingHeight = walkingControllerParameters.getSteppingParameters().getMinSwingHeightFromStanceFoot();
      maximumSwingHeight = walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot();
      collisionSphereRadius = walkingControllerParameters.getSteppingParameters().getActualFootLength() / 2.0;

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
      status = new YoEnum<>(namePrefix + "Status", parentRegistry, SwingOverPlanarRegionsTrajectoryExpansionStatus.class);
      mostSevereCollisionType = new YoEnum<>(namePrefix + "CollisionType", parentRegistry, SwingOverPlanarRegionsTrajectoryCollisionType.class);

      trajectoryPosition = new YoFramePoint3D(namePrefix + "TrajectoryPosition", worldFrame, parentRegistry);
      solePoseReferenceFrame = new PoseReferenceFrame(namePrefix + "SolePoseReferenceFrame", worldFrame);
      originalWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      originalWaypoints.add();
      originalWaypoints.add();
      adjustedWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      adjustedWaypoints.add();
      adjustedWaypoints.add();

      sphereWithConvexPolygonIntersector = new SphereWithConvexPolygonIntersector();
      closestPolygonPointMap = new HashMap<>();
      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         closestPolygonPointMap.put(swingOverPlanarRegionsTrajectoryCollisionType, new FramePoint3D());
      }
      footCollisionSphere = new FrameSphere3d();
      midGroundPoint = new FramePoint3D();
      waypointAdjustmentDirection = new Vector3D();
      waypointAdjustmentPlane = new Plane3D();
      swingFloorPlane = new Plane3D();
      swingStartToeFacingSwingEndPlane = new Plane3D();
      swingEndHeelFacingSwingStartPlane = new Plane3D();
      axisAngle = new AxisAngle();
      rigidBodyTransform = new RigidBodyTransform();

      initialVelocity = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);
      touchdownVelocity = new FrameVector3D(worldFrame, 0.0, 0.0, walkingControllerParameters.getSwingTrajectoryParameters().getDesiredTouchdownVelocity());
      swingStartPosition = new FramePoint3D();
      swingEndPosition = new FramePoint3D();
      stanceFootPosition = new FramePoint3D();

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

   public double expandTrajectoryOverPlanarRegions(FramePose3D stanceFootPose, FramePose3D swingStartPose, FramePose3D swingEndPose,
                                                   PlanarRegionsList planarRegionsList)
   {
      stanceFootPosition.setIncludingFrame(stanceFootPose.getPosition());
      stanceFootPosition.changeFrame(worldFrame);
      twoWaypointSwingGenerator.setStanceFootPosition(stanceFootPosition);

      swingStartPosition.setIncludingFrame(swingStartPose.getPosition());
      swingStartPosition.changeFrame(worldFrame);
      twoWaypointSwingGenerator.setInitialConditions(swingStartPosition, initialVelocity);

      swingEndPosition.setIncludingFrame(swingEndPose.getPosition());
      swingEndPosition.changeFrame(worldFrame);
      twoWaypointSwingGenerator.setFinalConditions(swingEndPosition, touchdownVelocity);
      twoWaypointSwingGenerator.setStepTime(1.0);

      originalWaypoints.get(0).interpolate(swingStartPosition, swingEndPosition, swingWaypointProportions[0]);
      originalWaypoints.get(0).add(0.0, 0.0, minimumSwingHeight);
      originalWaypoints.get(1).interpolate(swingStartPosition, swingEndPosition, swingWaypointProportions[1]);
      originalWaypoints.get(1).add(0.0, 0.0, minimumSwingHeight);

      adjustedWaypoints.get(0).set(originalWaypoints.get(0));
      adjustedWaypoints.get(1).set(originalWaypoints.get(1));

      midGroundPoint.interpolate(originalWaypoints.get(0), originalWaypoints.get(1), 0.5);
      midGroundPoint.subZ(minimumSwingHeight);

      adjustSwingEndIfCoincidentWithSwingStart();

      waypointAdjustmentPlane.set(swingStartPosition, adjustedWaypoints.get(0), swingEndPosition);

      axisAngle.set(waypointAdjustmentPlane.getNormal(), Math.PI / 2.0);
      rigidBodyTransform.setRotation(axisAngle);
      tempPlaneNormal.sub(swingStartPosition, swingEndPosition);
      rigidBodyTransform.transform(tempPlaneNormal);
      tempPlaneNormal.normalize();
      swingFloorPlane.set(swingStartPosition, tempPlaneNormal);

      stepDirectionVector.sub(swingEndPosition, swingStartPosition);
      stepDirectionVector.normalize();
      toeAtStartOfStep.scaleAdd(collisionSphereRadius, stepDirectionVector, swingStartPosition);
      swingStartToeFacingSwingEndPlane.set(toeAtStartOfStep, stepDirectionVector);

      stepDirectionVector.negate();
      heelAtEndOfStep.scaleAdd(collisionSphereRadius, stepDirectionVector, swingEndPosition);
      swingEndHeelFacingSwingStartPlane.set(heelAtEndOfStep, stepDirectionVector);

      wereWaypointsAdjusted.set(false);

      status.set(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION);
      numberOfTriesCounter.resetCount();

      double filterDistance = maximumSwingHeight + collisionSphereRadius + 2.0 * minimumClearance.getDoubleValue();
      List<PlanarRegion> filteredRegions = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCapsule(swingStartPosition, swingEndPosition, filterDistance, planarRegionsList.getPlanarRegionsAsList());

      if (doInitialFastApproximation.getBooleanValue())
      {
         while (status.getEnumValue().equals(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION) && !numberOfTriesCounter.maxCountReached())
         {
            for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
            {
               closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                                     .setIncludingFrame(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
            mostSevereCollisionType.set(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

            status.set(fastCheckAndAdjustForCollisions(filteredRegions));
            updateVisualizer();
            numberOfTriesCounter.countOne();
         }
      }

      status.set(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION);
      while (status.getEnumValue().equals(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION) && !numberOfTriesCounter.maxCountReached())
      {
         for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType
               .values())
         {
            closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                                  .setIncludingFrame(worldFrame, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         }
         mostSevereCollisionType.set(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

         status.set(checkAndAdjustForCollisions(filteredRegions));
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

   private SwingOverPlanarRegionsTrajectoryExpansionStatus fastCheckAndAdjustForCollisions(List<PlanarRegion> planarRegionsList)
   {
      double maxAdjustmentDistanceSquared = MathTools.square(maximumAdjustmentDistance.getDoubleValue());

      double timeOfCollision = getFastFractionThroughTrajectoryForCollision(planarRegionsList);
      if (timeOfCollision >= 0.0)
      {
         wereWaypointsAdjusted.set(true);
         computeWaypointAdjustmentDirection(timeOfCollision);

         adjustedWaypoints.get(0).scaleAdd((1.0 - timeOfCollision) * incrementalAdjustmentDistance.getDoubleValue(), waypointAdjustmentDirection, adjustedWaypoints.get(0));
         adjustedWaypoints.get(1).scaleAdd(timeOfCollision * incrementalAdjustmentDistance.getDoubleValue(), waypointAdjustmentDirection, adjustedWaypoints.get(1));

         if (adjustedWaypoints.get(0).distanceSquared(originalWaypoints.get(0)) > maxAdjustmentDistanceSquared
               || adjustedWaypoints.get(1).distanceSquared(originalWaypoints.get(1)) > maxAdjustmentDistanceSquared)
         {
            return SwingOverPlanarRegionsTrajectoryExpansionStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE;
         }

         return SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION;
      }

      return SwingOverPlanarRegionsTrajectoryExpansionStatus.SOLUTION_FOUND;
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

         double distance = PlanarRegionTools.getDistanceFromLineSegment3DToPlanarRegion(startInLocal, endInLocal, planarRegion, closestPointOnSegment, closestPointInRegion);

         updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

         if (distance < avoidanceDistance)
         {
            Point3D closestPointOnRegionInWorld = new Point3D(closestPointInRegion);
            planarRegion.transformFromLocalToWorld(closestPointOnRegionInWorld);

            updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_BELOW_IGNORE_PLANE);

            if (!checkIfCollidingWithFloorPlane(closestPointOnRegionInWorld))
            {
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_OUTSIDE_TRAJECTORY);

               return closestPointOnSegment.distance(startInLocal) / endInLocal.distance(startInLocal);
            }
         }
      }

      return -1.0;
   }

   private SwingOverPlanarRegionsTrajectoryExpansionStatus checkAndAdjustForCollisions(List<PlanarRegion> planarRegionsList)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();

      double maxAdjustmentDistanceSquared = MathTools.square(maximumAdjustmentDistance.getDoubleValue());

      double timeOfCollision = getFractionThroughTrajectoryForCollision(planarRegionsList);
      if (timeOfCollision >= 0.0)
      {
         wereWaypointsAdjusted.set(true);
         computeWaypointAdjustmentDirection(timeOfCollision);

         adjustedWaypoints.get(0).scaleAdd((1.0 - timeOfCollision) * incrementalAdjustmentDistance.getDoubleValue(), waypointAdjustmentDirection, adjustedWaypoints.get(0));
         adjustedWaypoints.get(1).scaleAdd(timeOfCollision * incrementalAdjustmentDistance.getDoubleValue(), waypointAdjustmentDirection, adjustedWaypoints.get(1));

         if (adjustedWaypoints.get(0).distanceSquared(originalWaypoints.get(0)) > maxAdjustmentDistanceSquared
               || adjustedWaypoints.get(1).distanceSquared(originalWaypoints.get(1)) > maxAdjustmentDistanceSquared)
         {
            return SwingOverPlanarRegionsTrajectoryExpansionStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE;
         }

         return SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION;
      }

      return SwingOverPlanarRegionsTrajectoryExpansionStatus.SOLUTION_FOUND;
   }

   private double getFractionThroughTrajectoryForCollision(List<PlanarRegion> planarRegions)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();

      double stepAmount = 1.0 / numberOfCheckpoints.getIntegerValue();
      double avoidanceDistance = collisionSphereRadius + minimumClearance.getDoubleValue();
      double avoidanceDistanceSquared = MathTools.square(avoidanceDistance);

      for (double fraction = minimumFractionOfSwingForCollisionCheck.getDoubleValue();
           fraction <= maximumFractionOfSwingForCollisionCheck.getDoubleValue(); fraction += stepAmount)
      {
         twoWaypointSwingGenerator.compute(fraction);
         FramePoint3D frameTupleUnsafe = new FramePoint3D(trajectoryPosition);
         twoWaypointSwingGenerator.getPosition(frameTupleUnsafe);
         trajectoryPosition.set(frameTupleUnsafe);

         for (PlanarRegion planarRegion : planarRegions)
         {
            Point3DReadOnly closestPointOnRegion = PlanarRegionTools.closestPointOnPlane(trajectoryPosition, planarRegion);
            if (closestPointOnRegion == null)
               continue;

            double distanceToClosestPoint = closestPointOnRegion.distanceSquared(trajectoryPosition);

            updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

            if (distanceToClosestPoint < avoidanceDistanceSquared)
            {
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_BELOW_IGNORE_PLANE);

               if (!checkIfCollidingWithFloorPlane(closestPointOnRegion))
               {
                  updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_OUTSIDE_TRAJECTORY);

                  boolean isCollisionCloserToMidpointThanSwing =
                        midGroundPoint.distanceSquared(closestPointOnRegion) < midGroundPoint.distanceSquared(trajectoryPosition);

                  if (isCollisionCloserToMidpointThanSwing)
                     return fraction;

                  boolean collisionIsBetweenToeAndHeel =
                        swingStartToeFacingSwingEndPlane.isOnOrAbove(closestPointOnRegion) && swingEndHeelFacingSwingStartPlane.isOnOrAbove(closestPointOnRegion);

                  if (collisionIsBetweenToeAndHeel)
                     return fraction;
               }
            }
         }

         updateVisualizer();
      }

      return -1.0;
   }

   private void updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType collisionType)
   {
      if (collisionType.ordinal() > this.mostSevereCollisionType.getEnumValue().ordinal())
      {
         this.mostSevereCollisionType.set(collisionType);
      }
      if (footCollisionSphere.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) < footCollisionSphere
            .distance(closestPolygonPointMap.get(collisionType)))
      {
         closestPolygonPointMap.get(collisionType).set(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon());
      }
   }

   private boolean checkIfCollidingWithFloorPlane(Point3DReadOnly collisionPoint)
   {
      // see if the collision is below the floor
      if (!swingFloorPlane.isOnOrAbove(collisionPoint))
         return true;

      // see if the collision is within some distance of the floor
      return swingFloorPlane.distance(collisionPoint) < minimumHeightAboveFloorForCollision.getDoubleValue() + minimumClearance.getDoubleValue();
   }

   private void computeWaypointAdjustmentDirection(double time)
   {
      axisAngle.set(waypointAdjustmentPlane.getNormal(), Math.PI * time);
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

   public SwingOverPlanarRegionsTrajectoryExpansionStatus getStatus()
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

   public FramePoint3D getClosestPolygonPoint(SwingOverPlanarRegionsTrajectoryCollisionType collisionType)
   {
      return closestPolygonPointMap.get(collisionType);
   }

   public SwingOverPlanarRegionsTrajectoryCollisionType getMostSevereCollisionType()
   {
      return mostSevereCollisionType.getEnumValue();
   }

   public double getSphereRadius()
   {
      return footCollisionSphere.getRadius();
   }
}
