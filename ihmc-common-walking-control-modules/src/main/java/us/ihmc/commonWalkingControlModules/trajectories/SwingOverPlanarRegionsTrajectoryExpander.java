package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   private static final double ignoreDistanceFromFloor = 0.02;
   private static final double[] swingWaypointProportions = TwoWaypointSwingGenerator.getDefaultWaypointProportions();

   private final TwoWaypointSwingGenerator twoWaypointSwingGenerator;

   private final YoInteger numberOfCheckpoints;
   private final YoCounter numberOfTriesCounter;
   private final YoDouble minimumClearance;
   private final YoDouble incrementalAdjustmentDistance;
   private final YoDouble maximumAdjustmentDistance;
   private final YoEnum<SwingOverPlanarRegionsTrajectoryCollisionType> mostSevereCollisionType;
   private final YoEnum<SwingOverPlanarRegionsTrajectoryExpansionStatus> status;

   private final YoBoolean wereWaypointsAdjusted;
   private final YoFramePoint3D trajectoryPosition;
   private final PoseReferenceFrame solePoseReferenceFrame;
   private final RecyclingArrayList<FramePoint3D> originalWaypoints;
   private final RecyclingArrayList<FramePoint3D> adjustedWaypoints;
   private final RecyclingArrayList<FrameVector3D> adjustmentVectors;
   private final double minimumSwingHeight;
   private final double maximumSwingHeight;
   private final double collisionSphereRadius;

   private final SphereWithConvexPolygonIntersector sphereWithConvexPolygonIntersector;
   private final Map<SwingOverPlanarRegionsTrajectoryCollisionType, FramePoint3D> closestPolygonPointMap;
   private final FrameSphere3d footCollisionSphere;
   private final FrameConvexPolygon2D framePlanarRegion;
   private final TransformReferenceFrame planarRegionReferenceFrame;
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

   // Anti-garbage variables
   private final RigidBodyTransform planarRegionTransform;

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

      numberOfCheckpoints = new YoInteger(namePrefix + "NumberOfCheckpoints", parentRegistry);
      numberOfTriesCounter = new YoCounter(namePrefix + "NumberOfTriesCounter", parentRegistry);
      minimumClearance = new YoDouble(namePrefix + "MinimumClearance", parentRegistry);
      incrementalAdjustmentDistance = new YoDouble(namePrefix + "IncrementalAdjustmentDistance", parentRegistry);
      maximumAdjustmentDistance = new YoDouble(namePrefix + "MaximumAdjustmentDistance", parentRegistry);
      wereWaypointsAdjusted = new YoBoolean(namePrefix + "WereWaypointsAdjusted", parentRegistry);
      status = new YoEnum<>(namePrefix + "Status", parentRegistry, SwingOverPlanarRegionsTrajectoryExpansionStatus.class);
      mostSevereCollisionType = new YoEnum<>(namePrefix + "CollisionType", parentRegistry, SwingOverPlanarRegionsTrajectoryCollisionType.class);

      trajectoryPosition = new YoFramePoint3D(namePrefix + "TrajectoryPosition", WORLD, parentRegistry);
      solePoseReferenceFrame = new PoseReferenceFrame(namePrefix + "SolePoseReferenceFrame", WORLD);
      originalWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      originalWaypoints.add();
      originalWaypoints.add();
      adjustedWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      adjustedWaypoints.add();
      adjustedWaypoints.add();
      adjustmentVectors = new RecyclingArrayList<>(2, FrameVector3D.class);
      adjustmentVectors.add();
      adjustmentVectors.add();

      sphereWithConvexPolygonIntersector = new SphereWithConvexPolygonIntersector();
      closestPolygonPointMap = new HashMap<>();
      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         closestPolygonPointMap.put(swingOverPlanarRegionsTrajectoryCollisionType, new FramePoint3D());
      }
      footCollisionSphere = new FrameSphere3d();
      framePlanarRegion = new FrameConvexPolygon2D();
      planarRegionReferenceFrame = new TransformReferenceFrame("planarRegionReferenceFrame", WORLD);
      midGroundPoint = new FramePoint3D();
      waypointAdjustmentDirection = new Vector3D();
      waypointAdjustmentPlane = new Plane3D();
      swingFloorPlane = new Plane3D();
      swingStartToeFacingSwingEndPlane = new Plane3D();
      swingEndHeelFacingSwingStartPlane = new Plane3D();
      axisAngle = new AxisAngle();
      rigidBodyTransform = new RigidBodyTransform();

      initialVelocity = new FrameVector3D(WORLD, 0.0, 0.0, 0.0);
      touchdownVelocity = new FrameVector3D(WORLD, 0.0, 0.0, walkingControllerParameters.getSwingTrajectoryParameters().getDesiredTouchdownVelocity());
      swingStartPosition = new FramePoint3D();
      swingEndPosition = new FramePoint3D();
      stanceFootPosition = new FramePoint3D();

      planarRegionTransform = new RigidBodyTransform();

      visualizer = Optional.empty();

      // Set default values
      numberOfCheckpoints.set(100);
      numberOfTriesCounter.setMaxCount(50);
      minimumClearance.set(0.04);
      incrementalAdjustmentDistance.set(0.03);
      maximumAdjustmentDistance.set(maximumSwingHeight - minimumSwingHeight);
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

   public double expandTrajectoryOverPlanarRegions(FramePose3D stanceFootPose, FramePose3D swingStartPose, FramePose3D swingEndPose,
                                                   PlanarRegionsList planarRegionsList)
   {
      wereWaypointsAdjusted.set(false);
      stanceFootPosition.setIncludingFrame(stanceFootPose.getPosition());
      stanceFootPosition.changeFrame(WORLD);
      twoWaypointSwingGenerator.setStanceFootPosition(stanceFootPosition);

      swingStartPosition.setIncludingFrame(swingStartPose.getPosition());
      swingStartPosition.changeFrame(WORLD);
      twoWaypointSwingGenerator.setInitialConditions(swingStartPosition, initialVelocity);

      swingEndPosition.setIncludingFrame(swingEndPose.getPosition());
      swingEndPosition.changeFrame(WORLD);
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

      heelAtEndOfStep.scaleAdd(-collisionSphereRadius, stepDirectionVector, swingEndPosition);
      swingEndHeelFacingSwingStartPlane.set(heelAtEndOfStep, stepDirectionVector);

      wereWaypointsAdjusted.set(false);

      status.set(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION);
      numberOfTriesCounter.resetCount();

      double filterDistance = maximumSwingHeight + collisionSphereRadius + 2.0 * minimumClearance.getDoubleValue();
      List<PlanarRegion> filteredRegions = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(swingStartPosition, swingEndPosition,
                                                                                                    filterDistance, planarRegionsList.getPlanarRegionsAsList());

      while (status.getEnumValue().equals(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION) && !numberOfTriesCounter.maxCountReached())
      {
         for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType
               .values())
         {
            closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                                  .setIncludingFrame(WORLD, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
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

   private SwingOverPlanarRegionsTrajectoryExpansionStatus checkAndAdjustForCollisions(List<PlanarRegion> planarRegionsList)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();

      double maxAdjustmentDistanceSquared = MathTools.square(maximumAdjustmentDistance.getDoubleValue());

      double timeOfCollision = getTrajectoryTimeOfCollision(planarRegionsList);
      if (timeOfCollision > 0.0)
      {
         computeWaypointAdjustmentDirection(timeOfCollision);

         adjustmentVectors.get(0).setAndScale((1.0 - timeOfCollision) * incrementalAdjustmentDistance.getDoubleValue(), waypointAdjustmentDirection);
         adjustmentVectors.get(1).setAndScale(timeOfCollision * incrementalAdjustmentDistance.getDoubleValue(), waypointAdjustmentDirection);

         adjustedWaypoints.get(0).add(adjustmentVectors.get(0));
         adjustedWaypoints.get(1).add(adjustmentVectors.get(1));

         if (adjustedWaypoints.get(0).distanceSquared(originalWaypoints.get(0)) > maxAdjustmentDistanceSquared
               || adjustedWaypoints.get(1).distanceSquared(originalWaypoints.get(1)) > maxAdjustmentDistanceSquared)
         {
            return SwingOverPlanarRegionsTrajectoryExpansionStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE;
         }

         return SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION;
      }

      return SwingOverPlanarRegionsTrajectoryExpansionStatus.SOLUTION_FOUND;
   }

   private double getTrajectoryTimeOfCollision(List<PlanarRegion> planarRegions)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();

      double stepAmount = 1.0 / numberOfCheckpoints.getIntegerValue();
      double avoidanceDistance = collisionSphereRadius + minimumClearance.getDoubleValue();

      for (double time = 0.0; time < 1.0; time += stepAmount)
      {
         twoWaypointSwingGenerator.compute(time);
         FramePoint3D frameTupleUnsafe = new FramePoint3D(trajectoryPosition);
         twoWaypointSwingGenerator.getPosition(frameTupleUnsafe);
         trajectoryPosition.set(frameTupleUnsafe);
         solePoseReferenceFrame.setPositionAndUpdate(trajectoryPosition);

         footCollisionSphere.setToZero(WORLD);
         footCollisionSphere.setRadius(avoidanceDistance);
         footCollisionSphere.getSphere3d().getPosition().set(solePoseReferenceFrame.getPosition());

         footCollisionSphere.changeFrame(WORLD);

         Point3D center = new Point3D();
         footCollisionSphere.getCenter(center);

         for (PlanarRegion planarRegion : planarRegions)
         {
            planarRegion.getTransformToWorld(planarRegionTransform);
            planarRegionReferenceFrame.setTransformAndUpdate(planarRegionTransform);

            for (int convexPolygonNumber = 0; convexPolygonNumber < planarRegion.getNumberOfConvexPolygons(); convexPolygonNumber++)
            {
               framePlanarRegion.setIncludingFrame(planarRegionReferenceFrame, planarRegion.getConvexPolygon(convexPolygonNumber));

               boolean intersectionExists = sphereWithConvexPolygonIntersector.checkIfIntersectionExists(footCollisionSphere, framePlanarRegion);
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

               if (intersectionExists)
               {
                  wereWaypointsAdjusted.set(true);

                  updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_BELOW_IGNORE_PLANE);

                  if (!checkIfCollidingWithFloorPlane())
                  {
                     updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_OUTSIDE_TRAJECTORY);

                     if ((swingStartToeFacingSwingEndPlane.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon())
                           && swingEndHeelFacingSwingStartPlane.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()))
                           || midGroundPoint.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) < midGroundPoint
                           .distance(solePoseReferenceFrame.getPosition()))
                     {
                        return time;
                     }
                  }
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

   private boolean checkIfCollidingWithFloorPlane()
   {
      // see if the collision is below the floor
      if (!swingFloorPlane.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()))
         return true;

      // see if the collision is within some distance of the floor
      return swingFloorPlane.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) < ignoreDistanceFromFloor + minimumClearance.getDoubleValue();
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
