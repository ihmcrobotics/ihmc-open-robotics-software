package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
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
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.algorithms.SphereWithConvexPolygonIntersector;
import us.ihmc.robotics.geometry.shapes.FrameSphere3d;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.YoCounter;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

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

   private final YoFramePoint3D trajectoryPosition;
   private final PoseReferenceFrame solePoseReferenceFrame;
   private final RecyclingArrayList<FramePoint3D> originalWaypoints;
   private final RecyclingArrayList<FramePoint3D> adjustedWaypoints;
   private final double minimumSwingHeight;
   private final double maximumSwingHeight;
   private double collisionSphereRadius;

   private final SphereWithConvexPolygonIntersector sphereWithConvexPolygonIntersector;
   private final Map<SwingOverPlanarRegionsTrajectoryCollisionType, FramePoint3D> closestPolygonPointMap;
   private final FrameSphere3d footCollisionSphere;
   private final FrameConvexPolygon2D framePlanarRegion;
   private final TransformReferenceFrame planarRegionReferenceFrame;
   private final FramePoint3D midGroundPoint;
   private final Vector3D waypointAdjustmentVector;
   private final Plane3D waypointAdjustmentPlane;
   private final Plane3D swingFloorPlane;
   private final Plane3D swingStartToeFacingSwingEndPlane;
   private final Plane3D swingEndHeelFacingSwingStartPlane;
   private final AxisAngle axisAngle;
   private final RigidBodyTransform rigidBodyTransform;

   private final Point3D tempPointOnPlane = new Point3D();
   private final Vector3D tempPlaneNormal = new Vector3D();

   // Boilerplate variables
   private final FrameVector3D initialVelocity;
   private final FrameVector3D touchdownVelocity;
   private final FramePoint3D swingStartPosition;
   private final FramePoint3D swingEndPosition;
   private final FramePoint3D stanceFootPosition;

   // Anti-garbage variables
   private final RigidBodyTransform planarRegionTransform;

   // Visualization
   private Optional<Updatable> visualizer;

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
      status = new YoEnum<SwingOverPlanarRegionsTrajectoryExpansionStatus>(namePrefix + "Status", parentRegistry,
                                                                                   SwingOverPlanarRegionsTrajectoryExpansionStatus.class);
      mostSevereCollisionType = new YoEnum<SwingOverPlanarRegionsTrajectoryCollisionType>(namePrefix + "CollisionType", parentRegistry,
                                                                                                  SwingOverPlanarRegionsTrajectoryCollisionType.class);

      trajectoryPosition = new YoFramePoint3D(namePrefix + "TrajectoryPosition", WORLD, parentRegistry);
      solePoseReferenceFrame = new PoseReferenceFrame(namePrefix + "SolePoseReferenceFrame", WORLD);
      originalWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      originalWaypoints.add();
      originalWaypoints.add();
      adjustedWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
      adjustedWaypoints.add();
      adjustedWaypoints.add();

      sphereWithConvexPolygonIntersector = new SphereWithConvexPolygonIntersector();
      closestPolygonPointMap = new HashMap<SwingOverPlanarRegionsTrajectoryCollisionType, FramePoint3D>();
      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         closestPolygonPointMap.put(swingOverPlanarRegionsTrajectoryCollisionType, new FramePoint3D());
      }
      footCollisionSphere = new FrameSphere3d();
      framePlanarRegion = new FrameConvexPolygon2D();
      planarRegionReferenceFrame = new TransformReferenceFrame("planarRegionReferenceFrame", WORLD);
      midGroundPoint = new FramePoint3D();
      waypointAdjustmentVector = new Vector3D();
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

   public double expandTrajectoryOverPlanarRegions(FramePose3D stanceFootPose, FramePose3D swingStartPose,
                                                 FramePose3D swingEndPose, PlanarRegionsList planarRegionsList)
   {
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

      originalWaypoints.get(0).setToZero();
      originalWaypoints.get(0).interpolate(swingStartPosition, swingEndPosition, swingWaypointProportions[0]);
      midGroundPoint.set(originalWaypoints.get(0));
      originalWaypoints.get(0).add(0.0, 0.0, minimumSwingHeight);
      adjustedWaypoints.get(0).set(originalWaypoints.get(0));
      originalWaypoints.get(1).setToZero();
      originalWaypoints.get(1).interpolate(swingStartPosition, swingEndPosition, swingWaypointProportions[1]);
      midGroundPoint.add(originalWaypoints.get(1));
      originalWaypoints.get(1).add(0.0, 0.0, minimumSwingHeight);
      adjustedWaypoints.get(1).set(originalWaypoints.get(1));

      midGroundPoint.scale(0.5);

      adjustSwingEndIfCoincidentWithSwingStart();

      waypointAdjustmentPlane.set(swingStartPosition, adjustedWaypoints.get(0), swingEndPosition);

      axisAngle.set(waypointAdjustmentPlane.getNormal(), Math.PI / 2.0);
      rigidBodyTransform.setRotation(axisAngle);
      tempPlaneNormal.sub(swingStartPosition, swingEndPosition);
      rigidBodyTransform.transform(tempPlaneNormal);
      tempPlaneNormal.normalize();
      swingFloorPlane.set(swingStartPosition, tempPlaneNormal);

      tempPlaneNormal.sub(swingEndPosition, swingStartPosition);
      tempPlaneNormal.normalize();
      tempPointOnPlane.scaleAdd(collisionSphereRadius, tempPlaneNormal, swingStartPosition);
      swingStartToeFacingSwingEndPlane.set(tempPointOnPlane, tempPlaneNormal);

      tempPlaneNormal.sub(swingStartPosition, swingEndPosition);
      tempPlaneNormal.normalize();
      tempPointOnPlane.scaleAdd(collisionSphereRadius, tempPlaneNormal, swingEndPosition);
      swingEndHeelFacingSwingStartPlane.set(tempPointOnPlane, tempPlaneNormal);

      status.set(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION);
      numberOfTriesCounter.resetCount();
      while (status.getEnumValue().equals(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION) && !numberOfTriesCounter.maxCountReached())
      {
         for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
         {
            closestPolygonPointMap.get(swingOverPlanarRegionsTrajectoryCollisionType).setIncludingFrame(WORLD, Double.MAX_VALUE, Double.MAX_VALUE,
                                                                                                        Double.MAX_VALUE);
         }
         mostSevereCollisionType.set(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

         status.set(tryATrajectory(planarRegionsList));
         updateVisualizer();
         numberOfTriesCounter.countOne();
      }

      double maxSpeed = twoWaypointSwingGenerator.computeAndGetMaxSpeed();
      return maxSpeed;
   }

   // TODO figure out a better solution for coincident points and replace this
   private void adjustSwingEndIfCoincidentWithSwingStart()
   {
      if(swingStartPosition.distance(swingEndPosition) < 1e-8)
         swingEndPosition.add(1e-4, 1e-4, 1e-4);
   }

   private SwingOverPlanarRegionsTrajectoryExpansionStatus tryATrajectory(PlanarRegionsList planarRegionsList)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();

      double stepAmount = 1.0 / numberOfCheckpoints.getIntegerValue();
      for (double time = 0.0; time < 1.0; time += stepAmount)
      {
         twoWaypointSwingGenerator.compute(time);
         FramePoint3D frameTupleUnsafe = new FramePoint3D(trajectoryPosition);
         twoWaypointSwingGenerator.getPosition(frameTupleUnsafe);
         trajectoryPosition.set(frameTupleUnsafe);
         solePoseReferenceFrame.setPositionAndUpdate(trajectoryPosition);

         footCollisionSphere.setToZero(WORLD);
         footCollisionSphere.setRadius(collisionSphereRadius);
         footCollisionSphere.getSphere3d().getPosition().set(solePoseReferenceFrame.getPosition());

         footCollisionSphere.changeFrame(WORLD);

         Point3D center = new Point3D();
         footCollisionSphere.getCenter(center);

         for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
            planarRegion.getTransformToWorld(planarRegionTransform);
            planarRegionReferenceFrame.setTransformAndUpdate(planarRegionTransform);
            for (int j = 0; j < planarRegion.getNumberOfConvexPolygons(); j++)
            {
               framePlanarRegion.setIncludingFrame(planarRegionReferenceFrame, planarRegion.getConvexPolygon(j));

               boolean intersectionExists = sphereWithConvexPolygonIntersector.checkIfIntersectionExists(footCollisionSphere, framePlanarRegion);
               updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.NO_INTERSECTION);

               if (intersectionExists)
               {
                  updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_BELOW_IGNORE_PLANE);

                  if (swingFloorPlane.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon())
                        && swingFloorPlane.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) > ignoreDistanceFromFloor)
                  {
                     updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.INTERSECTION_BUT_OUTSIDE_TRAJECTORY);

                     if ((swingStartToeFacingSwingEndPlane.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon())
                           && swingEndHeelFacingSwingStartPlane.isOnOrAbove(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()))
                           || midGroundPoint.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) < midGroundPoint.distance(solePoseReferenceFrame.getPosition()))
                     {
                        updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType.CRITICAL_INTERSECTION);

                        axisAngle.set(waypointAdjustmentPlane.getNormal(), Math.PI * time);
                        rigidBodyTransform.setRotation(axisAngle);

                        waypointAdjustmentVector.sub(swingStartPosition, swingEndPosition);
                        waypointAdjustmentVector.normalize();
                        rigidBodyTransform.transform(waypointAdjustmentVector);
                        waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());
                        waypointAdjustmentVector.scale(1.0 - time);
                        adjustedWaypoints.get(0).add(waypointAdjustmentVector);

                        waypointAdjustmentVector.sub(swingStartPosition, swingEndPosition);
                        waypointAdjustmentVector.normalize();
                        rigidBodyTransform.transform(waypointAdjustmentVector);
                        waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());
                        waypointAdjustmentVector.scale(time);
                        adjustedWaypoints.get(1).add(waypointAdjustmentVector);

                        if (adjustedWaypoints.get(0).distance(originalWaypoints.get(0)) > maximumAdjustmentDistance.getDoubleValue()
                              || adjustedWaypoints.get(1).distance(originalWaypoints.get(1)) > maximumAdjustmentDistance.getDoubleValue())
                        {
                           return SwingOverPlanarRegionsTrajectoryExpansionStatus.FAILURE_HIT_MAX_ADJUSTMENT_DISTANCE;
                        }

                        return SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION;
                     }
                  }
               }
            }
         }

         updateVisualizer();
      }

      return SwingOverPlanarRegionsTrajectoryExpansionStatus.SOLUTION_FOUND;
   }

   public void setCollisionSphereRadius(double collisionSphereRadius)
   {
      this.collisionSphereRadius = Math.max(0.0, collisionSphereRadius);
   }

   private void updateClosestAndMostSevereIntersectionPoint(SwingOverPlanarRegionsTrajectoryCollisionType collisionType)
   {
      if (collisionType.ordinal() > this.mostSevereCollisionType.getEnumValue().ordinal())
      {
         this.mostSevereCollisionType.set(collisionType);
      }
      if (footCollisionSphere.distance(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon()) < footCollisionSphere.distance(closestPolygonPointMap.get(collisionType)))
      {
         closestPolygonPointMap.get(collisionType).set(sphereWithConvexPolygonIntersector.getClosestPointOnPolygon());
      }
   }

   public RecyclingArrayList<FramePoint3D> getExpandedWaypoints()
   {
      return adjustedWaypoints;
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
         visualizer.get().update(0.0);
      }
   }

   public void attachVisualizer(Updatable visualizer)
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
