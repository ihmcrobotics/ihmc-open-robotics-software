package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.algorithms.SphereWithConvexPolygonIntersector;
import us.ihmc.robotics.geometry.shapes.FrameSphere3d;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class SwingOverPlanarRegionsTrajectoryExpander
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   
   private final TwoWaypointSwingGenerator twoWaypointSwingGenerator;

   private final IntegerYoVariable numberOfCheckpoints;
   private final DoubleYoVariable minimumClearance;
   private final DoubleYoVariable incrementalAdjustmentDistance;
   private final EnumYoVariable<SwingOverPlanarRegionsTrajectoryExpansionStatus> status;

   private final FrameConvexPolygon2d frameFootPolygon;
   private final PoseReferenceFrame solePoseReferenceFrame;
   private final RecyclingArrayList<FramePoint> interpolatedFlatPoints;
   private final RecyclingArrayList<FramePoint> adjustedWaypoints;
   private final double minimumSwingHeight;
   private final double maximumSwingHeight;
   
   private final SphereWithConvexPolygonIntersector sphereWithConvexPolygonIntersector;
   private final FrameSphere3d footCollisionSphere;
   private final FrameConvexPolygon2d framePlanarRegion;
   private final TransformReferenceFrame planarRegionReferenceFrame;
   private final Vector3d waypointAdjustmentVector;
   private final Plane3d waypointAdjustmentPlane;
   private final Quat4d waypointAdjustmentQuaternion;
   private final RigidBodyTransform waypointAdjustmentTransform;
   
   // Boilerplate variables
   private final FrameVector initialVelocity;
   private final FrameVector touchdownVelocity;
   private final FramePoint toeOffPosition;
   private final FramePoint swingEndPosition;
   private final FramePoint stanceFootPosition;
   private final FramePoint trajectoryPosition;
   
   // Anti-garbage variables
   private final RigidBodyTransform planarRegionTransform;
   
   public enum SwingOverPlanarRegionsTrajectoryExpansionStatus
   {
      FAILURE_HIT_MAX_SWING_HEIGHT,
      SEARCHING_FOR_SOLUTION,
      SOLUTION_FOUND,
   }

   public SwingOverPlanarRegionsTrajectoryExpander(WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry,
                                                   YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = "trajectoryExpander";
      twoWaypointSwingGenerator = new TwoWaypointSwingGenerator(namePrefix, walkingControllerParameters.getMinSwingHeightFromStanceFoot(),
                                                                walkingControllerParameters.getMaxSwingHeightFromStanceFoot(), parentRegistry,
                                                                graphicsListRegistry);
      minimumSwingHeight = walkingControllerParameters.getMinSwingHeightFromStanceFoot();
      maximumSwingHeight = walkingControllerParameters.getMaxSwingHeightFromStanceFoot();
      
      numberOfCheckpoints = new IntegerYoVariable(namePrefix + "NumberOfCheckpoints", parentRegistry);
      minimumClearance = new DoubleYoVariable(namePrefix + "MinimumClearance", parentRegistry);
      incrementalAdjustmentDistance = new DoubleYoVariable(namePrefix + "IncrementalAdjustmentDistance", parentRegistry);
      status = new EnumYoVariable<SwingOverPlanarRegionsTrajectoryExpansionStatus>(namePrefix + "Status", parentRegistry,
                                                                                   SwingOverPlanarRegionsTrajectoryExpansionStatus.class);

      frameFootPolygon = new FrameConvexPolygon2d();
      solePoseReferenceFrame = new PoseReferenceFrame("SolePoseReferenceFrame", WORLD);
      interpolatedFlatPoints = new RecyclingArrayList<>(2, FramePoint.class);
      interpolatedFlatPoints.set(0, new FramePoint());
      interpolatedFlatPoints.set(1, new FramePoint());
      adjustedWaypoints = new RecyclingArrayList<>(2, FramePoint.class);
      adjustedWaypoints.set(0, new FramePoint());
      adjustedWaypoints.set(1, new FramePoint());
      
      sphereWithConvexPolygonIntersector = new SphereWithConvexPolygonIntersector();
      footCollisionSphere = new FrameSphere3d();
      framePlanarRegion = new FrameConvexPolygon2d();
      planarRegionReferenceFrame = new TransformReferenceFrame("planarRegionReferenceFrame", WORLD);
      waypointAdjustmentVector = new Vector3d();
      waypointAdjustmentPlane = new Plane3d();
      waypointAdjustmentQuaternion = new Quat4d();
      waypointAdjustmentTransform = new RigidBodyTransform();

      initialVelocity = new FrameVector(WORLD, 0.0, 0.0, 0.0);
      touchdownVelocity = new FrameVector(WORLD, 0.0, 0.0, walkingControllerParameters.getDesiredTouchdownVelocity());
      toeOffPosition = new FramePoint();
      swingEndPosition = new FramePoint();
      stanceFootPosition = new FramePoint();
      trajectoryPosition = new FramePoint();
      
      planarRegionTransform = new RigidBodyTransform();
      
      // Set default values
      numberOfCheckpoints.set(100);
      minimumClearance.set(0.04);
      incrementalAdjustmentDistance.set(0.03);
   }

   /**
    * TODO Throw exception if past max swing height
    */
   public void expandTrajectoryOverPlanarRegions(ConvexPolygon2d footPolygonSoleFrame, FramePose stanceFootPose, FramePose swingStartPose,
                                                 FramePose swingEndPose, PlanarRegionsList planarRegionsList)
   {
      stanceFootPose.getPositionIncludingFrame(stanceFootPosition);
      stanceFootPosition.changeFrame(WORLD);
      twoWaypointSwingGenerator.setStanceFootPosition(stanceFootPosition);
      
      swingStartPose.getPositionIncludingFrame(toeOffPosition);
      toeOffPosition.changeFrame(WORLD);
      twoWaypointSwingGenerator.setInitialConditions(toeOffPosition, initialVelocity);
      
      swingEndPose.getPositionIncludingFrame(swingEndPosition);
      swingEndPosition.changeFrame(WORLD);
      twoWaypointSwingGenerator.setFinalConditions(swingEndPosition, touchdownVelocity);
      twoWaypointSwingGenerator.setStepTime(1.0);
      
      double[] defaultWaypointProportions = TwoWaypointSwingGenerator.getDefaultWaypointProportions();
      interpolatedFlatPoints.get(0).interpolate(toeOffPosition, swingEndPosition, defaultWaypointProportions[0]);
      adjustedWaypoints.get(0).set(interpolatedFlatPoints.get(0));
      adjustedWaypoints.get(0).add(0.0, 0.0, minimumSwingHeight);
      interpolatedFlatPoints.get(1).interpolate(toeOffPosition, swingEndPosition, defaultWaypointProportions[1]);
      adjustedWaypoints.get(1).set(interpolatedFlatPoints.get(1));
      adjustedWaypoints.get(1).add(0.0, 0.0, minimumSwingHeight);
      
      status.set(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION);
      while (status.equals(SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION))
      {
         status.set(tryATrajectory(footPolygonSoleFrame, planarRegionsList));
      }
   }
   
   public SwingOverPlanarRegionsTrajectoryExpansionStatus tryATrajectory(ConvexPolygon2d footPolygonSoleFrame, PlanarRegionsList planarRegionsList)
   {
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, adjustedWaypoints);
      twoWaypointSwingGenerator.initialize();
      
      double stepAmount = 1.0 / (double) numberOfCheckpoints.getIntegerValue();
      for (double time = 0; time < 1.0; time += stepAmount)
      {
         twoWaypointSwingGenerator.compute(time);
         twoWaypointSwingGenerator.getPosition(trajectoryPosition);
         solePoseReferenceFrame.setPositionAndUpdate(trajectoryPosition);
         
         footCollisionSphere.setIncludingFrame(solePoseReferenceFrame, footPolygonSoleFrame.getMaxX() + minimumClearance.getDoubleValue());
         frameFootPolygon.setIncludingFrame(solePoseReferenceFrame, footPolygonSoleFrame);
         
         footCollisionSphere.changeFrame(WORLD);
         frameFootPolygon.changeFrame(WORLD);
         
         for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
            for (int j = 0; j < planarRegion.getNumberOfConvexPolygons(); j++)
            {
               planarRegion.getTransformToWorld(planarRegionTransform);
               planarRegionReferenceFrame.setTransformAndUpdate(planarRegionTransform);
               
               framePlanarRegion.setIncludingFrame(planarRegionReferenceFrame, planarRegion.getConvexPolygon(j));

               if (sphereWithConvexPolygonIntersector.checkIfIntersectionExists(footCollisionSphere, framePlanarRegion))
               {
                  waypointAdjustmentPlane.setPoints(swingEndPosition.getPoint(), adjustedWaypoints.get(0).getPoint(), toeOffPosition.getPoint());
                  RotationTools.computeQuaternionFromYawAndZNormal(Math.PI * time, waypointAdjustmentPlane.getNormal(), waypointAdjustmentQuaternion);
                  waypointAdjustmentTransform.setRotation(waypointAdjustmentQuaternion);
                  
                  waypointAdjustmentVector.sub(toeOffPosition.getPoint(), swingEndPosition.getPoint());
                  waypointAdjustmentVector.normalize();
                  waypointAdjustmentTransform.transform(waypointAdjustmentVector);
                  waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());
                  waypointAdjustmentVector.scale(1.0 - time);
                  adjustedWaypoints.get(0).add(waypointAdjustmentVector);
                  
                  waypointAdjustmentVector.sub(toeOffPosition.getPoint(), swingEndPosition.getPoint());
                  waypointAdjustmentVector.normalize();
                  waypointAdjustmentTransform.transform(waypointAdjustmentVector);
                  waypointAdjustmentVector.scale(incrementalAdjustmentDistance.getDoubleValue());
                  waypointAdjustmentVector.scale(time);
                  adjustedWaypoints.get(1).add(waypointAdjustmentVector);
                  
                  if (adjustedWaypoints.get(0).getZ() - interpolatedFlatPoints.get(0).getZ() > maximumSwingHeight
                        || adjustedWaypoints.get(1).getZ() - interpolatedFlatPoints.get(1).getZ() > maximumSwingHeight)
                  {
                     return SwingOverPlanarRegionsTrajectoryExpansionStatus.FAILURE_HIT_MAX_SWING_HEIGHT;
                  }
                  
                  return SwingOverPlanarRegionsTrajectoryExpansionStatus.SEARCHING_FOR_SOLUTION;
               }
            }
         }
      }
      
      return SwingOverPlanarRegionsTrajectoryExpansionStatus.SOLUTION_FOUND;
   }
   
   public SwingOverPlanarRegionsTrajectoryExpansionStatus getStatus()
   {
      return status.getEnumValue();
   }
}
