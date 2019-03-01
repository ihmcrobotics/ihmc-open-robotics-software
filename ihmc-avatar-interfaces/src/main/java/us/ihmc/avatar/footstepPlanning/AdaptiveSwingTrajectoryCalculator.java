package us.ihmc.avatar.footstepPlanning;

import boofcv.struct.image.Planar;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.List;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class AdaptiveSwingTrajectoryCalculator
{
   private static final double stubClearance = 0.04;
   private static final double footStubWaypointShift = 0.15;
   private static final double boxHeight = 1.0;
   private static final double boxGroundClearance = 0.02;

   private final AdaptiveSwingParameters adaptiveSwingParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   public AdaptiveSwingTrajectoryCalculator(AdaptiveSwingParameters adaptiveSwingParameters, WalkingControllerParameters walkingControllerParameters)
   {
      this.adaptiveSwingParameters = adaptiveSwingParameters;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   public double calculateSwingTime(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double horizontalDistance = startPosition.distanceXY(endPosition);
      double maximumTranslationForMinimumSwingTime = adaptiveSwingParameters.getMaximumStepTranslationForMinimumSwingTime();
      double minimumTranslationForMaximumSwingTime = adaptiveSwingParameters.getMinimumStepTranslationForMaximumSwingTime();
      double alphaHorizontal = (horizontalDistance - maximumTranslationForMinimumSwingTime) / (minimumTranslationForMaximumSwingTime - maximumTranslationForMinimumSwingTime);

      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingTime = adaptiveSwingParameters.getMaximumStepHeightForMinimumSwingTime();
      double minimumStepHeightForMaximumSwingTime = adaptiveSwingParameters.getMinimumStepHeightForMaximumSwingTime();
      double alphaVertical = (verticalDistance - maximumStepHeightForMinimumSwingTime) / (minimumStepHeightForMaximumSwingTime - maximumStepHeightForMinimumSwingTime);
      
      double alpha = EuclidCoreTools.clamp(alphaHorizontal + alphaVertical, 0.0, 1.0);
      return EuclidCoreTools.interpolate(adaptiveSwingParameters.getMinimumSwingTime(), adaptiveSwingParameters.getMaximumSwingTime(), alpha);
   }

   public double calculateSwingHeight(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingHeight = adaptiveSwingParameters.getMaximumStepHeightForMinimumSwingHeight();
      double minimumStepHeightForMaximumSwingHeight = adaptiveSwingParameters.getMinimumStepHeightForMaximumSwingHeight();
      double alpha = (verticalDistance - maximumStepHeightForMinimumSwingHeight) / (minimumStepHeightForMaximumSwingHeight - maximumStepHeightForMinimumSwingHeight);
      alpha = EuclidCoreTools.clamp(alpha, 0.0, 1.0);
      
      return EuclidCoreTools.interpolate(adaptiveSwingParameters.getMinimumSwingHeight(), adaptiveSwingParameters.getMaximumSwingHeight(), alpha);
   }

   public void getWaypointProportions(Pose3DReadOnly startPose, Pose3DReadOnly endPose, PlanarRegionsList planarRegionsList, double[] waypointProportions)
   {
      setToDefaultProportions(waypointProportions);

      double minHeightDifferenceForStepUpOrDown = walkingControllerParameters.getSwingTrajectoryParameters().getMinHeightDifferenceForStepUpOrDown();
      double stepHeightDifference = endPose.getPosition().getZ() - startPose.getPosition().getZ();
      if(Math.abs(stepHeightDifference) < minHeightDifferenceForStepUpOrDown)
      {
         return;
      }

      Pose3D stepToCheckForToeStub = new Pose3D();
      boolean stepUp = stepHeightDifference > 0.0;
      if(stepUp)
      {
         stepToCheckForToeStub.set(startPose);
      }
      else
      {
         // Toe stub is defined as collision along +x, so just rotate around z to check for heel stub
         stepToCheckForToeStub.set(endPose);
         stepToCheckForToeStub.appendYawRotation(Math.PI);
      }

      Pose3DReadOnly stepAtRiskOfStubbing = stepUp ? startPose : endPose;

      if(checkForToeStub(stepAtRiskOfStubbing, planarRegionsList))
      {
         if(stepUp)
         {
            waypointProportions[0] = waypointProportions[0] - footStubWaypointShift;
         }
         else
         {
            waypointProportions[1] = waypointProportions[1] + footStubWaypointShift;
         }
      }
   }

   private boolean checkForToeStub(Pose3DReadOnly stepAtRiskOfToeStubbing, PlanarRegionsList planarRegionsList)
   {
      Pose3D boxCenter = new Pose3D();
      boxCenter.setPosition(stepAtRiskOfToeStubbing.getPosition());
      boxCenter.setOrientationYawPitchRoll(stepAtRiskOfToeStubbing.getYaw(), 0.0, 0.0);

      double footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      boxCenter.appendTranslation(0.5 * footLength, 0.0, 0.0);
      boxCenter.appendTranslation(0.5 * stubClearance, 0.0, 0.0);

      double zOffset = boxGroundClearance + 0.5 * boxHeight;
      if(stepAtRiskOfToeStubbing.getPitch() > 0.0)
      {
         zOffset -= Math.sin(stepAtRiskOfToeStubbing.getPitch()) * 0.5 * footLength;
      }
      else
      {
         zOffset += Math.sin(stepAtRiskOfToeStubbing.getPitch()) * 0.5 * (footLength + stubClearance);
      }

      boxCenter.appendTranslation(0.0, 0.0, zOffset);

      Box3D box = new Box3D();
      box.setPose(boxCenter);

      ConvexPolytope bodyBoxPolytope = new ConvexPolytope();
      bodyBoxPolytope.getVertices().clear();

      Point3D[] vertices = box.getVertices();
      for (int i = 0; i < vertices.length; i++)
      {
         bodyBoxPolytope.addVertex(vertices[i]);
      }

      List<ConvexPolytope> planarRegionPolytopes = createPlanarRegionPolytopeList(planarRegionsList);
      GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

      for (int i = 0; i < planarRegionPolytopes.size(); i++)
      {
         if(collisionDetector.arePolytopesColliding(planarRegionPolytopes.get(i), bodyBoxPolytope, new Point3D(), new Point3D()))
            return true;
      }

      return false;
   }

   private List<ConvexPolytope> createPlanarRegionPolytopeList(PlanarRegionsList planarRegions)
   {
      List<ConvexPolytope> planarRegionPolytopes = new ArrayList<>();
      RigidBodyTransform transform = new RigidBodyTransform();
      FramePoint3D framePoint = new FramePoint3D();

      List<PlanarRegion> planarRegionsList = planarRegions.getPlanarRegionsAsList();
      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.get(i);
         ConvexPolytope planarRegionPolytope = new ConvexPolytope();

         List<? extends Point2DReadOnly> pointsInPlanarRegion = planarRegion.getConvexHull().getVertexBufferView();
         planarRegion.getTransformToWorld(transform);

         for (Point2DReadOnly point : pointsInPlanarRegion)
         {
            framePoint.set(point.getX(), point.getY(), 0.0);
            framePoint.applyTransform(transform);
            planarRegionPolytope.addVertex(framePoint.getX(), framePoint.getY(), framePoint.getZ());
         }

         planarRegionPolytopes.add(planarRegionPolytope);
      }

      return planarRegionPolytopes;
   }

   private void setToDefaultProportions(double[] waypointProportions)
   {
      double[] defaultWaypointProportions = walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions();
      waypointProportions[0] = defaultWaypointProportions[0];
      waypointProportions[1] = defaultWaypointProportions[1];
   }
}
