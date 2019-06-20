package us.ihmc.avatar.footstepPlanning;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class AdaptiveSwingTrajectoryCalculator
{
   private static final double boxHeight = 0.5;
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

      boolean stepUp = stepHeightDifference > 0.0;
      Pose3DReadOnly stepAtRiskOfStubbing = stepUp ? startPose : endPose;

      if(checkForToeStub(stepAtRiskOfStubbing, stepUp, planarRegionsList))
      {
         if(stepUp)
         {
            waypointProportions[0] = waypointProportions[0] - adaptiveSwingParameters.getWaypointProportionShiftForStubAvoidance();
         }
         else
         {
            waypointProportions[1] = waypointProportions[1] + adaptiveSwingParameters.getWaypointProportionShiftForStubAvoidance();
         }
      }
   }

   private boolean checkForToeStub(Pose3DReadOnly stepAtRiskOfToeStubbing, boolean stepUp, PlanarRegionsList planarRegionsList)
   {
      Box3D footStubBox = new Box3D();
      double stubClearance = adaptiveSwingParameters.getFootStubClearance();
      footStubBox.setSize(stubClearance, walkingControllerParameters.getSteppingParameters().getFootWidth(), boxHeight);

      Pose3D boxCenter = new Pose3D(stepAtRiskOfToeStubbing);

      double footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      double xDirection = stepUp ? 1.0 : -1.0;
      boxCenter.appendTranslation(xDirection * 0.5 * (footLength + stubClearance), 0.0, 0.0);

      double zOffset = boxGroundClearance + 0.5 * boxHeight;
      boxCenter.appendTranslation(0.0, 0.0, zOffset);
      footStubBox.getPose().set(boxCenter);

      ConvexPolytope3D bodyBoxPolytope = new ConvexPolytope3D();
      Point3DBasics[] vertices = footStubBox.getVertices();

      for (int i = 0; i < vertices.length; i++)
      {
         bodyBoxPolytope.addVertex(vertices[i]);
      }

      List<ConvexPolytope3D> planarRegionPolytopes = createPlanarRegionPolytopeList(planarRegionsList);
      GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

      for (int i = 0; i < planarRegionPolytopes.size(); i++)
      {
         EuclidShape3DCollisionResult result = collisionDetector.evaluateCollision(planarRegionPolytopes.get(i), bodyBoxPolytope);
         if(result.areShapesColliding())
            return true;
      }

      return false;
   }

   private List<ConvexPolytope3D> createPlanarRegionPolytopeList(PlanarRegionsList planarRegions)
   {
      List<ConvexPolytope3D> planarRegionPolytopes = new ArrayList<>();
      RigidBodyTransform transform = new RigidBodyTransform();
      FramePoint3D framePoint = new FramePoint3D();

      List<PlanarRegion> planarRegionsList = planarRegions.getPlanarRegionsAsList();
      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.get(i);
         ConvexPolytope3D planarRegionPolytope = new ConvexPolytope3D();

         List<? extends Point2DReadOnly> pointsInPlanarRegion = planarRegion.getConvexHull().getVertexBufferView();
         planarRegion.getTransformToWorld(transform);

         for (Point2DReadOnly point : pointsInPlanarRegion)
         {
            framePoint.set(point.getX(), point.getY(), 0.0);
            framePoint.applyTransform(transform);
            planarRegionPolytope.addVertex(framePoint);
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
