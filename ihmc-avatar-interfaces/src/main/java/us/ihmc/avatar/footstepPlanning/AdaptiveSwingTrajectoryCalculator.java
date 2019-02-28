package us.ihmc.avatar.footstepPlanning;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class AdaptiveSwingTrajectoryCalculator
{
   private static final double stubClearance = 0.04;
   private static final double footStubWaypointShift = 0.15;
   private static final double boxHeight = 1.0;

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
      if(checkForFootStub(stepAtRiskOfStubbing, stepUp))
      {
         if(stepUp)
         {
            waypointProportions[0] = waypointProportions[0] - footStubWaypointShift;
         }
         else
         {
            waypointProportions[1] = waypointProportions[1] - footStubWaypointShift;
         }
      }
   }

   private boolean checkForFootStub(Pose3DReadOnly stepAtRiskOfStubbing, boolean checkToeStub)
   {
      // TODO check for collision
      return false;
   }

   private void setToDefaultProportions(double[] waypointProportions)
   {
      double[] defaultWaypointProportions = walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions();
      waypointProportions[0] = defaultWaypointProportions[0];
      waypointProportions[1] = defaultWaypointProportions[1];
   }
}
