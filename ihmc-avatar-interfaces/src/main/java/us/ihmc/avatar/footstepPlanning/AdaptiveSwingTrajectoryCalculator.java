package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class AdaptiveSwingTrajectoryCalculator
{
   private static final double boxHeight = 0.5;
   private static final double boxGroundClearance = 0.04;

   private final AdaptiveSwingParameters adaptiveSwingParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private PlanarRegionsList planarRegionsList;

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

   public boolean checkForFootCollision(Pose3DReadOnly startPose, FootstepDataMessage step)
   {      
      if(planarRegionsList == null)
      {
         return false;
      }

      double[] waypointProportions = walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions();      
      Pose3D endPose = new Pose3D(step.getLocation(), step.getOrientation());
      
      boolean toeIsClose = checkForToeOrHeelStub(startPose, true);
      boolean heelIsClose = checkForToeOrHeelStub(endPose, false);
            
      if(toeIsClose)
      {
         waypointProportions[0] = waypointProportions[0] - adaptiveSwingParameters.getWaypointProportionShiftForStubAvoidance();
      }

      if(heelIsClose)
      {
         waypointProportions[1] = waypointProportions[1] + adaptiveSwingParameters.getWaypointProportionShiftForStubAvoidance();
      }

      if(toeIsClose || heelIsClose)
      {
         step.setSwingHeight(adaptiveSwingParameters.getMaximumSwingHeight());
         step.getCustomWaypointProportions().add(waypointProportions);
      }

      return toeIsClose || heelIsClose;
   }

   private boolean checkForToeOrHeelStub(Pose3DReadOnly stepAtRiskOfToeStubbing, boolean checkToeStub)
   {
      Box3D footStubBox = new Box3D();
      double stubClearance = adaptiveSwingParameters.getFootStubClearance();
      footStubBox.setSize(stubClearance, walkingControllerParameters.getSteppingParameters().getFootWidth(), boxHeight);

      Pose3D boxCenter = new Pose3D(stepAtRiskOfToeStubbing);

      double footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      double xDirection = checkToeStub ? 1.0 : -1.0;
      boxCenter.appendTranslation(xDirection * 0.5 * (footLength + stubClearance), 0.0, boxGroundClearance + 0.5 * boxHeight);
      footStubBox.getPose().set(boxCenter);

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         if(collisionDetector.evaluateCollision(footStubBox, planarRegionsList.getPlanarRegion(i)).areShapesColliding())
            return true;
      }

      return false;
   }
   
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }
}
