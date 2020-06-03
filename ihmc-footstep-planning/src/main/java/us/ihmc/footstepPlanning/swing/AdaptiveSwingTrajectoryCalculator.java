package us.ihmc.footstepPlanning.swing;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * Calculates suggested swing time, swing height and waypoint proportions
 */
public class AdaptiveSwingTrajectoryCalculator
{
   private static final double boxHeight = 0.5;
   private static final double boxGroundClearance = 0.04;

   private final SwingPlannerParametersReadOnly parameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final GilbertJohnsonKeerthiCollisionDetector collisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private PlanarRegionsList planarRegionsList;

   public AdaptiveSwingTrajectoryCalculator(SwingPlannerParametersReadOnly parameters, WalkingControllerParameters walkingControllerParameters)
   {
      this.parameters = parameters;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   public double calculateSwingTime(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double horizontalDistance = startPosition.distanceXY(endPosition);
      double maximumTranslationForMinimumSwingTime = parameters.getMaximumStepTranslationForMinimumSwingTime();
      double minimumTranslationForMaximumSwingTime = parameters.getMinimumStepTranslationForMaximumSwingTime();
      double alphaHorizontal = (horizontalDistance - maximumTranslationForMinimumSwingTime) / (minimumTranslationForMaximumSwingTime - maximumTranslationForMinimumSwingTime);

      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingTime = parameters.getMaximumStepHeightForMinimumSwingTime();
      double minimumStepHeightForMaximumSwingTime = parameters.getMinimumStepHeightForMaximumSwingTime();
      double alphaVertical = (verticalDistance - maximumStepHeightForMinimumSwingTime) / (minimumStepHeightForMaximumSwingTime - maximumStepHeightForMinimumSwingTime);
      
      double alpha = EuclidCoreTools.clamp(alphaHorizontal + alphaVertical, 0.0, 1.0);
      return EuclidCoreTools.interpolate(parameters.getMinimumSwingTime(), parameters.getMaximumSwingTime(), alpha);
   }

   public double calculateSwingHeight(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingHeight = parameters.getMaximumStepHeightForMinimumSwingHeight();
      double minimumStepHeightForMaximumSwingHeight = parameters.getMinimumStepHeightForMaximumSwingHeight();
      double alpha = (verticalDistance - maximumStepHeightForMinimumSwingHeight) / (minimumStepHeightForMaximumSwingHeight - maximumStepHeightForMinimumSwingHeight);
      alpha = EuclidCoreTools.clamp(alpha, 0.0, 1.0);
      
      return EuclidCoreTools.interpolate(parameters.getMinimumSwingHeight(), parameters.getMaximumSwingHeight(), alpha);
   }

   public boolean checkForFootCollision(Pose3DReadOnly startPose, PlannedFootstep step)
   {      
      if(planarRegionsList == null)
      {
         return false;
      }

      double[] waypointProportions = walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions();      
      Pose3D endPose = new Pose3D(step.getFootstepPose());
      
      boolean toeIsClose = checkForToeOrHeelStub(startPose, true);
      boolean heelIsClose = checkForToeOrHeelStub(endPose, false);
            
      if(toeIsClose)
      {
         waypointProportions[0] = waypointProportions[0] - parameters.getWaypointProportionShiftForStubAvoidance();
      }

      if(heelIsClose)
      {
         waypointProportions[1] = waypointProportions[1] + parameters.getWaypointProportionShiftForStubAvoidance();
      }

      if(toeIsClose || heelIsClose)
      {
         step.setSwingHeight(parameters.getMaximumSwingHeight());
         step.getCustomWaypointProportions().add(waypointProportions);
      }

      return toeIsClose || heelIsClose;
   }

   private boolean checkForToeOrHeelStub(Pose3DReadOnly stepAtRiskOfToeStubbing, boolean checkToeStub)
   {
      Box3D footStubBox = new Box3D();
      double stubClearance = parameters.getFootStubClearance();
      footStubBox.getSize().set(stubClearance, walkingControllerParameters.getSteppingParameters().getFootWidth(), boxHeight);

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

   public void setSwingParameters(Pose3DReadOnly initialStanceFootPose, FootstepPlan footstepPlan)
   {
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         Pose3D startPose = new Pose3D();
         Pose3D endPose = new Pose3D(footstep.getFootstepPose());

         if(i < 2)
         {
            // TODO fixme
            startPose.set(initialStanceFootPose);
         }
         else
         {
            PlannedFootstep previousStep = footstepPlan.getFootstep(i - 2);
            startPose.set(previousStep.getFootstepPose());
         }

         if(!checkForFootCollision(startPose, footstep))
         {
            footstep.setSwingHeight(calculateSwingHeight(startPose.getPosition(), endPose.getPosition()));
         }

         footstep.setSwingDuration(calculateSwingTime(startPose.getPosition(), endPose.getPosition()));
      }
   }
}
