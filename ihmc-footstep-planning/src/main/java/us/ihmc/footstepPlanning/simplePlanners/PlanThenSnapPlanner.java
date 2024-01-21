package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapSnapWiggler;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class PlanThenSnapPlanner
{
   private final TurnWalkTurnPlanner turnWalkTurnPlanner;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepPlannerEnvironmentHandler internalEnvironmentHandler = new FootstepPlannerEnvironmentHandler();
   private HeightMapData heightMapData;
   private final HeightMapPolygonSnapper snapper;
   private final HeightMapSnapWiggler wiggler;

   public PlanThenSnapPlanner(FootstepPlannerParametersBasics footstepPlannerParameters, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.turnWalkTurnPlanner = new TurnWalkTurnPlanner(footstepPlannerParameters);
      this.footPolygons = footPolygons;
      SnapAndWiggleSingleStepParameters parameters = new SnapAndWiggleSingleStepParameters();
      parameters.setWiggleInWrongDirectionThreshold(Double.NaN);
      snapper = new HeightMapPolygonSnapper();
      wiggler = new HeightMapSnapWiggler(footPolygons, new WiggleParameters());
   }

   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide stanceSide)
   {
      turnWalkTurnPlanner.setInitialStanceFoot(stanceFootPose, stanceSide);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      turnWalkTurnPlanner.setGoal(goal);
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      internalEnvironmentHandler.setHeightMap(heightMapData);
      this.heightMapData = heightMapData;
   }

   private FootstepPlan footstepPlan = new FootstepPlan();

   public FootstepPlanningResult plan() throws SnappingFailedException
   {
      FootstepPlanningResult result = turnWalkTurnPlanner.plan();
      footstepPlan = turnWalkTurnPlanner.getPlan();

      if (internalEnvironmentHandler.hasHeightMap())
         return result;

      int numberOfFootsteps = footstepPlan.getNumberOfSteps();
      for (int i = 0; i < numberOfFootsteps; i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         FramePose3D solePose = footstep.getFootstepPose();
         ConvexPolygon2D footPolygon = footPolygons.get(footstep.getRobotSide());
         double snapHeightThreshold = 0.04;
         double minSurfaceIncline = Math.toRadians(45.0);

         DiscreteFootstep discreteFootstep = getAsDiscreteFootstep(footstep);
         FootstepSnapData snapData = snapper.computeSnapData(discreteFootstep, footPolygon, internalEnvironmentHandler, snapHeightThreshold, minSurfaceIncline);
         wiggler.computeWiggleTransform(discreteFootstep, internalEnvironmentHandler, snapData, snapHeightThreshold, minSurfaceIncline);
         ConvexPolygon2D footHold = snapData.getCroppedFoothold();
         solePose.set(snapData.getSnappedStepTransform(discreteFootstep));

         if (footHold != null)
         {
            footstep.getFoothold().set(footHold);
         }
      }
      return result;
   }

   public FootstepPlan getPlan()
   {
      return footstepPlan;
   }

   private static FootstepSnapData getAsSnapData(RigidBodyTransformReadOnly snapTransform)
   {
      return new FootstepSnapData(snapTransform);
   }

   private static DiscreteFootstep getAsDiscreteFootstep(PlannedFootstep plannedFootstep)
   {
      return new DiscreteFootstep(plannedFootstep.getFootstepPose().getX(),
                                  plannedFootstep.getFootstepPose().getY(),
                                  plannedFootstep.getFootstepPose().getYaw(),
                                  plannedFootstep.getRobotSide());
   }
}
