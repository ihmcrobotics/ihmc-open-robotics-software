package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapSnapWiggler;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanThenSnapPlanner
{
   private final TurnWalkTurnPlanner turnWalkTurnPlanner;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private TerrainMapData terrainMapData;
   private final HeightMapPolygonSnapper snapper;
   private final HeightMapSnapWiggler wiggler;

   public PlanThenSnapPlanner(DefaultFootstepPlannerParametersBasics footstepPlannerParameters, SideDependentList<ConvexPolygon2D> footPolygons)
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

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      this.terrainMapData = terrainMapData;
   }

   public void setBodyPath(WaypointDefinedBodyPathPlanHolder bodyPath)
   {
      turnWalkTurnPlanner.setBodyPath(bodyPath);
   }

   public void clearBodyPath()
   {
      turnWalkTurnPlanner.clearBodyPath();
   }

   private FootstepPlan footstepPlan = new FootstepPlan();

   public FootstepPlanningResult plan() throws SnappingFailedException
   {
      FootstepPlanningResult result = turnWalkTurnPlanner.plan();
      footstepPlan = turnWalkTurnPlanner.getPlan();

      if (terrainMapData == null)
         return result;

      int numberOfFootsteps = footstepPlan.getNumberOfSteps();
      for (int i = 0; i < numberOfFootsteps; i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         FramePose3D solePose = footstep.getFootstepPose();
         ConvexPolygon2D footPolygon = footPolygons.get(footstep.getRobotSide());

         DiscreteFootstep discreteFootstep = getAsDiscreteFootstep(footstep);
         FootstepSnapData snapData = snapper.computeSnapData(discreteFootstep, footPolygon, terrainMapData);
         wiggler.computeWiggleTransform(discreteFootstep, terrainMapData, snapData);
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
