package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class FootstepPlannerUtils
{
   public static void setPlannerParametersForToolbox(BipedalFootstepPlannerParameters footstepPlanningParameters)
   {
      footstepPlanningParameters.setMaximumStepReach(0.55);
      footstepPlanningParameters.setMaximumStepZ(0.28);

      footstepPlanningParameters.setMaximumStepXWhenForwardAndDown(0.35); //32);
      footstepPlanningParameters.setMaximumStepZWhenForwardAndDown(0.10); //18);

      footstepPlanningParameters.setMaximumStepYaw(0.15);
      footstepPlanningParameters.setMinimumStepWidth(0.16);
      footstepPlanningParameters.setMaximumStepWidth(0.4);
      footstepPlanningParameters.setMinimumStepLength(-0.01);

      footstepPlanningParameters.setMinimumFootholdPercent(0.95);

      footstepPlanningParameters.setWiggleInsideDelta(0.02);
      footstepPlanningParameters.setMaximumXYWiggleDistance(1.0);
      footstepPlanningParameters.setMaximumYawWiggle(0.1);
      footstepPlanningParameters.setRejectIfCannotFullyWiggleInside(true);
      footstepPlanningParameters.setWiggleIntoConvexHullOfPlanarRegions(true);

      footstepPlanningParameters.setCliffHeightToShiftAwayFrom(0.03);
      footstepPlanningParameters.setMinimumDistanceFromCliffBottoms(0.24);
      
      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.22;
      footstepPlanningParameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);
   }
}
