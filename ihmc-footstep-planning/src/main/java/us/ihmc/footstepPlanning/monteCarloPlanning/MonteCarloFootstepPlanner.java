package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;

public class MonteCarloFootstepPlanner
{
   public MonteCarloFootstepPlanner()
   {

   }

   public FootstepPlan generateFootstepPlan(Mat heightMapImage, Mat contactMapImage, MonteCarloFootstepPlannerParameters plannerParameters)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      return footstepPlan;
   }
}
