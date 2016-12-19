package us.ihmc.footstepPlanning.aStar;

public class FootstepHeuristics
{
   public static double computeNoHeuristics()
   {
      return 0.0;
   }

   public static double computeEuclidianHeuristics(FootstepNode node1, FootstepNode node2)
   {
      return node1.euclideanDistance(node2);
   }
}
