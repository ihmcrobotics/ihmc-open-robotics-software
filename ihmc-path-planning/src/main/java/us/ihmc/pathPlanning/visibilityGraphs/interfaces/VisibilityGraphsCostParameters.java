package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

public interface VisibilityGraphsCostParameters
{
   default double getHeuristicWeight()
   {
      return 2.0;
   }

   default double getDistanceWeight()
   {
      return 1.0;
   }
}
