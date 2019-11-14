package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;

import java.util.ArrayList;
import java.util.List;

public class OcclusionHandlingPathPlanner
{
   private final NavigableRegionsManager navigableRegionsManager;

   public OcclusionHandlingPathPlanner(NavigableRegionsManager navigableRegionsManager)
   {
      this.navigableRegionsManager = navigableRegionsManager;
   }

   public List<Point3DReadOnly> calculateBodyPath(final Point3DReadOnly startInWorld,
                                                  final Point3DReadOnly finalGoalInWorld,
                                                  boolean fullyExpandVisibilityGraph)
   {
      if (!navigableRegionsManager.initialize(startInWorld, finalGoalInWorld, fullyExpandVisibilityGraph))
         return null;

      /**
       * try to plan to final goal once. If does not reach the goal:
       * plan to the closest free/escape node to the goal
       * here, a free/escape node is defined as one on the edge of the navigable area but not next to an obstacle
       * Other options:
       * - Keep track of where we've travelled
       * - employ actual maze solving
       */

      List<Point3DReadOnly> plan = navigableRegionsManager.calculateBodyPath(startInWorld, finalGoalInWorld, fullyExpandVisibilityGraph);

      if (plan.isEmpty() || !plan.get(plan.size() - 1).geometricallyEquals(finalGoalInWorld, 1e-6))
      {
         navigableRegionsManager.expandVisibilityGraph(startInWorld, finalGoalInWorld, fullyExpandVisibilityGraph);

         // plan again, this time only to closest free edge
         ArrayList<VisibilityGraphNode> homeRegionNodes = new ArrayList<>();
         for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : navigableRegionsManager.getVisibilityGraph().getVisibilityGraphNavigableRegions())
         {
            for (VisibilityGraphNode homeRegionNode : visibilityGraphNavigableRegion.getHomeRegionNodes())
            {
               homeRegionNodes.add(homeRegionNode);
            }
         }

         if (homeRegionNodes.isEmpty())
         {
            return null; // trapped!
         }

         VisibilityGraphNode closestHomeRegionNodeToGoal = homeRegionNodes.get(0);
         for (VisibilityGraphNode visibilityGraphNode : homeRegionNodes)
         {
            if (visibilityGraphNode.getPointInWorld().distance(finalGoalInWorld) < closestHomeRegionNodeToGoal.getPointInWorld().distance(finalGoalInWorld))
            {
               closestHomeRegionNodeToGoal = visibilityGraphNode;
            }
         }

         plan = navigableRegionsManager.resetAndPlanToGoal(startInWorld, closestHomeRegionNodeToGoal.getPointInWorld());
         return plan;
      }
      else
      {
         return plan;
      }
   }
}
