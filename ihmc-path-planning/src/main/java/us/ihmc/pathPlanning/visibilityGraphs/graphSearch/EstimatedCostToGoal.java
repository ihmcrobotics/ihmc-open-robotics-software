package us.ihmc.pathPlanning.visibilityGraphs.graphSearch;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;

public class EstimatedCostToGoal
{
   private final VisibilityGraphsParametersReadOnly parameters;
   private final FramePoint3D goalInWorld = new FramePoint3D();

   public EstimatedCostToGoal(VisibilityGraphsParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public double compute(VisibilityGraphNode node)
   {
//      double weight = node.isPreferredNode() ? 1.0 : parameters.getWeightForNonPreferredEdge();
      return parameters.getHeuristicWeight() * parameters.getDistanceWeight()  * node.getPointInWorld().distanceXY(goalInWorld);
   }

   public void setGoalInWorld(FramePoint3DReadOnly goalInWorld)
   {
      this.goalInWorld.set(goalInWorld);
   }
}
