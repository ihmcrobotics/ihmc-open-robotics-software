package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class MonteCarloWaypointNode extends MonteCarloTreeNode
{
   private Point2D position;

   public MonteCarloWaypointNode(Point2DReadOnly state, MonteCarloWaypointNode parent, int id)
   {
      super(parent, id);
      this.position = new Point2D(state);
   }

   public Point2D getPosition()
   {
      return position;
   }
}
