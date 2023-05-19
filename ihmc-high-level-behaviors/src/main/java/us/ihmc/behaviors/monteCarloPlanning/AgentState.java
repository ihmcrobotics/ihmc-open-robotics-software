package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class AgentState
{
   private final Point2D position = new Point2D();

   public AgentState(Point2D position)
   {
      this.position.set(position);
   }

   public void updateState(Vector2D action)
   {
      position.add(action);
   }

   public Point2D getPosition()
   {
      return position;
   }
}
