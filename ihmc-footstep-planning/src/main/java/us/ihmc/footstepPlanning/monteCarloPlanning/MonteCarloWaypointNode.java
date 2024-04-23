package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.ArrayList;

public class MonteCarloWaypointNode extends MonteCarloTreeNode
{
   private Point2D position;
   private int stepLength = 4;

   public MonteCarloWaypointNode(Point2DReadOnly state, MonteCarloWaypointNode parent, int id)
   {
      super(parent, id);
      this.position = new Point2D(state);
   }

   /**
    * Back propagates the given score to the root node.
    */
   @Override
   public ArrayList<MonteCarloWaypointNode> getAvailableStates(MonteCarloPlanningWorld world)
   {
      ArrayList<MonteCarloWaypointNode> availableStates = new ArrayList<>();

      Vector2D[] actions = new Vector2D[] {new Vector2D(0, stepLength),
                                           new Vector2D(0, -stepLength),
                                           new Vector2D(stepLength, 0),
                                           new Vector2D(-stepLength, 0)};

      for (Vector2D action : actions)
      {
         if (checkActionObstacles(position, action, world))
         {
            if (checkActionBoundaries(position, action, world.getGridWidth()))
            {
               availableStates.add(new MonteCarloWaypointNode(computeActionResult(position, action), null, 0));
            }
         }
      }

      //LogTools.warn("Available Actions: {}", availableStates.size());

      return availableStates;
   }

   public boolean checkActionObstacles(Point2DReadOnly state, Vector2DReadOnly action, MonteCarloPlanningWorld world)
   {
      Point2D position = computeActionResult(state, action);
      return !MonteCarloPlannerTools.isPointOccupied(position, world.getGrid());
   }

   public boolean checkActionBoundaries(Point2DReadOnly state, Vector2DReadOnly action, int gridWidth)
   {
      Point2D position = new Point2D();
      position.add(state, action);
      return MonteCarloPlannerTools.isWithinGridBoundaries(position, gridWidth);
   }

   private static Point2D computeActionResult(Point2DReadOnly state, Vector2DReadOnly action)
   {
      Point2D actionResult = new Point2D();
      actionResult.add(state, action);
      return actionResult;
   }

   public Point2D getState()
   {
      return position;
   }
}
