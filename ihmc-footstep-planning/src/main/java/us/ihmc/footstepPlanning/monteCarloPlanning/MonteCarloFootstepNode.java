package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class MonteCarloFootstepNode extends MonteCarloTreeNode
{
   private Point3D position;

   private RobotSide robotSide;
   private int stepLength = 4;

   public MonteCarloFootstepNode(Point3DReadOnly state, MonteCarloFootstepNode parent, int id)
   {
      super(parent, id);
      this.position = new Point3D(state);
   }

   /**
    * Back propagates the given score to the root node.
    */
   @Override
   public ArrayList<MonteCarloFootstepNode> getAvailableStates(MonteCarloPlanningWorld world)
   {
      ArrayList<MonteCarloFootstepNode> availableStates = new ArrayList<>();

      Vector3D[] actions = new Vector3D[] {new Vector3D(0, stepLength, 0),
                                           new Vector3D(0, -stepLength, 0),
                                           new Vector3D(stepLength, 0, 0),
                                           new Vector3D(-stepLength, 0, 0)};

      for (Vector3D action : actions)
      {
         if (checkActionObstacles(action, world))
         {
            if (checkActionBoundaries(action, world.getGridWidth()))
            {
               availableStates.add(computeActionResult(action));
            }
         }
      }

      return availableStates;
   }

   public boolean checkActionObstacles(Vector3DReadOnly action, MonteCarloPlanningWorld world)
   {
      MonteCarloFootstepNode resultState = computeActionResult(action);
      //return !MonteCarloPlannerTools.isPointOccupied(resultState.getPosition(), world.getGrid());
      return true;
   }

   public boolean checkActionBoundaries(Vector3DReadOnly action, int gridWidth)
   {
      Point3D newPosition = new Point3D();
      newPosition.add(position, action);
      //return MonteCarloPlannerTools.isWithinGridBoundaries(position, gridWidth);
      return true;
   }

   private MonteCarloFootstepNode computeActionResult(Vector3DReadOnly action)
   {
      Point3D actionResult = new Point3D();
      actionResult.add(position, action);
      return new MonteCarloFootstepNode(actionResult, null, 0);
   }

   public Point3D getPosition()
   {
      return position;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
