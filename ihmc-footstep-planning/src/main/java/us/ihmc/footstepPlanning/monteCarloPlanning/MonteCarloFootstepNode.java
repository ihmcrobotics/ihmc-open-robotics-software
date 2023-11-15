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

   private final ArrayList<Vector3D> actions = new ArrayList<>();

   public MonteCarloFootstepNode(Point3DReadOnly state, MonteCarloFootstepNode parent, RobotSide robotSide, int id)
   {
      super(parent, id);
      this.position = new Point3D(state);

      this.robotSide = robotSide;
   }

   /**
    * Back propagates the given score to the root node.
    */
   public ArrayList<MonteCarloFootstepNode> getAvailableStates(MonteCarloPlanningWorld world, MonteCarloFootstepPlannerRequest request)
   {
      ArrayList<MonteCarloFootstepNode> availableStates = new ArrayList<>();

      MonteCarloPlannerTools.getFootstepActionGrid(actions, position, robotSide == RobotSide.LEFT ? -1 : 1);

      for (Vector3D action : actions)
      {
         if (checkActionBoundaries(action, world.getGridWidth()))
         {
            availableStates.add(computeActionResult(action));
         }
      }

      return availableStates;
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
      MonteCarloFootstepNode monteCarloFootstepNode = new MonteCarloFootstepNode(actionResult, null, robotSide.getOppositeSide(), 0);
      return monteCarloFootstepNode;
   }

   public Point3D getPosition()
   {
      return position;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj instanceof MonteCarloFootstepNode)
      {
         MonteCarloFootstepNode other = (MonteCarloFootstepNode) obj;
         return position.equals(other.position) && robotSide == other.robotSide;
      }
      else
      {
         return false;
      }
   }
}
