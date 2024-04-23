package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Objects;

public class MonteCarloFootstepNode extends MonteCarloTreeNode
{
   private Point3D state;
   private RobotSide robotSide;

   private final ArrayList<Vector3D> actions = new ArrayList<>();

   public MonteCarloFootstepNode(Point3DReadOnly state, MonteCarloFootstepNode parent, RobotSide robotSide, int id)
   {
      super(parent, id);
      this.state = new Point3D(state);

      this.robotSide = robotSide;
   }

   /**
    * Back propagates the given score to the root node.
    */
   public ArrayList<MonteCarloFootstepNode> getAvailableStates(MonteCarloFootstepPlannerRequest request, MonteCarloFootstepPlannerParameters parameters)
   {
      ArrayList<MonteCarloFootstepNode> availableStates = new ArrayList<>();
      MonteCarloPlannerTools.populateFootstepActionSet(parameters, actions, state.getZ32(), robotSide == RobotSide.LEFT ? -1 : 1);

      for (Vector3D action : actions)
      {
         if (checkReachability(request, parameters, action))
         {
            MonteCarloFootstepNode nodeToInsert = computeActionResult(action);
            availableStates.add(nodeToInsert);
         }
      }

      return availableStates;
   }

   public boolean checkReachability(MonteCarloFootstepPlannerRequest request, MonteCarloFootstepPlannerParameters parameters, Vector3DReadOnly action)
   {
      Point2D newPosition = new Point2D();
      Point2D previousPosition = new Point2D(state);
      Vector2D positionAction = new Vector2D(action.getX(), action.getY()); // yaw is the z-component, not used

      newPosition.add(previousPosition, positionAction);
      newPosition.scale(1.0 / parameters.getNodesPerMeter());
      previousPosition.scale(1.0 / parameters.getNodesPerMeter());

      double previousHeight = request.getTerrainMapData().getHeightInWorld((float) previousPosition.getX(), (float) previousPosition.getY());
      double currentHeight = request.getTerrainMapData().getHeightInWorld((float) newPosition.getX(), (float) newPosition.getY());

      boolean valid = Math.abs(currentHeight - previousHeight) < parameters.getMaxTransferHeight();
      return valid;
   }

   private MonteCarloFootstepNode computeActionResult(Vector3DReadOnly action)
   {
      Point3D actionResult = new Point3D();
      actionResult.add(state, action);
      MonteCarloFootstepNode monteCarloFootstepNode = new MonteCarloFootstepNode(actionResult, null, robotSide.getOppositeSide(), 0);
      return monteCarloFootstepNode;
   }

   public Point3D getState()
   {
      return state;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
      {
         return true;
      }
      if (obj == null || getClass() != obj.getClass())
      {
         return false;
      }
      MonteCarloFootstepNode other = (MonteCarloFootstepNode) obj;
      return Double.compare(other.state.getX(), state.getX()) == 0 && Double.compare(other.state.getY(), state.getY()) == 0
             && Double.compare(other.state.getZ(), state.getZ()) == 0 && robotSide == other.robotSide;
   }

   @Override
   public int hashCode()
   {
      return Objects.hash(state.getX(), state.getY(), state.getZ(), robotSide);
   }
}
