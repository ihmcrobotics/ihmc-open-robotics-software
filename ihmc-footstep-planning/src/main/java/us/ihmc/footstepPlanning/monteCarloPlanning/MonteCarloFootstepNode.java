package us.ihmc.footstepPlanning.monteCarloPlanning;

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

      //MonteCarloPlannerTools.getFootstepActionGrid(actions, robotSide == RobotSide.LEFT ? -1 : 1);
      MonteCarloPlannerTools.getFootstepRadialActionSet(parameters, actions, state.getZ32(), robotSide == RobotSide.LEFT ? -1 : 1);

      for (Vector3D action : actions)
      {
         if (checkActionBoundaries(action, request.getTerrainMapData().getLocalGridSize()) && checkReachability(request, parameters, action))
         {
            MonteCarloFootstepNode nodeToInsert = computeActionResult(action);
            availableStates.add(nodeToInsert);
         }
      }

      return availableStates;
   }

   public boolean checkActionBoundaries(Vector3DReadOnly action, int gridWidth)
   {
      Point3D newPosition = new Point3D();
      newPosition.add(state, action);
      return true;
      //      return MonteCarloPlannerTools.isWithinGridBoundaries(new Point2D(newPosition.getX() + (double) gridWidth / 2, newPosition.getY() + (double) gridWidth / 2), gridWidth);
   }

   public boolean checkReachability(MonteCarloFootstepPlannerRequest request, MonteCarloFootstepPlannerParameters parameters, Vector3DReadOnly action)
   {
      Point3D newPosition = new Point3D();
      newPosition.add(state, action);

      int offsetX = (int) (request.getTerrainMapData().getSensorOrigin().getX() * 50);
      int offsetY = (int) (request.getTerrainMapData().getSensorOrigin().getY() * 50);

      int rIndexPrevious = (int) (state.getX() + request.getTerrainMapData().getLocalGridSize() / 2) - offsetX;
      int cIndexPrevious = (int) (state.getY() + request.getTerrainMapData().getLocalGridSize() / 2) - offsetY;

      int rIndex = (int) (newPosition.getX() + request.getTerrainMapData().getLocalGridSize() / 2) - offsetX;
      int cIndex = (int) (newPosition.getY() + request.getTerrainMapData().getLocalGridSize() / 2) - offsetY;

      double previousHeight = request.getTerrainMapData().getHeightLocal(rIndexPrevious, cIndexPrevious);
      double currentHeight = request.getTerrainMapData().getHeightLocal(rIndex, cIndex);

      boolean valid = ((currentHeight - previousHeight) < parameters.getMaxTransferHeight());
      valid = valid & ((currentHeight - previousHeight) > -parameters.getMaxTransferDepth());
      valid = valid & (Math.abs(state.getZ() - newPosition.getZ()) < parameters.getMaxTransferYaw());

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
