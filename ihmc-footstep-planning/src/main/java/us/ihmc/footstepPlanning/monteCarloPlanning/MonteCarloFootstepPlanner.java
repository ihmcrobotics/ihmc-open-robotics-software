package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

public class MonteCarloFootstepPlanner
{
   private MonteCarloFootstepPlannerStatistics statistics;
   private MonteCarloFootstepPlannerParameters parameters;
   private TerrainPlanningDebugger debugger;
   private MonteCarloFootstepNode root;

   private MonteCarloFootstepPlannerRequest request;
   private final HashMap<MonteCarloFootstepNode, MonteCarloFootstepNode> visitedNodes = new HashMap<>();
   private final Random random = new Random();

   private boolean planning = false;
   private int uniqueNodeId = 0;

   public MonteCarloFootstepPlanner(MonteCarloFootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
      this.statistics = new MonteCarloFootstepPlannerStatistics();
      this.debugger = new TerrainPlanningDebugger(this);
   }

   public FootstepPlan generateFootstepPlan(MonteCarloFootstepPlannerRequest request)
   {
      this.request = request;
      statistics.startTotalTime();
      if (root == null)
      {
         Point2D position = new Point2D(request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getX() * 50,
                                        request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getY() * 50);
         float yaw = (float) -request.getStartFootPoses().get(RobotSide.LEFT).getYaw();
         Point3D state = new Point3D(position.getX(), position.getY(), yaw);
         root = new MonteCarloFootstepNode(state, null, RobotSide.LEFT, uniqueNodeId++);
      }

      planning = true;
      debugger.setRequest(request);
      debugger.refresh(request.getTerrainMapData());

      for (int i = 0; i < parameters.getNumberOfIterations(); i++)
      {
         updateTree(root, request);
      }

      FootstepPlan plan = MonteCarloPlannerTools.getFootstepPlanFromTree(root, request);

      statistics.stopTotalTime();
      statistics.setNodesPerLayerString(MonteCarloPlannerTools.getLayerCountsString(root));
      statistics.logToFile(false, true);

      planning = false;
      return plan;
   }

   public void updateTree(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      if (node == null)
      {
         LogTools.debug("Node is null");
         return;
      }

      statistics.startPruningTime();
      node.sortChildren();
      node.prune(parameters.getMaxNumberOfChildNodes());
      statistics.stopPruningTime();

      if (node.getChildren().isEmpty())
      {
         statistics.startExpansionTime();
         MonteCarloFootstepNode childNode = expand(node, request);
         statistics.stopExpansionTime();
         if (childNode != null)
         {
            statistics.startSimulationTime();
            double score = simulate(childNode, request);
            statistics.stopSimulationTime();

            childNode.setValue((float) score);

            statistics.startPropagationTime();
            backPropagate(node, (float) score);
            statistics.stopPropagationTime();
         }
      }
      else
      {
         statistics.startSearchTime();
         float bestScore = 0;
         MonteCarloFootstepNode bestNode = null;
         for (MonteCarloTreeNode child : node.getChildren())
         {
            child.updateUpperConfidenceBound(parameters.getExplorationConstant());
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = (MonteCarloFootstepNode) child;
            }
         }
         statistics.stopSearchTime();
         updateTree(bestNode, request);
      }
   }

   public MonteCarloFootstepNode expand(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      ArrayList<?> availableStates = node.getAvailableStates(request, parameters);

      //LogTools.info("Total Available States: {}", availableStates.size());

      for (Object newStateObj : availableStates)
      {
         MonteCarloFootstepNode newState = (MonteCarloFootstepNode) newStateObj;
         double score = MonteCarloPlannerTools.scoreFootstepNode(node, newState, request, parameters);

         //if (node.getLevel() == 0)
         //{
         //   debugger.plotNodes(availableStates);
         //   //LogTools.info(String.format("Previous: %d, %d, %.2f, Node: %d, %d, %.2f, Action: %d, %d, %.2f, Score: %.2f",
         //   //                            (int) node.getState().getX(),
         //   //                            (int) node.getState().getY(),
         //   //                            node.getState().getZ(),
         //   //                            (int) newState.getState().getX(),
         //   //                            (int) newState.getState().getY(),
         //   //                            newState.getState().getZ(),
         //   //                            (int) (newState.getState().getX() - node.getState().getX()),
         //   //                            (int) (newState.getState().getY() - node.getState().getY()),
         //   //                            newState.getState().getZ() - node.getState().getZ(),
         //   //                            score));
         //}

         if (node.getLevel() < parameters.getMaxTreeDepth() && score > parameters.getInitialValueCutoff())
         {
            if (visitedNodes.getOrDefault(newState, null) != null)
            {
               MonteCarloFootstepNode existingNode = visitedNodes.get(newState);
               node.addChild(existingNode);
               existingNode.getParents().add(node);
            }
            else
            {
               MonteCarloFootstepNode postNode = new MonteCarloFootstepNode(newState.getState(), node, newState.getRobotSide(), uniqueNodeId++);
               postNode.setValue((float) score);
               visitedNodes.put(newState, postNode);
               node.addChild(postNode);

               // plot the newest node on debugger
               //debugger.plotNode(postNode);
               //debugger.display(30);
            }
         }
         //else
         //{
         //   LogTools.warn("Not Inserting Action: {}, Score: {}", newState.getState(), score);
         //}

      }

      if (node.getChildren().isEmpty())
      {
         LogTools.debug("No Children");
         return null;
      }

      //return (MonteCarloFootstepNode) node.getMaxQueueNode();

      // return a random node
      return (MonteCarloFootstepNode) node.getChildren().get(random.nextInt(0, node.getChildren().size() - 1));
   }

   public double simulate(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      double score = 0;

      MonteCarloFootstepNode simulationState = new MonteCarloFootstepNode(node.getState(), null, node.getRobotSide().getOppositeSide(), 0);

      for (int i = 0; i < parameters.getNumberOfSimulations(); i++)
      {
         ArrayList<MonteCarloFootstepNode> nextStates = simulationState.getAvailableStates(request, parameters);
         //LogTools.info("Number of next states: {}", nextStates.size());
         if (nextStates.isEmpty())
            break;

         int actionIndex = random.nextInt(0, nextStates.size() - 1);
         simulationState = nextStates.get(actionIndex);

         //LogTools.info(String.format("Simulation %d, Random State: %s, Actions: %d, Side:%s", i, randomState.getPosition(), nextStates.size(), randomState.getRobotSide()));
         score += MonteCarloPlannerTools.scoreFootstepNode(node, simulationState, request, parameters);
         //LogTools.info("Action Taken: {}, Score: {}", actionIndex, score);
      }

      return score;
   }

   public void backPropagate(MonteCarloFootstepNode node, float score)
   {
      node.setValue(Math.max(score, node.getValue()));
      node.incrementVisits();

      if (!node.getParents().isEmpty())
      {
         for (MonteCarloTreeNode parent : node.getParents())
         {
            backPropagate((MonteCarloFootstepNode) parent, score);
         }
      }
   }

   public void reset(MonteCarloFootstepPlannerRequest request)
   {
      random.setSeed(100);
      uniqueNodeId = 0;
      visitedNodes.clear();

      if (request == null)
         root = new MonteCarloFootstepNode(new Point3D(), null, RobotSide.LEFT, uniqueNodeId++);
      else
         root = new MonteCarloFootstepNode(new Point3D(request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getX() * 50,
                                                       request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getY() * 50,
                                                       -request.getStartFootPoses().get(RobotSide.LEFT).getYaw()),
                                           null,
                                           RobotSide.LEFT,
                                           uniqueNodeId++);
   }

   public Vector3D transitionToOptimal()
   {
      if (root.getChildren().isEmpty())
      {
         reset(request);
         return new Vector3D();
      }

      for (MonteCarloTreeNode child : root.getChildren())
      {
         child.getParents().clear();
      }
      MonteCarloFootstepNode maxNode = (MonteCarloFootstepNode) root.getMaxQueueNode();
      Vector3D action = new Vector3D(maxNode.getState());
      action.sub(root.getState());
      action.scale(1 / 50.0);

      root = maxNode;
      MonteCarloPlannerTools.pruneTree(root, parameters.getMaxNumberOfChildNodes());
      MonteCarloPlannerTools.resetNodeLevels(root, 0);

      return action;
   }

   public MonteCarloTreeNode getRoot()
   {
      return root;
   }

   public List<MonteCarloFootstepNode> getVisitedNodes()
   {
      return new ArrayList<>(visitedNodes.values());
   }

   public TerrainPlanningDebugger getDebugger()
   {
      return debugger;
   }

   public boolean isPlanning()
   {
      return planning;
   }
}
