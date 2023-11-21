package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
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
   private MonteCarloFootstepPlannerParameters plannerParameters;
   private MonteCarloFootstepPlanningDebugger debugger;

   private MonteCarloPlanningWorld world;
   private MonteCarloWaypointAgent agent;
   private MonteCarloFootstepNode root;

   private HashMap<MonteCarloFootstepNode, MonteCarloFootstepNode> visitedNodes = new HashMap<>();

   Random random = new Random();

   //private int searchIterations = 100;
   //private int simulationIterations = 8;
   //private int goalMargin = 5;

   private int uniqueNodeId = 0;
   private int worldHeight = 200;
   private int worldWidth = 200;

   private boolean planning = false;

   public MonteCarloFootstepPlanner(MonteCarloFootstepPlannerParameters plannerParameters)
   {
      this.debugger = new MonteCarloFootstepPlanningDebugger(this);
      this.plannerParameters = plannerParameters;
      this.world = new MonteCarloPlanningWorld(plannerParameters.getGoalMargin(), worldHeight, worldWidth);
   }

   public FootstepPlan generateFootstepPlan(MonteCarloFootstepPlannerRequest request)
   {
      if (root == null)
      {
         Point2D position = new Point2D(request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getX() * 50,
                                        request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getY() * 50);
         float yaw = (float) request.getStartFootPoses().get(RobotSide.LEFT).getYaw();
         Point3D state = new Point3D(position.getX(), position.getY(), yaw);
         root = new MonteCarloFootstepNode(state, null, RobotSide.LEFT, uniqueNodeId++);
      }

      planning = true;
      debugger.setRequest(request);

      for (int i = 0; i < plannerParameters.getNumberOfIterations(); i++)
      {
         updateTree(root, request);
      }

      FootstepPlan plan = MonteCarloPlannerTools.getFootstepPlanFromTree(root, request);

      planning = false;
      return plan;
   }

   public void updateTree(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      if (node == null)
      {
         LogTools.warn("Node is null");
         return;
      }

      //node.sortChildren();
      //node.prune(plannerParameters.getMaxNumberOfChildNodes());

      if (node.getChildren().isEmpty())
      {
         MonteCarloFootstepNode childNode = expand(node, request);
         double score = simulate(childNode, request);
         childNode.setValue((float) score);
         backPropagate(node, (float) score);
      }
      else
      {
         float bestScore = 0;
         MonteCarloFootstepNode bestNode = null;
         for (MonteCarloTreeNode child : node.getChildren())
         {
            child.updateUpperConfidenceBound(plannerParameters.getExplorationConstant());
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = (MonteCarloFootstepNode) child;
            }
         }
         updateTree(bestNode, request);
      }
   }

   public MonteCarloFootstepNode expand(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      ArrayList<?> availableStates = node.getAvailableStates(world, request);

      for (Object newStateObj : availableStates)
      {
         MonteCarloFootstepNode newState = (MonteCarloFootstepNode) newStateObj;
         double score = MonteCarloPlannerTools.scoreFootstepNode(node, newState, request, plannerParameters);

         //if (node.getLevel() == 0)
         //{
         //   LogTools.info(String.format("Previous: %d, %d, Node: %d, %d, Action: %d, %d, Score: %.2f",
         //                               (int) node.getState().getX(),
         //                               (int) node.getState().getY(),
         //                               (int) newState.getState().getX(),
         //                               (int) newState.getState().getY(),
         //                               (int) (newState.getState().getX() - node.getState().getX()),
         //                               (int) (newState.getState().getY() - node.getState().getY()),
         //                               score));
         //}

         if (node.getLevel() < plannerParameters.getMaxTreeDepth() && score > plannerParameters.getInitialValueCutoff())
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
      }

      return (MonteCarloFootstepNode) node.getMaxQueueNode();
   }

   public double simulate(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      double score = 0;

      MonteCarloFootstepNode simulationState = new MonteCarloFootstepNode(node.getState(), null, node.getRobotSide().getOppositeSide(), 0);

      for (int i = 0; i < plannerParameters.getNumberOfSimulations(); i++)
      {
         ArrayList<MonteCarloFootstepNode> nextStates = simulationState.getAvailableStates(world, request);
         int actionIndex = random.nextInt(0, nextStates.size() - 1);
         simulationState = nextStates.get(actionIndex);

         //LogTools.info(String.format("Simulation %d, Random State: %s, Actions: %d, Side:%s", i, randomState.getPosition(), nextStates.size(), randomState.getRobotSide()));
         score += MonteCarloPlannerTools.scoreFootstepNode(node, simulationState, request, plannerParameters);
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
                                                       request.getStartFootPoses().get(RobotSide.LEFT).getYaw()),
                                           null,
                                           RobotSide.LEFT,
                                           uniqueNodeId++);
   }

   public MonteCarloPlanningWorld getWorld()
   {
      return world;
   }

   public MonteCarloTreeNode getRoot()
   {
      return root;
   }

   public List<MonteCarloFootstepNode> getVisitedNodes()
   {
      return new ArrayList<>(visitedNodes.values());
   }

   public MonteCarloFootstepPlanningDebugger getDebugger()
   {
      return debugger;
   }

   public boolean isPlanning()
   {
      return planning;
   }
}
