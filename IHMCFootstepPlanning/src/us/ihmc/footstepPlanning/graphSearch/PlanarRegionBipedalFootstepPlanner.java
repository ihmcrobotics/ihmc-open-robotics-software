package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionBipedalFootstepPlanner implements FootstepPlanner
{
   protected final Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();

   protected final PlanarRegionPotentialNextStepCalculator planarRegionPotentialNextStepCalculator;
   protected final HashMap<Integer, List<BipedalFootstepPlannerNode>> mapToAllExploredNodes = new HashMap<>();

   protected SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;

   protected RobotSide initialSide;
   protected RigidBodyTransform initialFootPose = new RigidBodyTransform();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final IntegerYoVariable maximumNumberOfNodesToExpand = new IntegerYoVariable("maximumNumberOfNodesToExpand", registry);
   protected final IntegerYoVariable numberOfNodesExpanded = new IntegerYoVariable("numberOfNodesExpanded", registry);

   protected BipedalFootstepPlannerNode startNode, goalNode;
   protected FootstepPlan footstepPlan = null;

   protected BipedalFootstepPlannerListener listener;

   public PlanarRegionBipedalFootstepPlanner(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      planarRegionPotentialNextStepCalculator = new PlanarRegionPotentialNextStepCalculator(parameters, parentRegistry);
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
      planarRegionPotentialNextStepCalculator.setBipedalFootstepPlannerListener(listener);
   }

   public void setMaximumNumberOfNodesToExpand(int maximumNumberOfNodesToExpand)
   {
      this.maximumNumberOfNodesToExpand.set(maximumNumberOfNodesToExpand);
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      planarRegionPotentialNextStepCalculator.setFeetPolygons(footPolygonsInSoleFrame);
   }

   public SideDependentList<ConvexPolygon2d> getFootPolygonsInSoleFrame()
   {
      return footPolygonsInSoleFrame;
   }

   protected boolean initialStanceFootWasSet = false;
   protected boolean goalWasSet = false;
   
   @Override
   public final void setInitialStanceFoot(FramePose stanceFootPose, RobotSide initialSide)
   {
      initialStanceFootWasSet = true;
      stanceFootPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      this.initialSide = initialSide;
      stanceFootPose.getPose(initialFootPose);
   }

   @Override
   public final void setGoal(FootstepPlannerGoal goal)
   {
      goalWasSet = true;
      planarRegionPotentialNextStepCalculator.setGoal(goal);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionPotentialNextStepCalculator.setPlanarRegions(planarRegionsList);
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (goalNode == null)
      {
         return null;
      }

      if (footstepPlan == null)
      {
         footstepPlan = new FootstepPlan(goalNode);
      }

      return footstepPlan;
   }

   protected void initialize()
   {
      stack.clear();
      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      notifiyListenersStartNodeWasAdded(startNode);
      stack.push(startNode);
//      closestNodeToGoal = null;
      mapToAllExploredNodes.clear();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      goalNode = null;
      footstepPlan = null;

      if (!initialStanceFootWasSet || !goalWasSet)
      {         
         return FootstepPlanningResult.NO_PATH_EXISTS;
      }

      initialize();
      planarRegionPotentialNextStepCalculator.setStartNode(startNode);

      numberOfNodesExpanded.set(0);
      while ((!stack.isEmpty()) && (numberOfNodesExpanded.getIntegerValue() < maximumNumberOfNodesToExpand.getIntegerValue()))
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();

         boolean nearbyNodeAlreadyExists = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
         if (nearbyNodeAlreadyExists)
            continue;

         numberOfNodesExpanded.increment();
         notifyListenerNodeIsBeingExpanded(nodeToExpand);

         if (nodeToExpand.isAtGoal())
         {
            if ((nodeToExpand.getParentNode() != null) && (nodeToExpand.getParentNode().isAtGoal()))
            {
               goalNode = nodeToExpand;

               notifyListenerSolutionWasFound(getPlan());
               return FootstepPlanningResult.OPTIMAL_SOLUTION;
            }
         }

         expandChildrenAndAddNodes(stack, nodeToExpand);
      }

      notifyListenerSolutionWasNotFound();
      return FootstepPlanningResult.NO_PATH_EXISTS;
   }

   protected void expandChildrenAndAddNodes(Deque<BipedalFootstepPlannerNode> stack, BipedalFootstepPlannerNode nodeToExpand)
   {
      ArrayList<BipedalFootstepPlannerNode> nodesToAddFromWorstToBest = planarRegionPotentialNextStepCalculator.computeChildrenNodes(nodeToExpand);

      for (BipedalFootstepPlannerNode node : nodesToAddFromWorstToBest)
      {
         stack.push(node);
      }
   }

   protected boolean checkIfNearbyNodeAlreadyExistsAndStoreIfNot(BipedalFootstepPlannerNode nodeToExpand)
   {
      int hashCode = nodeToExpand.hashCode();
      
      List<BipedalFootstepPlannerNode> nodesWithThisHash = mapToAllExploredNodes.get(hashCode);

      if (nodesWithThisHash == null)
      {
         nodesWithThisHash = new ArrayList<>();
         nodesWithThisHash.add(nodeToExpand);
         mapToAllExploredNodes.put(hashCode, nodesWithThisHash);

         return false;
      }
      else
      {
         int size = nodesWithThisHash.size();
//         System.out.println(size);
         
         for (int i = 0; i < size; i++)
         {
            BipedalFootstepPlannerNode nodeWithSameHash = nodesWithThisHash.get(i);

            if (!nodeToExpand.isAtGoal() && nodeToExpand.equals(nodeWithSameHash))
            {
               return true;
            }
         }

         return false;
      }
   }

   protected void notifyListenerSolutionWasFound(FootstepPlan footstepPlan)
   {
      if (listener != null)
      {
         listener.solutionWasFound(footstepPlan);
      }
   }

   protected void notifyListenerSolutionWasNotFound()
   {
      if (listener != null)
      {
         listener.solutionWasNotFound();
      }
   }

   protected void notifyListenerNodeIsBeingExpanded(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeIsBeingExpanded(nodeToExpand);
      }
   }
   
   
   protected void notifiyListenersStartNodeWasAdded(BipedalFootstepPlannerNode startNode)
   {
      if (listener != null)
      {
         listener.startNodeWasAdded(startNode);
      }    
   }
}
