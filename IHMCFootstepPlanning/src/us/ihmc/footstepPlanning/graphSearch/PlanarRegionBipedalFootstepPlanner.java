package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;

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
   protected PlanarRegionPotentialNextStepCalculator planarRegionPotentialNextStepCalculator;

   protected SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;

   protected RobotSide initialSide;
   protected RigidBodyTransform initialFootPose = new RigidBodyTransform();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final IntegerYoVariable maximumNumberOfNodesToExpand = new IntegerYoVariable("maximumNumberOfNodesToExpand", registry);
   protected final IntegerYoVariable numberOfNodesExpanded = new IntegerYoVariable("numberOfNodesExpanded", registry);

   protected BipedalFootstepPlannerNode startNode, goalNode;
   protected FootstepPlan footstepPlan = null;

   protected BipedalFootstepPlannerListener listener;

   private final BipedalFootstepPlannerParameters parameters;

   public PlanarRegionBipedalFootstepPlanner(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      parameters = new BipedalFootstepPlannerParameters(parentRegistry);

      planarRegionPotentialNextStepCalculator = new PlanarRegionPotentialNextStepCalculator(parentRegistry, parameters);
   }

   public BipedalFootstepPlannerParameters getParameters()
   {
      return parameters;
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

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide initialSide)
   {
      stanceFootPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      this.initialSide = initialSide;
      stanceFootPose.getPose(initialFootPose);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
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

   @Override
   public FootstepPlanningResult plan()
   {
      goalNode = null;
      footstepPlan = null;

      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();
      stack.push(startNode);

      numberOfNodesExpanded.set(0);
      while ((!stack.isEmpty()) && (numberOfNodesExpanded.getIntegerValue() < maximumNumberOfNodesToExpand.getIntegerValue()))
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         boolean nodeIsAcceptableToExpand = planarRegionPotentialNextStepCalculator.checkNodeAcceptableToExpand(nodeToExpand);

         if (!nodeIsAcceptableToExpand)
            continue;

         numberOfNodesExpanded.increment();
         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);

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
      ArrayList<BipedalFootstepPlannerNode> nodesToAdd = planarRegionPotentialNextStepCalculator.computeChildrenNodes(nodeToExpand);

      //      Collections.shuffle(nodesToAdd);
      Collections.reverse(nodesToAdd);

      for (BipedalFootstepPlannerNode node : nodesToAdd)
      {
         stack.push(node);
      }
   }

   protected void notifyListenerSolutionWasFound(FootstepPlan footstepPlan)
   {
      if (listener != null)
      {
         listener.notifyListenerSolutionWasFound(footstepPlan);
      }
   }

   protected void notifyListenerSolutionWasNotFound()
   {
      if (listener != null)
      {
         listener.notifyListenerSolutionWasNotFound();
      }
   }

   protected void notifyListenerNodeForExpansionWasAccepted(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeForExpansionWasAccepted(nodeToExpand);
      }
   }
}
