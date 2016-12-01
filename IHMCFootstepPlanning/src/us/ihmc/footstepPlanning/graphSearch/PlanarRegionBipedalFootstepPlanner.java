package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
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
   
   protected PlanarRegionsList planarRegionsList;
   protected SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;

   protected RobotSide initialSide;
   protected RigidBodyTransform initialFootPose = new RigidBodyTransform();

   protected SideDependentList<RigidBodyTransform> goalFootstepPoses;

   private FootstepPlannerGoalType footstepPlannerGoalType;
   private Point2d xyGoal;
   private double distanceFromXYGoal;

   protected SideDependentList<Point3d> goalPositions;
   protected SideDependentList<Double> goalYaws;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final IntegerYoVariable numberOfNodesExpanded = new IntegerYoVariable("numberOfNodesExpanded", registry);

   
   private int maximumNumberOfNodesToExpand;
   //   private FootstepPlan footstepPlan;

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
      this.maximumNumberOfNodesToExpand = maximumNumberOfNodesToExpand;
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
 
      footstepPlannerGoalType = goal.getFootstepPlannerGoalType();

      switch(footstepPlannerGoalType)
      {
      case CLOSE_TO_XY_POSITION:
         setGoalXYAndRadius(goal);
         setGoalPositionsAndYaws(goal);
         break;
      case POSE_BETWEEN_FEET:
         setGoalPositionsAndYaws(goal);
         break;
      default:
         throw new RuntimeException("Method for setting for from goal type " + footstepPlannerGoalType + " is not implemented");
      }
   }

   private void setGoalXYAndRadius(FootstepPlannerGoal goal)
   {
      goalNode = null;

      xyGoal = new Point2d(goal.getXYGoal());
      distanceFromXYGoal = goal.getDistanceFromXYGoal();
   }

   private void setGoalPositionsAndYaws(FootstepPlannerGoal goal)
   {
      FramePose goalPose = goal.getGoalPoseBetweenFeet();
      goalPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      RigidBodyTransform goalLeftFootPose = new RigidBodyTransform();
      RigidBodyTransform goalRightFootPose = new RigidBodyTransform();

      goalPose.getPose(goalLeftFootPose);
      goalPose.getPose(goalRightFootPose);

      Vector3d toTheLeft;
      Vector3d toTheRight;

      double idealFootstepWidth = parameters.getIdealFootstepWidth();
      
      if(idealFootstepWidth == 0.0)
      {
         toTheLeft = new Vector3d(0.0, 0.15, 0.0);
         toTheRight = new Vector3d(0.0, -0.15, 0.0);
      }
      else
      {
         toTheLeft = new Vector3d(0.0, idealFootstepWidth/2.0, 0.0);
         toTheRight = new Vector3d(0.0, - idealFootstepWidth/2.0, 0.0);
      }

      goalLeftFootPose.applyTranslation(toTheLeft);
      goalRightFootPose.applyTranslation(toTheRight);

      goalFootstepPoses = new SideDependentList<>(goalLeftFootPose, goalRightFootPose);

      Point3d goalLeftSolePosition = new Point3d();
      goalLeftFootPose.transform(goalLeftSolePosition);

      Point3d goalRightSolePosition = new Point3d();
      goalRightFootPose.transform(goalRightSolePosition);

      Vector3d eulerAngles = new Vector3d();
      goalLeftFootPose.getRotationEuler(eulerAngles);
      double goalLeftSoleYaw = eulerAngles.getZ();
      goalRightFootPose.getRotationEuler(eulerAngles);
      double goalRightSoleYaw = eulerAngles.getZ();

      goalPositions = new SideDependentList<>(goalLeftSolePosition, goalRightSolePosition);
      goalYaws = new SideDependentList<>(goalLeftSoleYaw, goalRightSoleYaw);

      if (listener != null)
      {
         listener.goalWasSet(goalLeftFootPose, goalRightFootPose);
      }
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionPotentialNextStepCalculator.setPlanarRegions(planarRegionsList);

      this.planarRegionsList = planarRegionsList;

//      if (listener != null)
//      {
//         listener.planarRegionsListSet(planarRegionsList);
//      }
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
      while ((!stack.isEmpty()) && (numberOfNodesExpanded.getIntegerValue() < maximumNumberOfNodesToExpand))
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         notifyListenerNodeSelectedForExpansion(nodeToExpand);

         
         boolean nodeIsAcceptableToExpand = planarRegionPotentialNextStepCalculator.checkNodeAcceptableToExpand(nodeToExpand);
         
         if (!nodeIsAcceptableToExpand) continue;

         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);
         numberOfNodesExpanded.increment();

         // Check if at goal:

         if (nodeToExpand.isAtGoal())
         {
//            System.out.println("Expanding is at goal!!!");
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
      BipedalFootstepPlannerNode goalNode = this.planarRegionPotentialNextStepCalculator.computeGoalNodeIfGoalIsReachable(nodeToExpand);
      if (goalNode != null)
      {
         stack.push(goalNode);
         return;
      }
      
      ArrayList<BipedalFootstepPlannerNode> nodesToAdd = planarRegionPotentialNextStepCalculator.computeChildrenNodes(nodeToExpand);

      //      Collections.shuffle(nodesToAdd);
      Collections.reverse(nodesToAdd);

      for (BipedalFootstepPlannerNode node : nodesToAdd)
      {
         stack.push(node);
      } 
   }

   protected void notifyListenerNodeSnappedAndStillSelectedForExpansion(BipedalFootstepPlannerNode nodeAfterSnap)
   {
      if (listener != null)
      {
         listener.nodeSelectedForExpansion(nodeAfterSnap);
      }
   }

   protected void notifyListenerNodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeSelectedForExpansion(nodeToExpand);
      }
   }

   protected void notifyListenerNodeForExpansionWasAccepted(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeForExpansionWasAccepted(nodeToExpand);
      }
   }

   protected void notifyListenerNodeForExpansionWasRejected(BipedalFootstepPlannerNode nodeToExpand, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (listener != null)
      {
         listener.nodeForExpansionWasRejected(nodeToExpand, reason);
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


}
