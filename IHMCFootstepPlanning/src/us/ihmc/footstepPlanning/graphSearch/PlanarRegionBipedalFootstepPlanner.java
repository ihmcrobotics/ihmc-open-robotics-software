package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionBipedalFootstepPlanner implements FootstepPlanner
{
   private PlanarRegionsList planarRegionsList;
   private SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;

   private RobotSide initialSide;
   private RigidBodyTransform initialFootPose = new RigidBodyTransform();

   private RigidBodyTransform goalLeftFootPose = new RigidBodyTransform();
   private RigidBodyTransform goalRightFootPose = new RigidBodyTransform();

   private double idealFootstepLength;
   private double idealFootstepWidth;

   private double maximumStepReach;
   private double maximumStepZ;
   private double minimumFootholdPercent;

   //   private FootstepPlan footstepPlan;

   private BipedalFootstepPlannerNode startNode, goalNode;

   private BipedalFootstepPlannerListener listener;

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach = maximumStepReach;
   }

   public void setMaximumStepZ(double maximumStepZ)
   {
      this.maximumStepZ = maximumStepZ;
   }

   public void setMinimumFootholdPercent(double minimumFootholdPercent)
   {
      this.minimumFootholdPercent = minimumFootholdPercent;
   }

   public void setIdealFootstep(double idealFootstepLength, double idealFootstepWidth)
   {
      this.idealFootstepLength = idealFootstepLength;
      this.idealFootstepWidth = idealFootstepWidth;
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
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
      FramePose goalPose = goal.getGoalPoseBetweenFeet();
      goalPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      goalPose.getPose(goalLeftFootPose);
      goalPose.getPose(goalRightFootPose);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;

      if (listener != null)
      {
         listener.planarRegionsListSet(planarRegionsList);
      }
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (goalNode == null)
      {
         return null;
      }

      FootstepPlan result = new FootstepPlan();

      BipedalFootstepPlannerNode node = goalNode;

      while (node != null)
      {
         RigidBodyTransform soleTransform = new RigidBodyTransform();
         node.getSoleTransform(soleTransform);

         FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), soleTransform);
         result.addFootstep(node.getRobotSide(), framePose);

         node = node.getParentNode();
      }

      result.reverse();
      return result;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      startNode = goalNode = null;

      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();
      stack.push(startNode);

      RigidBodyTransform soleZUpTransform = new RigidBodyTransform();

      Point3d goalLeftSolePosition = new Point3d();
      goalLeftFootPose.transform(goalLeftSolePosition);

      Point3d goalRightSolePosition = new Point3d();
      goalRightFootPose.transform(goalRightSolePosition);

      SideDependentList<Point3d> goalPositions = new SideDependentList<>(goalLeftSolePosition, goalRightSolePosition);

      int count = 0;
      while (!stack.isEmpty())
      {
         count++;
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         notifyListenerNodeSelectedForExpansion(nodeToExpand);

         RobotSide currentSide = nodeToExpand.getRobotSide();
         RobotSide nextSide = currentSide.getOppositeSide();

         // Make sure popped node is a good one and can be expanded...
         PlanarRegion planarRegion = new PlanarRegion();
         RigidBodyTransform nodeToExpandSnapTransform = getSnapTransform(nodeToExpand, planarRegion);

         if (nodeToExpandSnapTransform == null)
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            continue;
         }

         nodeToExpand.transformSoleTransformWithSnapTransformFromZeroZ(nodeToExpandSnapTransform, planarRegion);


         RigidBodyTransform nodeToExpandTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(nodeToExpandTransform);
         RigidBodyTransform soleTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(soleTransform);
         ConvexPolygon2d snappedPolygon = footPolygonsInSoleFrame.get(currentSide);
         snappedPolygon.update();
         double footArea = snappedPolygon.getArea();

         ArrayList<ConvexPolygon2d> intersectionsToPack = new ArrayList<>();
         planarRegion.getPolygonIntersectionsWhenSnapped(snappedPolygon, nodeToExpandTransform, intersectionsToPack);

         double totalArea = 0.0;
         for (int i=0; i<intersectionsToPack.size(); i++)
         {
            ConvexPolygon2d intersectionPolygon = intersectionsToPack.get(i);
            intersectionPolygon.update();
            totalArea = totalArea + intersectionPolygon.getArea();
         }

         if (totalArea < minimumFootholdPercent * footArea)
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            continue;
         }

         RigidBodyTransform transformToParent = nodeToExpand.getTransformToParent();
         if (transformToParent != null)
         {
            Point3d stepFromParent = new Point3d();
            transformToParent.transform(stepFromParent);
//            System.out.println("Step From parent:\n" + stepFromParent);

            Vector3d stepFromParentInWorld = new Vector3d(stepFromParent);

            RigidBodyTransform transformToWorld = new RigidBodyTransform();

            nodeToExpand.getParentNode().getSoleTransform(transformToWorld);
            transformToWorld.transform(stepFromParentInWorld);
//            System.out.println("Step From parent:\n" + stepFromParentInWorld);
//            System.out.println();

            if (Math.abs(stepFromParentInWorld.getZ()) > maximumStepZ)
            {
               notifyListenerNodeForExpansionWasRejected(nodeToExpand);
               continue;
            }
         }

         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);

         // Check if goal is reachable:
         nodeToExpand.getSoleTransform(soleZUpTransform);
         setTransformZUpPreserveX(soleZUpTransform);

         Point3d currentSolePosition = new Point3d();
         soleZUpTransform.transform(currentSolePosition);

         Point3d goalSolePosition = goalPositions.get(nextSide);

         if (goalSolePosition.distance(currentSolePosition) < maximumStepReach)
         {
            Vector3d finishStep = new Vector3d();
            finishStep.sub(goalSolePosition, currentSolePosition);

            goalNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, finishStep);

            planarRegion = new PlanarRegion();
            RigidBodyTransform goalNodeSnapTransform = getSnapTransform(goalNode, planarRegion);

            if (goalNodeSnapTransform == null)
            {
               notifyListenerNodeForExpansionWasRejected(goalNode);
            }
            else
            {
               goalNode.transformSoleTransformWithSnapTransformFromZeroZ(goalNodeSnapTransform, planarRegion);
               notifyListenerNodeForExpansionWasAccepted(goalNode);

               notifyListenerSolutionWasFound(goalNode);
               return FootstepPlanningResult.OPTIMAL_SOLUTION;
            }
         }

         expandChildrenAndAddToQueue(stack, soleZUpTransform, nodeToExpand, currentSide);
      }

      notifyListenerSolutionWasNotFound();

      return FootstepPlanningResult.NO_PATH_EXISTS;
   }

   private void expandChildrenAndAddToQueue(Deque<BipedalFootstepPlannerNode> stack, RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand, RobotSide currentSide)
   {
      ArrayList<BipedalFootstepPlannerNode> nodesToAdd = new ArrayList<>();

      Vector3d idealStep = new Vector3d(idealFootstepLength, currentSide.negateIfLeftSide(idealFootstepWidth), 0.0);
      BipedalFootstepPlannerNode childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, idealStep);
      nodesToAdd.add(childNode);

      for (double xStep = idealFootstepLength/2.0; xStep < 1.6 * idealFootstepLength; xStep = xStep + idealFootstepLength / 2.0)
      {
         for (double yStep = idealFootstepWidth; yStep < 1.4 * idealFootstepWidth; yStep = yStep + idealFootstepWidth / 4.0)
         {
            //for (double thetaStep = -0.1; thetaStep < 0.1; thetaStep = thetaStep + 0.1 / 2.0)
            {
               Vector3d nextStepVector = new Vector3d(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
               childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector);
               nodesToAdd.add(childNode);
            }
         }
      }

//      Collections.shuffle(nodesToAdd);
      Collections.reverse(nodesToAdd);

      for (BipedalFootstepPlannerNode node : nodesToAdd)
      {
         stack.push(node);
      }

   }

   private BipedalFootstepPlannerNode createAndAddNextNodeGivenStep(RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand, Vector3d idealStepVector)
   {
      RigidBodyTransform nextTransform = new RigidBodyTransform();
      RobotSide nextSide = nodeToExpand.getRobotSide().getOppositeSide();

      nextTransform.set(soleZUpTransform);

      Point3d idealStepLocation = new Point3d(idealStepVector);
      nextTransform.transform(idealStepLocation);

      nextTransform.setTranslation(idealStepLocation.getX(), idealStepLocation.getY(), idealStepLocation.getZ());

      BipedalFootstepPlannerNode childNode = new BipedalFootstepPlannerNode(nextSide, nextTransform);
      childNode.setParentNode(nodeToExpand);
      nodeToExpand.addChild(childNode);
      return childNode;
   }

   private RigidBodyTransform getSnapTransform(BipedalFootstepPlannerNode bipedalFootstepPlannerNode, PlanarRegion planarRegionToPack)
   {
      RobotSide nodeSide = bipedalFootstepPlannerNode.getRobotSide();
      RigidBodyTransform soleTransformBeforeSnap = new RigidBodyTransform();

      bipedalFootstepPlannerNode.getSoleTransform(soleTransformBeforeSnap);

      ConvexPolygon2d currentFootPolygon = new ConvexPolygon2d(footPolygonsInSoleFrame.get(nodeSide));
      currentFootPolygon.applyTransformAndProjectToXYPlane(soleTransformBeforeSnap);

      if (planarRegionsList != null)
      {
         return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(currentFootPolygon, planarRegionsList, planarRegionToPack);
      }
      else
      {
         return null;
      }
   }

   private void notifyListenerNodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeSelectedForExpansion(nodeToExpand);
      }
   }

   private void notifyListenerNodeForExpansionWasAccepted(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeForExpansionWasAccepted(nodeToExpand);
      }
   }

   private void notifyListenerNodeForExpansionWasRejected(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeForExpansionWasRejected(nodeToExpand);
      }
   }

   private void notifyListenerSolutionWasFound(BipedalFootstepPlannerNode goalNode)
   {
      if (listener != null)
      {
         listener.notifyListenerSolutionWasFound();
      }
   }

   private void notifyListenerSolutionWasNotFound()
   {
      if (listener != null)
      {
         listener.notifyListenerSolutionWasNotFound();
      }
   }

   private final Vector3d xAxis = new Vector3d();
   private final Vector3d yAxis = new Vector3d();
   private final Vector3d zAxis = new Vector3d();

   private void setTransformZUpPreserveX(RigidBodyTransform transform)
   {
      xAxis.set(transform.getM00(), transform.getM10(), 0.0);
      xAxis.normalize();
      zAxis.set(0.0, 0.0, 1.0);
      yAxis.cross(zAxis, xAxis);

      transform.setM00(xAxis.getX());
      transform.setM10(xAxis.getY());
      transform.setM20(xAxis.getZ());

      transform.setM01(yAxis.getX());
      transform.setM11(yAxis.getY());
      transform.setM21(yAxis.getZ());

      transform.setM02(zAxis.getX());
      transform.setM12(zAxis.getY());
      transform.setM22(zAxis.getZ());
   }

}
