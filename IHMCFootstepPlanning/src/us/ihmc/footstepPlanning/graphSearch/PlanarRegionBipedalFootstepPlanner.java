package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.Deque;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionBipedalFootstepPlanner
{
   private PlanarRegionsList planarRegionsList;
   private SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;

   private RobotSide initialSide;
   private RigidBodyTransform initialFootPose = new RigidBodyTransform();

   private RigidBodyTransform goalLeftFootPose = new RigidBodyTransform();
   private RigidBodyTransform goalRightFootPose = new RigidBodyTransform();

   private double idealFootstepLength;
   private double idealFootstepWidth;

   private double maxStepReach;

   private SimpleFootstepPath solution;

   private BipedalFootstepPlannerListener listener;

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;

      if (listener != null)
      {
         listener.planarRegionsListSet(planarRegionsList);
      }
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }

   public void setMaxStepReach(double maxStepReach)
   {
      this.maxStepReach = maxStepReach;
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

   public void setInitialState(RobotSide initialSide, RigidBodyTransform initialFootPose)
   {
      this.initialSide = initialSide;
      this.initialFootPose.set(initialFootPose);
   }

   public void setGoalFootsteps(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose)
   {
      this.goalLeftFootPose.set(goalLeftFootPose);
      this.goalRightFootPose.set(goalRightFootPose);
   }

   public SimpleFootstepPath getSolution()
   {
      return solution;
   }

   public boolean solve()
   {
      BipedalFootstepPlannerNode startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();
      stack.push(startNode);

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      RigidBodyTransform soleZUpTransform = new RigidBodyTransform();
      RigidBodyTransform nextTransform = new RigidBodyTransform();

      Point3d goalLeftSolePosition = new Point3d();
      goalLeftFootPose.transform(goalLeftSolePosition);

      Point3d goalRightSolePosition = new Point3d();
      goalRightFootPose.transform(goalRightSolePosition);

      SideDependentList<Point3d> goalPositions = new SideDependentList<>(goalLeftSolePosition, goalRightSolePosition);

      while (!stack.isEmpty())
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         notifyListenerNodeSelectedForExpansion(nodeToExpand);

         RobotSide currentSide = nodeToExpand.getSide();
         RobotSide nextSide = currentSide.getOppositeSide();

         // Make sure popped node is a good one and can be expanded...
         nodeToExpand.getSoleTransform(soleTransform);
         ConvexPolygon2d currentFootPolygon = new ConvexPolygon2d(footPolygonsInSoleFrame.get(currentSide));

         currentFootPolygon.applyTransformAndProjectToXYPlane(soleTransform);
         RigidBodyTransform nodeToExpandSnapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(currentFootPolygon, planarRegionsList);

         if (nodeToExpandSnapTransform == null)
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            continue;
         }

         nodeToExpand.transformSoleTransform(nodeToExpandSnapTransform);
         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);

         soleZUpTransform.set(soleTransform);
         setTransformZUpPreserveX(soleZUpTransform);

         Point3d idealStep = new Point3d(idealFootstepLength, currentSide.negateIfLeftSide(idealFootstepWidth), 0.0);
         nextTransform.set(soleZUpTransform);
         nextTransform.transform(idealStep);

         Point3d currentSolePosition = new Point3d();
         soleTransform.transform(currentSolePosition);

         Point3d goalSolePosition = goalPositions.get(nextSide);

         if (goalSolePosition.distance(currentSolePosition) < maxStepReach)
         {
            Vector3d finishStep = new Vector3d();
            finishStep.sub(goalSolePosition, currentSolePosition);
            nextTransform.setTranslation(idealStep.getX(), idealStep.getY(), idealStep.getZ());
            BipedalFootstepPlannerNode childNode = new BipedalFootstepPlannerNode(nextSide, nextTransform);
            nodeToExpand.addChild(childNode);

            notifyListenerSolutionWasFound();

            return true;
         }

         // Expand Children...
         nextTransform.setTranslation(idealStep.getX(), idealStep.getY(), idealStep.getZ());

         BipedalFootstepPlannerNode childNode = new BipedalFootstepPlannerNode(nextSide, nextTransform);
         nodeToExpand.addChild(childNode);

         stack.add(childNode);
      }

      notifyListenerSolutionWasNotFound();

      return false;
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

   private void notifyListenerSolutionWasFound()
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
