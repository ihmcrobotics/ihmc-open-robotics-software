package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
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

   private SideDependentList<RigidBodyTransform> goalFootstepPoses;
   private SideDependentList<Point3d> goalPositions;
   private SideDependentList<Double> goalYaws;

   private double idealFootstepLength;
   private double idealFootstepWidth;

   private double maximumStepReach;
   private double maximumStepZ;
   private double maximumStepYaw;
   private double minimumStepWidth;

   private double minimumFootholdPercent;

   private double maximumXYWiggleDistance = 0.1;
   private double maximumYawWiggle = 0.1;

   private double minimumSurfaceNormalZ = 0.7;
   private double maximumZPenetrationOnVRegions = 0.01;

   private int maximumNumberOfNodesToExpand;
   //   private FootstepPlan footstepPlan;

   private BipedalFootstepPlannerNode startNode, goalNode;

   private BipedalFootstepPlannerListener listener;

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }

   public void setMaximumNumberOfNodesToExpand(int maximumNumberOfNodesToExpand)
   {
      this.maximumNumberOfNodesToExpand = maximumNumberOfNodesToExpand;
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach = maximumStepReach;
   }

   public void setMaximumStepZ(double maximumStepZ)
   {
      this.maximumStepZ = maximumStepZ;
   }

   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw = maximumStepYaw;
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth = minimumStepWidth;
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

      RigidBodyTransform goalLeftFootPose = new RigidBodyTransform();
      RigidBodyTransform goalRightFootPose = new RigidBodyTransform();

      goalPose.getPose(goalLeftFootPose);
      goalPose.getPose(goalRightFootPose);

      Vector3d toTheLeft = new Vector3d(0.0, 0.15, 0.0);
      Vector3d toTheRight = new Vector3d(0.0, -0.15, 0.0);

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
      goalNode = null;

      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();
      stack.push(startNode);

      int numberOfNodesExpanded = 0;
      while ((!stack.isEmpty()) && (numberOfNodesExpanded < maximumNumberOfNodesToExpand))
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         notifyListenerNodeSelectedForExpansion(nodeToExpand);

         // Make sure popped node is a good one and can be expanded...
         boolean snapSucceded = snapToPlanarRegionAndCheckIfGoodSnap(nodeToExpand);
         if (!snapSucceded)
            continue;

         boolean goodFootstep = checkIfGoodFootstep(nodeToExpand);
         if (!goodFootstep)
            continue;

         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);
         numberOfNodesExpanded++;

         // Check if at goal:

         if (nodeToExpand.isAtGoal())
         {
            if ((nodeToExpand.getParentNode() != null) && (nodeToExpand.getParentNode().isAtGoal()))
            {
               goalNode = nodeToExpand;
               notifyListenerSolutionWasFound(goalNode);
               return FootstepPlanningResult.OPTIMAL_SOLUTION;
            }
         }

         RigidBodyTransform soleZUpTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(soleZUpTransform);
         setTransformZUpPreserveX(soleZUpTransform);

         // Check if goal is reachable:
         boolean goalIsReachable = addGoalNodeIfGoalIsReachable(nodeToExpand, soleZUpTransform, stack);
         if (goalIsReachable)
            continue;

         expandChildrenAndAddToQueue(stack, soleZUpTransform, nodeToExpand);
      }

      notifyListenerSolutionWasNotFound();
      return FootstepPlanningResult.NO_PATH_EXISTS;
   }

   private boolean addGoalNodeIfGoalIsReachable(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform, Deque<BipedalFootstepPlannerNode> stack)
   {
      Point3d currentSolePosition = nodeToExpand.getSolePosition();

      RobotSide currentSide = nodeToExpand.getRobotSide();
      RobotSide nextSide = currentSide.getOppositeSide();

      Point3d goalSolePosition = goalPositions.get(nextSide);

      if (goalSolePosition.distance(currentSolePosition) < maximumStepReach)
      {
         double currentSoleYaw = nodeToExpand.getSoleYaw();
         double goalSoleYaw = goalYaws.get(nextSide);

         double stepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(goalSoleYaw, currentSoleYaw);

         if (Math.abs(stepYaw) < maximumStepYaw)
         {
            Vector3d finishStep = new Vector3d();
            finishStep.sub(goalSolePosition, currentSolePosition);

            RigidBodyTransform inverseTransform = new RigidBodyTransform();
            nodeToExpand.getSoleTransform(inverseTransform);
            inverseTransform.invert();
            inverseTransform.transform(finishStep);

            BipedalFootstepPlannerNode goalNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, finishStep, stepYaw);
            goalNode.setIsAtGoal();

            stack.push(goalNode);
            return true;
         }
      }

      return false;
   }

   private boolean checkIfGoodFootstep(BipedalFootstepPlannerNode nodeToExpand)
   {
      RobotSide robotSide = nodeToExpand.getRobotSide();

      RigidBodyTransform transformToParent = nodeToExpand.getTransformToParent();
      if (transformToParent != null)
      {
         Point3d stepFromParent = new Point3d();
         transformToParent.transform(stepFromParent);

         if (((robotSide == RobotSide.LEFT) && (stepFromParent.getY() < minimumStepWidth)) || ((robotSide == RobotSide.RIGHT) && (stepFromParent.getY() > -minimumStepWidth)))
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            return false;
         }

         Vector3d stepFromParentInWorld = new Vector3d(stepFromParent);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();

         nodeToExpand.getParentNode().getSoleTransform(transformToWorld);
         transformToWorld.transform(stepFromParentInWorld);

         if (Math.abs(stepFromParentInWorld.getZ()) > maximumStepZ)
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            return false;
         }

         if (stepFromParentInWorld.length() > maximumStepReach)
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            return false;
         }
      }

      return true;
   }

   private boolean snapToPlanarRegionAndCheckIfGoodSnap(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (planarRegionsList != null)
      {
         PlanarRegion planarRegion = new PlanarRegion();
         RigidBodyTransform nodeToExpandSnapTransform = getSnapAndWiggleTransform(nodeToExpand, planarRegion);

         if (nodeToExpandSnapTransform == null)
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            return false;
         }

         nodeToExpand.transformSoleTransformWithSnapTransformFromZeroZ(nodeToExpandSnapTransform, planarRegion);

         RigidBodyTransform nodeToExpandTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(nodeToExpandTransform);
         RigidBodyTransform soleTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(soleTransform);
         ConvexPolygon2d snappedPolygon = footPolygonsInSoleFrame.get(nodeToExpand.getRobotSide());
         snappedPolygon.update();
         double footArea = snappedPolygon.getArea();

         ArrayList<ConvexPolygon2d> polygonIntersectionsOnPlanarRegion = new ArrayList<>();
         planarRegion.getPolygonIntersectionsWhenSnapped(snappedPolygon, nodeToExpandTransform, polygonIntersectionsOnPlanarRegion);

         double totalArea = 0.0;
         for (int i = 0; i < polygonIntersectionsOnPlanarRegion.size(); i++)
         {
            ConvexPolygon2d intersectionPolygon = polygonIntersectionsOnPlanarRegion.get(i);
            intersectionPolygon.update();
            totalArea = totalArea + intersectionPolygon.getArea();
         }

         if (totalArea < minimumFootholdPercent * footArea)
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand);
            return false;
         }
      }

      return true;
   }

   private void expandChildrenAndAddToQueue(Deque<BipedalFootstepPlannerNode> stack, RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand)
   {
      ArrayList<BipedalFootstepPlannerNode> nodesToAdd = new ArrayList<>();

      RobotSide currentSide = nodeToExpand.getRobotSide();
      RobotSide nextSide = currentSide.getOppositeSide();

      Point3d goalPosition = goalPositions.get(nextSide);
      Point3d currentPosition = nodeToExpand.getSolePosition();
      Vector3d currentToGoalVector = new Vector3d();
      currentToGoalVector.sub(goalPosition, currentPosition);

      double distance = currentToGoalVector.length();

      Vector3d idealStepVector;

      if (distance > 2.0 * maximumStepReach)
      {
         idealStepVector = new Vector3d(idealFootstepLength, currentSide.negateIfLeftSide(idealFootstepWidth), 0.0);
      }
      else
      {
         idealStepVector = new Vector3d(currentToGoalVector);
         RigidBodyTransform inverseTransform = new RigidBodyTransform(soleZUpTransform);
         inverseTransform.invert();
         inverseTransform.transform(idealStepVector);
      }

      if (idealStepVector.length() > maximumStepReach)
      {
         idealStepVector.scale(maximumStepReach / idealStepVector.length());
      }

      if ((nextSide == RobotSide.LEFT) && (idealStepVector.getY() < minimumStepWidth))
      {
         idealStepVector.setY(minimumStepWidth);
      }
      else if ((nextSide == RobotSide.RIGHT) && (idealStepVector.getY() > -minimumStepWidth))
      {
         idealStepVector.setY(-minimumStepWidth);
      }

      Point3d idealStepLocationInWorld = new Point3d(idealStepVector);
      soleZUpTransform.transform(idealStepLocationInWorld);

      Vector3d vectorToGoal = new Vector3d();
      vectorToGoal.sub(goalPosition, idealStepLocationInWorld);

      Vector3d currentRotationEulerInWorld = new Vector3d();
      soleZUpTransform.getRotationEuler(currentRotationEulerInWorld);
      double currentYaw = currentRotationEulerInWorld.getZ();

      double idealYawInWorld;

      if (distance > 2.0 * maximumStepReach)
      {
         idealYawInWorld = Math.atan2(vectorToGoal.getY(), vectorToGoal.getX());
      }
      else
      {
         idealYawInWorld = goalYaws.get(nextSide);
      }

      double idealStepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(idealYawInWorld, currentYaw);
      idealStepYaw = MathTools.clipToMinMax(idealStepYaw, maximumStepYaw);

      BipedalFootstepPlannerNode childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, idealStepVector, idealStepYaw);
      nodesToAdd.add(childNode);

      for (double xStep = idealFootstepLength / 2.0; xStep < 1.6 * idealFootstepLength; xStep = xStep + idealFootstepLength / 2.0)
      {
         for (double yStep = idealFootstepWidth; yStep < 1.4 * idealFootstepWidth; yStep = yStep + idealFootstepWidth / 4.0)
         {
            //for (double thetaStep = -0.1; thetaStep < 0.1; thetaStep = thetaStep + 0.1 / 2.0)
            {
               double nextStepYaw = idealStepYaw;
               Vector3d nextStepVector = new Vector3d(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
               childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector, nextStepYaw);
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

   private BipedalFootstepPlannerNode createAndAddNextNodeGivenStep(RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand, Vector3d stepVectorInSoleFrame, double stepYaw)
   {
      Point3d stepLocationInWorld = new Point3d(stepVectorInSoleFrame);
      soleZUpTransform.transform(stepLocationInWorld);

      Vector3d stepRotationEulerInWorld = new Vector3d();
      soleZUpTransform.getRotationEuler(stepRotationEulerInWorld);
      //      stepRotationEulerInWorld.setZ((stepRotationEulerInWorld.getZ() + stepYaw + 2.0 * Math.PI) % Math.PI);
      stepRotationEulerInWorld.setZ(stepRotationEulerInWorld.getZ() + stepYaw);

      RigidBodyTransform nextTransform = new RigidBodyTransform();
      nextTransform.setRotationEulerAndZeroTranslation(stepRotationEulerInWorld);
      nextTransform.setTranslation(stepLocationInWorld.getX(), stepLocationInWorld.getY(), stepLocationInWorld.getZ());

      RobotSide nextSide = nodeToExpand.getRobotSide().getOppositeSide();

      BipedalFootstepPlannerNode childNode = new BipedalFootstepPlannerNode(nextSide, nextTransform);
      childNode.setParentNode(nodeToExpand);
      nodeToExpand.addChild(childNode);
      return childNode;
   }

   private RigidBodyTransform getSnapAndWiggleTransform(BipedalFootstepPlannerNode bipedalFootstepPlannerNode, PlanarRegion planarRegionToPack)
   {
      if (planarRegionsList == null)
      {
         throw new RuntimeException("Only call this if planarRegionsList exists. Check for null before calling.");
      }

      RobotSide nodeSide = bipedalFootstepPlannerNode.getRobotSide();
      RigidBodyTransform soleTransformBeforeSnap = new RigidBodyTransform();

      bipedalFootstepPlannerNode.getSoleTransform(soleTransformBeforeSnap);

      ConvexPolygon2d currentFootPolygon = new ConvexPolygon2d(footPolygonsInSoleFrame.get(nodeSide));
      currentFootPolygon.applyTransformAndProjectToXYPlane(soleTransformBeforeSnap);

      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(currentFootPolygon, planarRegionsList, planarRegionToPack);
      if (snapTransform == null) return null;

      if (Math.abs(snapTransform.getM22()) < minimumSurfaceNormalZ) return null;

      WiggleParameters parameters = new WiggleParameters();
      parameters.minX = -0.1;
      parameters.maxX = 0.1;
      parameters.minY = -0.1;
      parameters.maxY = 0.1;
      parameters.minYaw = -0.1;
      parameters.maxYaw = 0.1;
      parameters.rotationWeight = 1.0;

      ConvexPolygon2d polygonToWiggleInRegionFrame = planarRegionToPack.snapPolygonIntoRegionAndChangeFrameToRegionFrame(currentFootPolygon, snapTransform);
      RigidBodyTransform wiggleTransform = PolygonWiggler.wigglePolygonIntoRegion(polygonToWiggleInRegionFrame, planarRegionToPack, parameters);
      if (wiggleTransform == null) return snapTransform;

//      System.out.println("wiggleTransform = \n" + wiggleTransform);

      Point3d wiggleTranslation = new Point3d();
      wiggleTransform.transform(wiggleTranslation);
      Vector3d wiggleVector = new Vector3d(wiggleTranslation);
      if (wiggleVector.length() > maximumXYWiggleDistance)
      {
         wiggleVector.scale(maximumXYWiggleDistance/wiggleVector.length());
      }

      Vector3d rotationEuler = new Vector3d();
      wiggleTransform.getRotationEuler(rotationEuler);
      double yaw = rotationEuler.getZ();
      yaw = MathTools.clipToMinMax(yaw, maximumYawWiggle);

      rotationEuler.setZ(yaw);
      wiggleTransform.setRotationEulerAndZeroTranslation(rotationEuler);
      wiggleTransform.setTranslation(wiggleVector);

//      System.out.println("Limited wiggleTransform = \n" + wiggleTransform);

      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform();
      snapAndWiggleTransform.multiply(snapTransform, wiggleTransform);


      // Ensure polygon will be completely above the planarRegions with this snap and wiggle:
      ConvexPolygon2d checkFootPolygonInWorld = new ConvexPolygon2d(currentFootPolygon);
      checkFootPolygonInWorld.applyTransformAndProjectToXYPlane(snapAndWiggleTransform);

      List<PlanarRegion> planarRegionsIntersectingSnappedAndWiggledPolygon = planarRegionsList.findPlanarRegionsIntersectingPolygon(checkFootPolygonInWorld);

      ArrayList<ConvexPolygon2d> intersectionsInPlaneFrameToPack = new ArrayList<>();
      RigidBodyTransform transformToWorldFromIntersectingPlanarRegion = new RigidBodyTransform();

      for (PlanarRegion planarRegionIntersectingSnappedAndWiggledPolygon : planarRegionsIntersectingSnappedAndWiggledPolygon)
      {
         planarRegionIntersectingSnappedAndWiggledPolygon.getTransformToWorld(transformToWorldFromIntersectingPlanarRegion);
         intersectionsInPlaneFrameToPack.clear();
         planarRegionIntersectingSnappedAndWiggledPolygon.getPolygonIntersectionsWhenProjectedVertically(checkFootPolygonInWorld, intersectionsInPlaneFrameToPack);

         // If any points are above the plane of the planarRegionToPack, then this is stepping into a v type problem.
         for (ConvexPolygon2d intersectionPolygon : intersectionsInPlaneFrameToPack)
         {
            int numberOfVertices = intersectionPolygon.getNumberOfVertices();
            for (int i=0; i<numberOfVertices; i++)
            {
               Point2d vertex2d = intersectionPolygon.getVertex(i);
               Point3d vertex3dInWorld = new Point3d(vertex2d.getX(), vertex2d.getY(), 0.0);
               transformToWorldFromIntersectingPlanarRegion.transform(vertex3dInWorld);

               double planeZGivenXY = planarRegionToPack.getPlaneZGivenXY(vertex3dInWorld.getX(), vertex3dInWorld.getY());

               double zPenetration = vertex3dInWorld.getZ() - planeZGivenXY;
               if (zPenetration >  + maximumZPenetrationOnVRegions)
               {
//                  System.out.println("zPenetration = " + zPenetration);
                  return null;
               }
            }
         }
      }


      return snapAndWiggleTransform;
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
