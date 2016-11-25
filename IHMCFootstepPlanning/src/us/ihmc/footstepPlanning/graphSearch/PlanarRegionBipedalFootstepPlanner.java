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
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
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

   protected final DoubleYoVariable maximumStepReach = new DoubleYoVariable("maximumStepReach", registry);
   protected final DoubleYoVariable minimumFootholdPercent = new DoubleYoVariable("minimumFootholdPercent", registry);

   protected final DoubleYoVariable idealFootstepLength = new DoubleYoVariable("idealFootstepLength", registry);
   protected final DoubleYoVariable idealFootstepWidth = new DoubleYoVariable("idealFootstepWidth", registry);

   protected final DoubleYoVariable maximumStepZ = new DoubleYoVariable("maximumStepZ", registry);
   protected final DoubleYoVariable maximumStepYaw = new DoubleYoVariable("maximumStepYaw", registry);
   protected final DoubleYoVariable minimumStepWidth = new DoubleYoVariable("minimumStepWidth", registry);

   protected final DoubleYoVariable wiggleInsideDelta = new DoubleYoVariable("wiggleInsideDelta", registry);

   protected final IntegerYoVariable numberOfNodesExpanded = new IntegerYoVariable("numberOfNodesExpanded", registry);

   protected final DoubleYoVariable maximumXYWiggleDistance = new DoubleYoVariable("maximumXYWiggleDistance", registry);
   protected final DoubleYoVariable maximumYawWiggle = new DoubleYoVariable("maximumYawWiggle", registry);

   protected final DoubleYoVariable footArea = new DoubleYoVariable("footArea", registry);
   protected final DoubleYoVariable totalArea = new DoubleYoVariable("totalArea", registry);

   protected double minimumSurfaceNormalZ = 0.7;
   protected double maximumZPenetrationOnVRegions = 0.008;

   private int maximumNumberOfNodesToExpand;
   //   private FootstepPlan footstepPlan;

   protected BipedalFootstepPlannerNode startNode, goalNode;

   protected BipedalFootstepPlannerListener listener;

   public PlanarRegionBipedalFootstepPlanner(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      maximumXYWiggleDistance.set(0.1);
      maximumYawWiggle.set(0.1);
   }

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
      this.maximumStepReach.set(maximumStepReach);
   }

   public void setMaximumStepZ(double maximumStepZ)
   {
      this.maximumStepZ.set(maximumStepZ);
   }

   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw.set(maximumStepYaw);
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth.set(minimumStepWidth);
   }

   public void setMinimumFootholdPercent(double minimumFootholdPercent)
   {
      this.minimumFootholdPercent.set(minimumFootholdPercent);
   }

   public void setIdealFootstep(double idealFootstepLength, double idealFootstepWidth)
   {
      this.idealFootstepLength.set(idealFootstepLength);
      this.idealFootstepWidth.set(idealFootstepWidth);
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      this.wiggleInsideDelta.set(wiggleInsideDelta);
   }

   public void setMaximumXYWiggleDistance(double maximumXYWiggleDistance)
   {
      this.maximumXYWiggleDistance.set(maximumXYWiggleDistance);
   }

   public void setMaximumYawWiggle(double maximumYawWiggle)
   {
      this.maximumYawWiggle.set(maximumYawWiggle);
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
      footstepPlannerGoalType = goal.getFootstepPlannerGoalType();

      setGoalXYAndRadius(goal);
      setGoalPositionsAndYaws(goal);
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

      if(idealFootstepWidth.getDoubleValue() == 0.0)
      {
         toTheLeft = new Vector3d(0.0, 0.15, 0.0);
         toTheRight = new Vector3d(0.0, -0.15, 0.0);
      }
      else
      {
         toTheLeft = new Vector3d(0.0, idealFootstepWidth.getDoubleValue()/2.0, 0.0);
         toTheRight = new Vector3d(0.0, - idealFootstepWidth.getDoubleValue()/2.0, 0.0);
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

      FootstepPlan result = new FootstepPlan(goalNode);
      return result;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      goalNode = null;

      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      Deque<BipedalFootstepPlannerNode> stack = new ArrayDeque<BipedalFootstepPlannerNode>();
      stack.push(startNode);

      numberOfNodesExpanded.set(0);
      while ((!stack.isEmpty()) && (numberOfNodesExpanded.getIntegerValue() < maximumNumberOfNodesToExpand))
      {
         BipedalFootstepPlannerNode nodeToExpand = stack.pop();
         notifyListenerNodeSelectedForExpansion(nodeToExpand);

         if (nodeToExpand != startNode) // StartNode is from an actual footstep, so we don't need to snap it...
         {
            // Make sure popped node is a good one and can be expanded...
            boolean snapSucceded = snapToPlanarRegionAndCheckIfGoodSnap(wiggleInsideDelta.getDoubleValue(), nodeToExpand);
            if (!snapSucceded)
               continue;

            boolean goodFootstep = checkIfGoodFootstep(nodeToExpand);
            if (!goodFootstep)
               continue;

            boolean differentFromParent = checkIfDifferentFromGrandParent(nodeToExpand);
            {
               if (!differentFromParent)
                  continue;
            }
         }

         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);
         numberOfNodesExpanded.increment();

         // Check if at goal:

         if (nodeToExpand.isAtGoal())
         {
//            System.out.println("Expanding is at goal!!!");
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

   protected boolean addGoalNodeIfGoalIsReachable(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform, Deque<BipedalFootstepPlannerNode> stack)
   {
      BipedalFootstepPlannerNode goalNode = null;

      if (footstepPlannerGoalType == FootstepPlannerGoalType.CLOSE_TO_XY_POSITION)
      {
         goalNode = findGoalNodeUsingCloseToXY(nodeToExpand, soleZUpTransform, stack);
      }
      else
      {
         goalNode = findGoalNodeUsingSolePositions(nodeToExpand, soleZUpTransform);
      }

      if (goalNode != null)
      {
         stack.push(goalNode);
         return true;
      }

      return false;
   }

   private BipedalFootstepPlannerNode findGoalNodeUsingCloseToXY(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform, Deque<BipedalFootstepPlannerNode> stack)
   {
      return null;
   }

   private BipedalFootstepPlannerNode findGoalNodeUsingSolePositions(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
   {
      Point3d currentSolePosition = nodeToExpand.getSolePosition();

      RobotSide currentSide = nodeToExpand.getRobotSide();
      RobotSide nextSide = currentSide.getOppositeSide();

      Point3d goalSolePosition = goalPositions.get(nextSide);

      if (goalSolePosition.distance(currentSolePosition) < maximumStepReach.getDoubleValue())
      {
         double currentSoleYaw = nodeToExpand.getSoleYaw();
         double goalSoleYaw = goalYaws.get(nextSide);

         double stepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(goalSoleYaw, currentSoleYaw);

         if (Math.abs(stepYaw) < maximumStepYaw.getDoubleValue())
         {
            Vector3d finishStep = new Vector3d();
            finishStep.sub(goalSolePosition, currentSolePosition);

            RigidBodyTransform inverseTransform = new RigidBodyTransform();
            nodeToExpand.getSoleTransform(inverseTransform);
            inverseTransform.invert();
            inverseTransform.transform(finishStep);

            BipedalFootstepPlannerNode goalNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, finishStep, stepYaw);
            goalNode.setIsAtGoal();

            return goalNode;
         }
      }

      return null;
   }

   private boolean checkIfDifferentFromGrandParent(BipedalFootstepPlannerNode nodeToExpand)
   {
      BipedalFootstepPlannerNode parentNode = nodeToExpand.getParentNode();
      if (parentNode == null) return true;

      BipedalFootstepPlannerNode grandParentNode = parentNode.getParentNode();
      if (grandParentNode == null) return true;

      if (grandParentNode.epsilonEquals(nodeToExpand, 1e-1))
      {
         notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_IN_PLACE);
         return false;
      }

      return true;
   }

   protected boolean checkIfGoodFootstep(BipedalFootstepPlannerNode nodeToExpand)
   {
      RobotSide robotSide = nodeToExpand.getRobotSide();

      RigidBodyTransform transformToParent = nodeToExpand.getTransformToParent();
      if (transformToParent != null)
      {
         Point3d stepFromParent = new Point3d();
         transformToParent.transform(stepFromParent);

         if (((robotSide == RobotSide.LEFT) && (stepFromParent.getY() < minimumStepWidth.getDoubleValue())) || ((robotSide == RobotSide.RIGHT) && (stepFromParent.getY() > -minimumStepWidth.getDoubleValue())))
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH);
            return false;
         }

         Vector3d stepFromParentInWorld = new Vector3d(stepFromParent);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();

         nodeToExpand.getParentNode().getSoleTransform(transformToWorld);
         transformToWorld.transform(stepFromParentInWorld);

         if (Math.abs(stepFromParentInWorld.getZ()) > maximumStepZ.getDoubleValue())
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
            return false;
         }

         if (stepFromParentInWorld.length() > maximumStepReach.getDoubleValue())
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR);
            return false;
         }
      }

      return true;
   }

   protected boolean snapToPlanarRegionAndCheckIfGoodSnap(double wiggleInsideDelta, BipedalFootstepPlannerNode nodeToExpand)
   {
      if (planarRegionsList != null)
      {
         PlanarRegion planarRegion = new PlanarRegion();
         RigidBodyTransform nodeToExpandSnapTransform = getSnapAndWiggleTransform(wiggleInsideDelta, nodeToExpand, planarRegion);

         if (nodeToExpandSnapTransform == null)
         {
//            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.BAD_SNAP_OR_WIGGLE);
            return false;
         }

         nodeToExpand.transformSoleTransformWithSnapTransformFromZeroZ(nodeToExpandSnapTransform, planarRegion);

         RigidBodyTransform nodeToExpandTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(nodeToExpandTransform);
         RigidBodyTransform soleTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(soleTransform);
         ConvexPolygon2d snappedPolygon = footPolygonsInSoleFrame.get(nodeToExpand.getRobotSide());
         snappedPolygon.update();
         footArea.set(snappedPolygon.getArea());

         ArrayList<ConvexPolygon2d> polygonIntersectionsOnPlanarRegion = new ArrayList<>();
         planarRegion.getPolygonIntersectionsWhenSnapped(snappedPolygon, nodeToExpandTransform, polygonIntersectionsOnPlanarRegion);

         totalArea.set(0.0);
         for (int i = 0; i < polygonIntersectionsOnPlanarRegion.size(); i++)
         {
            ConvexPolygon2d intersectionPolygon = polygonIntersectionsOnPlanarRegion.get(i);
            intersectionPolygon.update();
            totalArea.add(intersectionPolygon.getArea());
         }

         if (totalArea.getDoubleValue() < minimumFootholdPercent.getDoubleValue() * footArea.getDoubleValue())
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
            return false;
         }
      }

      return true;
   }

   protected void expandChildrenAndAddToQueue(Deque<BipedalFootstepPlannerNode> stack, RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand)
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

      if (distance > 2.0 * maximumStepReach.getDoubleValue())
      {
         idealStepVector = new Vector3d(idealFootstepLength.getDoubleValue(), currentSide.negateIfLeftSide(idealFootstepWidth.getDoubleValue()), 0.0);
      }
      else
      {
         idealStepVector = new Vector3d(currentToGoalVector);
         RigidBodyTransform inverseTransform = new RigidBodyTransform(soleZUpTransform);
         inverseTransform.invert();
         inverseTransform.transform(idealStepVector);
      }

      if (idealStepVector.length() > maximumStepReach.getDoubleValue())
      {
         idealStepVector.scale(maximumStepReach.getDoubleValue() / idealStepVector.length());
      }

      if ((nextSide == RobotSide.LEFT) && (idealStepVector.getY() < minimumStepWidth.getDoubleValue()))
      {
         idealStepVector.setY(minimumStepWidth.getDoubleValue());
      }
      else if ((nextSide == RobotSide.RIGHT) && (idealStepVector.getY() > -minimumStepWidth.getDoubleValue()))
      {
         idealStepVector.setY(-minimumStepWidth.getDoubleValue());
      }

      Point3d idealStepLocationInWorld = new Point3d(idealStepVector);
      soleZUpTransform.transform(idealStepLocationInWorld);

      Vector3d vectorToGoal = new Vector3d();
      vectorToGoal.sub(goalPosition, idealStepLocationInWorld);

      Vector3d currentRotationEulerInWorld = new Vector3d();
      soleZUpTransform.getRotationEuler(currentRotationEulerInWorld);
      double currentYaw = currentRotationEulerInWorld.getZ();

      double idealYawInWorld;

      if (distance > 2.0 * maximumStepReach.getDoubleValue())
      {
         idealYawInWorld = Math.atan2(vectorToGoal.getY(), vectorToGoal.getX());
      }
      else
      {
         idealYawInWorld = goalYaws.get(nextSide);
      }

      double idealStepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(idealYawInWorld, currentYaw);
      idealStepYaw = MathTools.clipToMinMax(idealStepYaw, maximumStepYaw.getDoubleValue());

      BipedalFootstepPlannerNode childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, idealStepVector, idealStepYaw);
      seeIfNodeIsAtGoal(childNode);
      nodesToAdd.add(childNode);

      for (double xStep = idealFootstepLength.getDoubleValue() / 2.0; xStep < 1.6 * idealFootstepLength.getDoubleValue(); xStep = xStep + idealFootstepLength.getDoubleValue() / 4.0)
      {
         for (double yStep = idealFootstepWidth.getDoubleValue(); yStep < 1.6 * idealFootstepWidth.getDoubleValue(); yStep = yStep + idealFootstepWidth.getDoubleValue() / 4.0)
         {
            //for (double thetaStep = -0.1; thetaStep < 0.1; thetaStep = thetaStep + 0.1 / 2.0)
            {
               double nextStepYaw = idealStepYaw;
               Vector3d nextStepVector = new Vector3d(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
               childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector, nextStepYaw);

               seeIfNodeIsAtGoal(childNode);
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

   private void seeIfNodeIsAtGoal(BipedalFootstepPlannerNode childNode)
   {
      if (footstepPlannerGoalType == FootstepPlannerGoalType.CLOSE_TO_XY_POSITION)
      {
         Point3d solePosition = childNode.getSolePosition();

         double deltaX = solePosition.getX() - xyGoal.getX();
         double deltaY = solePosition.getY() - xyGoal.getY();
         double distanceSquared = deltaX * deltaX + deltaY * deltaY;
         double distanceFromXYGoalSquared = distanceFromXYGoal * distanceFromXYGoal;

//                  System.out.println("distanceSquared = " + distanceSquared);
//                  System.out.println("distanceFromXYGoalSquared = " + distanceFromXYGoalSquared);
         if (distanceSquared < distanceFromXYGoalSquared)
         {
//                     System.out.println("Setting at goal for child node!");
            childNode.setIsAtGoal();
         }
      }
   }

   protected BipedalFootstepPlannerNode createAndAddNextNodeGivenStep(RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand, Vector3d stepVectorInSoleFrame, double stepYaw)
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

   protected RigidBodyTransform getSnapAndWiggleTransform(double wiggleInsideDelta, BipedalFootstepPlannerNode bipedalFootstepPlannerNode, PlanarRegion planarRegionToPack)
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
      if (snapTransform == null)
      {
         notifyListenerNodeForExpansionWasRejected(bipedalFootstepPlannerNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return null;
      }

      if (Math.abs(snapTransform.getM22()) < minimumSurfaceNormalZ)
      {
         notifyListenerNodeForExpansionWasRejected(bipedalFootstepPlannerNode, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return null;
      }

      BipedalFootstepPlannerNode nodeAfterSnap = new BipedalFootstepPlannerNode(bipedalFootstepPlannerNode);
      nodeAfterSnap.transformSoleTransformWithSnapTransformFromZeroZ(snapTransform, planarRegionToPack);
      notifyListenerNodeSnappedAndStillSelectedForExpansion(nodeAfterSnap);

      WiggleParameters parameters = new WiggleParameters();
      parameters.deltaInside = wiggleInsideDelta;
//      parameters.minX = -0.1;
//      parameters.maxX = 0.1;
//      parameters.minY = -0.1;
//      parameters.maxY = 0.1;
//      parameters.minYaw = -0.1;
//      parameters.maxYaw = 0.1;
//      parameters.rotationWeight = 1.0;

      ConvexPolygon2d polygonToWiggleInRegionFrame = planarRegionToPack.snapPolygonIntoRegionAndChangeFrameToRegionFrame(currentFootPolygon, snapTransform);
//      System.out.println("polygonToWiggleInRegionFrame = \n" + polygonToWiggleInRegionFrame);
//      System.out.println("planarRegionToPack = \n" + planarRegionToPack);

      RigidBodyTransform wiggleTransformLocalToLocal = PolygonWiggler.wigglePolygonIntoRegion(polygonToWiggleInRegionFrame, planarRegionToPack, parameters);
      if (wiggleTransformLocalToLocal == null)
      {
         notifyListenerNodeForExpansionWasRejected(nodeAfterSnap, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_WIGGLE_INSIDE);
         return null;

         //TODO: Make a parameter of whether to fail if cannot wiggle fully inside.
         //TODO: Possibly have different node scores depending on how firm on ground they are.
//         return snapTransform;
      }

//      System.out.println("wiggleTransformLocalToLocal = \n" + wiggleTransformLocalToLocal);

//      wiggleTransform = new RigidBodyTransform();
//      wiggleTransform.setTranslation(0.2, 0.0, 0.0);

      Point3d wiggleTranslation = new Point3d();
      wiggleTransformLocalToLocal.transform(wiggleTranslation);
      Vector3d wiggleVector = new Vector3d(wiggleTranslation);
      if (wiggleVector.length() > maximumXYWiggleDistance.getDoubleValue())
      {
         wiggleVector.scale(maximumXYWiggleDistance.getDoubleValue()/wiggleVector.length());
      }

      Vector3d rotationEuler = new Vector3d();
      wiggleTransformLocalToLocal.getRotationEuler(rotationEuler);
      double yaw = rotationEuler.getZ();
      yaw = MathTools.clipToMinMax(yaw, maximumYawWiggle.getDoubleValue());

      rotationEuler.setZ(yaw);
      wiggleTransformLocalToLocal.setRotationEulerAndZeroTranslation(rotationEuler);
      wiggleTransformLocalToLocal.setTranslation(wiggleVector);

//      System.out.println("Limited wiggleTransformLocalToLocal = \n" + wiggleTransformLocalToLocal);


      RigidBodyTransform wiggleTransformWorldToWorld = new RigidBodyTransform();
      RigidBodyTransform transformOne = new RigidBodyTransform();
      planarRegionToPack.getTransformToWorld(transformOne);
      RigidBodyTransform transformTwo = new RigidBodyTransform(transformOne);
      transformTwo.invert();

      wiggleTransformWorldToWorld.multiply(transformOne, wiggleTransformLocalToLocal);
      wiggleTransformWorldToWorld.multiply(wiggleTransformWorldToWorld, transformTwo);

//      System.out.println("wiggleTransformWorldToWorld = \n" + wiggleTransformWorldToWorld);


      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform();
      snapAndWiggleTransform.multiply(wiggleTransformWorldToWorld, snapTransform);

      // Ensure polygon will be completely above the planarRegions with this snap and wiggle:
      ConvexPolygon2d checkFootPolygonInWorld = new ConvexPolygon2d(currentFootPolygon);
      checkFootPolygonInWorld.applyTransformAndProjectToXYPlane(snapAndWiggleTransform);

      List<PlanarRegion> planarRegionsIntersectingSnappedAndWiggledPolygon = planarRegionsList.findPlanarRegionsIntersectingPolygon(checkFootPolygonInWorld);

      ArrayList<ConvexPolygon2d> intersectionsInPlaneFrameToPack = new ArrayList<>();
      RigidBodyTransform transformToWorldFromIntersectingPlanarRegion = new RigidBodyTransform();

      if (planarRegionsIntersectingSnappedAndWiggledPolygon != null)
      {
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
                  //               System.out.println("zPenetration = " + zPenetration);

                  if (zPenetration > maximumZPenetrationOnVRegions)
                  {
                     notifyListenerNodeForExpansionWasRejected(bipedalFootstepPlannerNode, BipedalFootstepPlannerNodeRejectionReason.TOO_MUCH_PENETRATION_AFTER_WIGGLE);
                     return null;
                  }
               }
            }
         }
      }

      return snapAndWiggleTransform;
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

   protected void notifyListenerSolutionWasFound(BipedalFootstepPlannerNode goalNode)
   {
      if (listener != null)
      {
         listener.notifyListenerSolutionWasFound();
      }
   }

   protected void notifyListenerSolutionWasNotFound()
   {
      if (listener != null)
      {
         listener.notifyListenerSolutionWasNotFound();
      }
   }

   protected final Vector3d xAxis = new Vector3d();
   protected final Vector3d yAxis = new Vector3d();
   protected final Vector3d zAxis = new Vector3d();

   protected void setTransformZUpPreserveX(RigidBodyTransform transform)
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
