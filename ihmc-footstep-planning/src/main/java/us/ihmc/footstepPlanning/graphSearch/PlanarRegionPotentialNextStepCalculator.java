package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.scoring.BipedalStepAdjustmentCostCalculator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class PlanarRegionPotentialNextStepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean enableStepAdjustmentCosts;

   private final BipedalFootstepPlannerParameters parameters;
   private FootstepPlannerGoalType footstepPlannerGoalType;
   private Point2D xyGoal;
   private double distanceFromXYGoal;

   private SideDependentList<Point3D> goalPositions;
   private SideDependentList<Double> goalYaws;
   private BipedalFootstepPlannerListener listener;

   private final BipedalStepAdjustmentCostCalculator stepAdjustmentCostCalculator;
   private final IncreasingCostEachStepProvider increasingCostEachStepProvider;

   private final BipedalFootstepPlannerNodeChecker nodeChecker;

   PlanarRegionPotentialNextStepCalculator(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.parameters = parameters;
      this.nodeChecker = new BipedalFootstepPlannerNodeChecker(parameters, yoGraphicsListRegistry);

      enableStepAdjustmentCosts = new YoBoolean("enablePenalizationHeatmapScoring", registry);
      enableStepAdjustmentCosts.set(true);

      stepAdjustmentCostCalculator = new BipedalStepAdjustmentCostCalculator(parentRegistry, null);
      increasingCostEachStepProvider = new IncreasingCostEachStepProvider();

      parentRegistry.addChild(registry);
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, SideDependentList<ConvexPolygon2D> controllerPolygonsInSoleFrame)
   {
      nodeChecker.setFeetPolygons(footPolygonsInSoleFrame, controllerPolygonsInSoleFrame);
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      nodeChecker.setPlanarRegions(planarRegionsList);

      if (listener != null)
      {
         listener.planarRegionsListSet(planarRegionsList);
      }
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
      nodeChecker.setBipedalFootstepPlannerListener(listener);
   }

   public void setStartNode(BipedalFootstepPlannerNode startNode)
   {
      nodeChecker.setStartNode(startNode);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      footstepPlannerGoalType = goal.getFootstepPlannerGoalType();

      switch (footstepPlannerGoalType)
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
      xyGoal = new Point2D(goal.getXYGoal());
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

      Vector3D toTheLeft;
      Vector3D toTheRight;

      double idealFootstepWidth = parameters.getIdealFootstepWidth();

      if (idealFootstepWidth == 0.0)
      {
         toTheLeft = new Vector3D(0.0, 0.15, 0.0);
         toTheRight = new Vector3D(0.0, -0.15, 0.0);
      }
      else
      {
         toTheLeft = new Vector3D(0.0, idealFootstepWidth / 2.0, 0.0);
         toTheRight = new Vector3D(0.0, -idealFootstepWidth / 2.0, 0.0);
      }

      RigidBodyTransform tempTransform = new RigidBodyTransform();
      tempTransform.setTranslation(toTheLeft);
      goalLeftFootPose.transform(tempTransform);
      goalLeftFootPose.set(tempTransform);

      tempTransform.setIdentity();
      tempTransform.setTranslation(toTheRight);
      goalRightFootPose.transform(tempTransform);
      goalRightFootPose.set(tempTransform);

      Point3D goalLeftSolePosition = new Point3D();
      goalLeftFootPose.transform(goalLeftSolePosition);

      Point3D goalRightSolePosition = new Point3D();
      goalRightFootPose.transform(goalRightSolePosition);

      Vector3D eulerAngles = new Vector3D();
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

   public BipedalFootstepPlannerNode computeGoalNodeIfGoalIsReachable(BipedalFootstepPlannerNode nodeToExpand)
   {
      BipedalFootstepPlannerNode goalNode = null;

      RigidBodyTransform soleZUpTransform = computeSoleZUpTransform(nodeToExpand);

      if (footstepPlannerGoalType == FootstepPlannerGoalType.CLOSE_TO_XY_POSITION)
      {
         goalNode = findGoalNodeUsingCloseToXY(nodeToExpand, soleZUpTransform);
      }
      else
      {
         goalNode = findGoalNodeUsingSolePositions(nodeToExpand, soleZUpTransform);
      }

      return goalNode;
   }

   private RigidBodyTransform computeSoleZUpTransform(BipedalFootstepPlannerNode nodeToExpand)
   {
      RigidBodyTransform soleZUpTransform = new RigidBodyTransform();
      nodeToExpand.getSoleTransform(soleZUpTransform);
      setTransformZUpPreserveX(soleZUpTransform);
      return soleZUpTransform;
   }

   private BipedalFootstepPlannerNode findGoalNodeUsingCloseToXY(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
   {
      return null;
   }

   private BipedalFootstepPlannerNode findGoalNodeUsingSolePositions(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
   {
      Point3D currentSolePosition = BipedalFootstepPlannerNodeUtils.getSolePosition(nodeToExpand);

      RobotSide currentSide = nodeToExpand.getRobotSide();
      RobotSide nextSide = currentSide.getOppositeSide();

      Point3D goalSolePosition = goalPositions.get(nextSide);

      //      stepReach.set(goalSolePosition.distance(currentSolePosition));
      double stepReach = goalSolePosition.distance(currentSolePosition);
      if (stepReach < parameters.getMaximumStepReach())
      {
         double currentSoleYaw = BipedalFootstepPlannerNodeUtils.getSoleYaw(nodeToExpand);
         double goalSoleYaw = goalYaws.get(nextSide);

         double stepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(goalSoleYaw, currentSoleYaw);

         if (Math.abs(stepYaw) < parameters.getMaximumStepYaw())
         {
            Vector3D finishStep = new Vector3D();
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

   public ArrayList<BipedalFootstepPlannerNode> computeChildrenNodes(BipedalFootstepPlannerNode nodeToExpand, double smallestCostToGoal)
   {
      ArrayList<BipedalFootstepPlannerNode> nodesToAdd = new ArrayList<>();

      BipedalFootstepPlannerNode goalNode = computeGoalNodeIfGoalIsReachable(nodeToExpand);
      if (goalNode != null)
      {
         boolean acceptable = checkIfNodeAcceptableCostAndAddToList(goalNode, nodesToAdd, new Vector3D(), 0.0, smallestCostToGoal);
         if (acceptable)
            return nodesToAdd;
      }

      RigidBodyTransform soleZUpTransform = computeSoleZUpTransform(nodeToExpand);

      RobotSide currentSide = nodeToExpand.getRobotSide();
      RobotSide nextSide = currentSide.getOppositeSide();

      Point3D goalPosition = goalPositions.get(nextSide);
      Point3D currentPosition = BipedalFootstepPlannerNodeUtils.getSolePosition(nodeToExpand);
      Vector3D currentToGoalVector = new Vector3D();
      currentToGoalVector.sub(goalPosition, currentPosition);

      double distance = currentToGoalVector.length();
      Vector3D idealStepVector = computeIdealStepVector(parameters, soleZUpTransform, nextSide, currentToGoalVector);

      Point3D idealStepLocationInWorld = new Point3D(idealStepVector);
      soleZUpTransform.transform(idealStepLocationInWorld);

      Vector3D vectorToGoal = new Vector3D();
      vectorToGoal.sub(goalPosition, idealStepLocationInWorld);

      Vector3D currentRotationEulerInWorld = new Vector3D();
      soleZUpTransform.getRotationEuler(currentRotationEulerInWorld);
      double currentYaw = currentRotationEulerInWorld.getZ();

      double idealYawInWorld;

      if (distance > 2.0 * parameters.getMaximumStepReach())
      {
         idealYawInWorld = Math.atan2(vectorToGoal.getY(), vectorToGoal.getX());
      }
      else
      {
         idealYawInWorld = goalYaws.get(nextSide);
      }

      double idealStepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(idealYawInWorld, currentYaw);
      idealStepYaw = MathTools.clamp(idealStepYaw, parameters.getMaximumStepYaw());

      BipedalFootstepPlannerNode childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, idealStepVector, idealStepYaw);
      seeIfNodeIsAtGoal(childNode);

      checkIfNodeAcceptableCostAndAddToList(childNode, nodesToAdd, idealStepVector, idealStepYaw, smallestCostToGoal);

      for (double xStep = idealStepVector.getX() / 2.0; xStep < 1.6 * idealStepVector.getX(); xStep = xStep + idealStepVector.getX() / 4.0)
      {
         for (double yStep = parameters.getMinimumStepWidth(); yStep < parameters.getMaximumStepWidth(); yStep = yStep + parameters.getMaximumStepWidth() / 4.0)
         {
            //for (double thetaStep = -0.1; thetaStep < 0.1; thetaStep = thetaStep + 0.1 / 2.0)
            {
               double nextStepYaw = idealStepYaw;
               Vector3D nextStepVector = new Vector3D(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
               childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector, nextStepYaw);

               seeIfNodeIsAtGoal(childNode);

               checkIfNodeAcceptableCostAndAddToList(childNode, nodesToAdd, idealStepVector, idealStepYaw, smallestCostToGoal);
            }
         }
      }

      // Add a side step.
      double xStep = 1.001 * Math.max(0.0, parameters.getMinimumStepLength());
      double yStep = parameters.getIdealFootstepWidth();
      Vector3D nextStepVector = new Vector3D(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
      double nextStepYaw = idealStepYaw;
      childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector, nextStepYaw);

      seeIfNodeIsAtGoal(childNode);
      checkIfNodeAcceptableCostAndAddToList(childNode, nodesToAdd, idealStepVector, idealStepYaw, smallestCostToGoal);

      NodeCostComparator nodeCostComparator = new NodeCostComparator();

      Collections.sort(nodesToAdd, nodeCostComparator);
      return nodesToAdd;
   }

   private static Vector3D computeIdealStepVector(BipedalFootstepPlannerParameters parameters, RigidBodyTransform soleZUpTransform, RobotSide nextSide,
                                                  Vector3D currentToGoalInWorld)
   {
      double distanceToGoal = currentToGoalInWorld.length();

      Vector3D currentToGoalInSoleFrame = new Vector3D(currentToGoalInWorld);
      RigidBodyTransform inverseTransform = new RigidBodyTransform(soleZUpTransform);
      inverseTransform.invert();
      inverseTransform.transform(currentToGoalInSoleFrame);

      Vector3D idealStepVector;

      if (distanceToGoal > 2.0 * parameters.getMaximumStepReach())
      {
         idealStepVector = new Vector3D(parameters.getIdealFootstepLength(), nextSide.negateIfRightSide(parameters.getIdealFootstepWidth()), 0.0);

         double idealYawInSoleFrame = Math.atan2(currentToGoalInSoleFrame.getY(), currentToGoalInSoleFrame.getX());

         double numberOfStepsToYawToGoal = Math.abs(idealYawInSoleFrame) / parameters.getMaximumStepYaw();
         double distancePerStepToTurn = distanceToGoal / numberOfStepsToYawToGoal * 0.25;

         if (idealStepVector.getX() > distancePerStepToTurn)
         {
            idealStepVector.setX(distancePerStepToTurn);
         }
      }
      else
      {
         idealStepVector = currentToGoalInSoleFrame;
      }

      if (idealStepVector.length() > 0.9 * parameters.getMaximumStepReach())
      {
         idealStepVector.scale(0.9 * parameters.getMaximumStepReach() / idealStepVector.length());
      }

      double minimumStepWidth = parameters.getMinimumStepWidth();

      if ((nextSide == RobotSide.LEFT) && (idealStepVector.getY() < 1.01 * minimumStepWidth))
      {
         idealStepVector.setY(1.01 * minimumStepWidth);
      }
      else if ((nextSide == RobotSide.RIGHT) && (idealStepVector.getY() > -1.01 * minimumStepWidth))
      {
         idealStepVector.setY(-1.01 * minimumStepWidth);
      }
      return idealStepVector;
   }

   private final RigidBodyTransform leftSoleTransform = new RigidBodyTransform();
   private final RigidBodyTransform rightSoleTransform = new RigidBodyTransform();
   private final SideDependentList<RigidBodyTransform> soleTransforms = new SideDependentList<>(leftSoleTransform, rightSoleTransform);

   private boolean checkIfNodeAcceptableCostAndAddToList(BipedalFootstepPlannerNode node, ArrayList<BipedalFootstepPlannerNode> nodesToAdd,
                                                          Vector3D idealStepVector, double idealStepYaw, double smallestCostToGoal)
   {
      notifyListenerNodeUnderConsideration(node);

      boolean acceptable = nodeChecker.snapNodeAndCheckIfAcceptableToExpand(node);

      if (acceptable)
      {
         BipedalFootstepPlannerNode parentNode = node.getParentNode();

         if (parentNode == null)
         {
            node.setSingleStepCost(0.0);
         }
         else
         {
            node.getSoleTransform(soleTransforms.get(node.getRobotSide()));
            parentNode.getSoleTransform(soleTransforms.get(parentNode.getRobotSide()));

            RigidBodyTransform nodeTransform = new RigidBodyTransform();
            node.getSoleTransform(nodeTransform);
            FramePose candidateFootPose = new FramePose(ReferenceFrame.getWorldFrame(), nodeTransform);

            RigidBodyTransform stanceFootTransform = soleTransforms.get(node.getRobotSide().getOppositeSide());
            RigidBodyTransform swingStartFootTransform = soleTransforms.get(node.getRobotSide());

            FramePose stanceFootPose = new FramePose(ReferenceFrame.getWorldFrame(), stanceFootTransform);
            FramePose swingStartFootPose = new FramePose(ReferenceFrame.getWorldFrame(), swingStartFootTransform);

            RigidBodyTransform idealStepTransform = getTransformFromStepToWorld(stanceFootTransform, idealStepVector, idealStepYaw);
            FramePose idealFootstepPose = new FramePose(ReferenceFrame.getWorldFrame(), idealStepTransform);

            double cost;
            if (enableStepAdjustmentCosts.getBooleanValue())
            {
               cost = stepAdjustmentCostCalculator.calculateCost(stanceFootPose, swingStartFootPose, idealFootstepPose, candidateFootPose, node.getPercentageOfFoothold());
            }
            else
            {
               cost = increasingCostEachStepProvider.calculateCost(stanceFootPose, swingStartFootPose, idealFootstepPose, candidateFootPose, node.getPercentageOfFoothold());
            }

            node.setSingleStepCost(cost);
         }

         if (BipedalFootstepPlannerNodeUtils.getCostFromStartToNode(node) < smallestCostToGoal)
            nodesToAdd.add(node);
         else
            return false;
      }
      else
      {
         node.setSingleStepCost(Double.POSITIVE_INFINITY);
      }

      return acceptable;
   }

   private BipedalFootstepPlannerNode createAndAddNextNodeGivenStep(RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand,
                                                                    Vector3D stepVectorInSoleFrame, double stepYaw)
   {
      RigidBodyTransform nextTransform = getTransformFromStepToWorld(soleZUpTransform, stepVectorInSoleFrame, stepYaw);

      RobotSide nextSide = nodeToExpand.getRobotSide().getOppositeSide();

      Vector3D nextStepTranslation = new Vector3D();
      nextTransform.getTranslation(nextStepTranslation);

      double nextStepYaw = nodeToExpand.getYaw() + stepYaw;

      BipedalFootstepPlannerNode childNode = new BipedalFootstepPlannerNode(nextStepTranslation.getX(), nextStepTranslation.getY(), nextStepYaw, nextSide);
      childNode.setParentNode(nodeToExpand);
      nodeToExpand.addChild(childNode);
      return childNode;
   }

   private RigidBodyTransform getTransformFromStepToWorld(RigidBodyTransform soleZUpTransform, Vector3D stepVectorInSoleFrame, double stepYaw)
   {
      Point3D stepLocationInWorld = new Point3D(stepVectorInSoleFrame);
      soleZUpTransform.transform(stepLocationInWorld);

      Vector3D stepRotationEulerInWorld = new Vector3D();
      soleZUpTransform.getRotationEuler(stepRotationEulerInWorld);
      //      stepRotationEulerInWorld.setZ((stepRotationEulerInWorld.getZ() + stepYaw + 2.0 * Math.PI) % Math.PI);
      stepRotationEulerInWorld.setZ(stepRotationEulerInWorld.getZ() + stepYaw);

      RigidBodyTransform nextTransform = new RigidBodyTransform();
      nextTransform.setRotationEulerAndZeroTranslation(stepRotationEulerInWorld);
      nextTransform.setTranslation(stepLocationInWorld.getX(), stepLocationInWorld.getY(), stepLocationInWorld.getZ());
      return nextTransform;
   }

   private void seeIfNodeIsAtGoal(BipedalFootstepPlannerNode childNode)
   {
      if (footstepPlannerGoalType == FootstepPlannerGoalType.CLOSE_TO_XY_POSITION)
      {
         Point3D solePosition = BipedalFootstepPlannerNodeUtils.getSolePosition(childNode);

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

   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();
   private final Vector3D zAxis = new Vector3D();

   private void setTransformZUpPreserveX(RigidBodyTransform transform)
   {
      xAxis.set(transform.getM00(), transform.getM10(), 0.0);
      xAxis.normalize();
      zAxis.set(0.0, 0.0, 1.0);
      yAxis.cross(zAxis, xAxis);

      transform.setRotation(xAxis.getX(), yAxis.getX(), zAxis.getX(), xAxis.getY(), yAxis.getY(), zAxis.getY(), xAxis.getZ(), yAxis.getZ(), zAxis.getZ());
   }

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TransformReferenceFrame parentSoleFrame = new TransformReferenceFrame("parentSole", ReferenceFrame.getWorldFrame());
   private final ZUpFrame parentSoleZupFrame = new ZUpFrame(worldFrame, parentSoleFrame, "parentSoleZupFrame");

   private void notifyListenerNodeUnderConsideration(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeUnderConsideration(nodeToExpand);
      }
   }

   public Point3D getGoalPosition(RobotSide robotSide)
   {
      return goalPositions.get(robotSide);
   }

   private class NodeCostComparator implements Comparator<BipedalFootstepPlannerNode>
   {
      @Override
      public int compare(BipedalFootstepPlannerNode nodeOne, BipedalFootstepPlannerNode nodeTwo)
      {
         if (nodeOne.getSingleStepCost() > nodeTwo.getSingleStepCost())
         {
            return -1;
         }
         else
         {
            return 1;
         }
      }
   }
}
