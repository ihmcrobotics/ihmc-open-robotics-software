package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.footstepPlanning.scoring.BipedalStepAdjustmentCostCalculator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionPotentialNextStepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable footArea = new DoubleYoVariable("footArea", registry);
   private final DoubleYoVariable totalArea = new DoubleYoVariable("totalArea", registry);
   private final DoubleYoVariable stepReach = new DoubleYoVariable("stepReach", registry);
   
   private final BooleanYoVariable enableStepAdjustmentCosts;

   private final BipedalFootstepPlannerParameters parameters;

   private PlanarRegionsList planarRegionsList;
   private SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;
   private SideDependentList<ConvexPolygon2d> controllerPolygonsInSoleFrame;

   private FootstepPlannerGoalType footstepPlannerGoalType;
   private Point2D xyGoal;
   private double distanceFromXYGoal;

   private SideDependentList<Point3D> goalPositions;
   private SideDependentList<Double> goalYaws;
   private SideDependentList<RigidBodyTransform> goalFootstepPoses;

   private BipedalFootstepPlannerNode startNode;

   private BipedalFootstepPlannerListener listener;

   private final BipedalStepAdjustmentCostCalculator stepAdjustmentCostCalculator;
   private final IncreasingCostEachStepProvider increasingCostEachStepProvider;
   private final PlanarRegionBaseOfCliffAvoider baseOfCliffAvoider;

   PlanarRegionPotentialNextStepCalculator(BipedalFootstepPlannerParameters parameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.parameters = parameters;
      
      baseOfCliffAvoider = new PlanarRegionBaseOfCliffAvoider(registry, yoGraphicsListRegistry); 

      enableStepAdjustmentCosts = new BooleanYoVariable("enablePenalizationHeatmapScoring", registry);
      enableStepAdjustmentCosts.set(true);

      stepAdjustmentCostCalculator = new BipedalStepAdjustmentCostCalculator(parentRegistry, null);
      increasingCostEachStepProvider = new IncreasingCostEachStepProvider();

      parentRegistry.addChild(registry);
   }
   
   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      setFeetPolygons(footPolygonsInSoleFrame, footPolygonsInSoleFrame);
   }
   
   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame, SideDependentList<ConvexPolygon2d> controllerPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.controllerPolygonsInSoleFrame = controllerPolygonsInSoleFrame;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
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

   public void setStartNode(BipedalFootstepPlannerNode startNode)
   {
      this.startNode = startNode;
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

      goalFootstepPoses = new SideDependentList<>(goalLeftFootPose, goalRightFootPose);

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
      Point3D currentSolePosition = nodeToExpand.getSolePosition();

      RobotSide currentSide = nodeToExpand.getRobotSide();
      RobotSide nextSide = currentSide.getOppositeSide();

      Point3D goalSolePosition = goalPositions.get(nextSide);

      //      stepReach.set(goalSolePosition.distance(currentSolePosition));
      double stepReach = goalSolePosition.distance(currentSolePosition);
      if (stepReach < parameters.getMaximumStepReach())
      {
         double currentSoleYaw = nodeToExpand.getSoleYaw();
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
      Point3D currentPosition = nodeToExpand.getSolePosition();
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

      boolean acceptable = snapNodeAndCheckIfAcceptableToExpand(node);

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

         if (node.getCostToHereFromStart() < smallestCostToGoal)
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

      BipedalFootstepPlannerNode childNode = new BipedalFootstepPlannerNode(nextSide, nextTransform);
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
         Point3D solePosition = childNode.getSolePosition();

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

   protected boolean snapNodeAndCheckIfAcceptableToExpand(BipedalFootstepPlannerNode nodeToExpand)
   {
      nodeToExpand.removePitchAndRoll();

      if (nodeToExpand != startNode) // StartNode is from an actual footstep, so we don't need to snap it...
      {
         // Make sure popped node is a good one and can be expanded...
         boolean snapSucceded = snapToPlanarRegionAndCheckIfGoodSnap(nodeToExpand);
         if (!snapSucceded)
            return false;

         boolean goodFootstep = checkIfGoodFootstep(nodeToExpand);
         if (!goodFootstep)
            return false;

         boolean differentFromParent = checkIfDifferentFromGrandParent(nodeToExpand);
         {
            if (!differentFromParent)
               return false;
         }
      }

      notifyListenerNodeUnderConsiderationWasSuccessful(nodeToExpand);

      return true;
   }

   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TransformReferenceFrame parentSoleFrame = new TransformReferenceFrame("parentSole", ReferenceFrame.getWorldFrame());
   private final ZUpFrame parentSoleZupFrame = new ZUpFrame(worldFrame, parentSoleFrame, "parentSoleZupFrame");
   private final TransformReferenceFrame nodeSoleFrame = new TransformReferenceFrame("nodeSole", ReferenceFrame.getWorldFrame());
   private final FramePoint solePositionInParentZUpFrame = new FramePoint(parentSoleZupFrame);

   private boolean checkIfGoodFootstep(BipedalFootstepPlannerNode nodeToExpand)
   {
      BipedalFootstepPlannerNode parentNode = nodeToExpand.getParentNode();
      if (parentNode == null)
         return true;

      parentNode.getSoleTransform(transform);

      parentSoleFrame.setTransformAndUpdate(transform);
      parentSoleZupFrame.update();

      nodeToExpand.getSoleTransform(transform);
      nodeSoleFrame.setTransformAndUpdate(transform);

      solePositionInParentZUpFrame.setToZero(nodeSoleFrame);
      solePositionInParentZUpFrame.changeFrame(parentSoleZupFrame);

      double minimumStepWidth = parameters.getMinimumStepWidth();

      RobotSide robotSide = nodeToExpand.getRobotSide();
      if (((robotSide == RobotSide.LEFT) && (solePositionInParentZUpFrame.getY() < minimumStepWidth))
            || ((robotSide == RobotSide.RIGHT) && (solePositionInParentZUpFrame.getY() > -minimumStepWidth)))
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH);
         return false;
      }

      double maximumStepWidth = parameters.getMaximumStepWidth();

      if (((robotSide == RobotSide.LEFT) && (solePositionInParentZUpFrame.getY() > maximumStepWidth))
            || ((robotSide == RobotSide.RIGHT) && (solePositionInParentZUpFrame.getY() < -maximumStepWidth)))
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE);
         return false;
      }

      double minimumStepLength = parameters.getMinimumStepLength();
      if (solePositionInParentZUpFrame.getX() < minimumStepLength)
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH);
         return false;
      }

      if (Math.abs(solePositionInParentZUpFrame.getZ()) > parameters.getMaximumStepZ())
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
         return false;
      }

      if ((solePositionInParentZUpFrame.getX() > parameters.getMaximumStepXWhenForwardAndDown())
            && (solePositionInParentZUpFrame.getZ() < -Math.abs(parameters.getMaximumStepZWhenForwardAndDown())))
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN);
         return false;
      }

      stepReach.set(getXYLength(solePositionInParentZUpFrame));
      if (stepReach.getDoubleValue() > parameters.getMaximumStepReach())
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR);
         return false;
      }

      return true;
   }

   private double getXYLength(FramePoint point)
   {
      return Math.sqrt(point.getX() * point.getX() + point.getY() * point.getY());
   }

   private boolean checkIfDifferentFromGrandParent(BipedalFootstepPlannerNode nodeToExpand)
   {
      // OK to step in place if at goal.
      if (nodeToExpand.isAtGoal())
         return true;

      BipedalFootstepPlannerNode parentNode = nodeToExpand.getParentNode();
      if (parentNode == null)
         return true;

      BipedalFootstepPlannerNode grandParentNode = parentNode.getParentNode();
      if (grandParentNode == null)
      {
         //TODO: How to check in place step when first step?
         return true;
      }

      if (grandParentNode.epsilonEquals(nodeToExpand, 1e-1))
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_IN_PLACE);
         return false;
      }

      return true;
   }

   private boolean snapToPlanarRegionAndCheckIfGoodSnap(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (planarRegionsList != null)
      {
         PlanarRegion planarRegion = new PlanarRegion();
         RigidBodyTransform nodeToExpandSnapTransform = getSnapAndWiggleTransform(parameters.getWiggleInsideDelta(), nodeToExpand, planarRegion);

         if (nodeToExpandSnapTransform == null)
         {
            return false;
         }

         nodeToExpand.transformSoleTransformWithSnapTransformFromZeroZ(nodeToExpandSnapTransform, planarRegion);

         baseOfCliffAvoider.shiftAwayFromCliffBottoms(parameters, planarRegionsList, nodeToExpand);

         boolean isEnoughArea = checkIfEnoughArea(nodeToExpand, planarRegion);
         if (!isEnoughArea)
            return false;
      }

      return true;
   }

   private boolean checkIfEnoughArea(BipedalFootstepPlannerNode nodeToExpand, PlanarRegion planarRegion)
   {
      RigidBodyTransform nodeToExpandTransform = new RigidBodyTransform();
      nodeToExpand.getSoleTransform(nodeToExpandTransform);

      ConvexPolygon2d snappedPolygon = controllerPolygonsInSoleFrame.get(nodeToExpand.getRobotSide());
      snappedPolygon.update();
      footArea.set(snappedPolygon.getArea());

      ConvexPolygon2d footholdPolygon = new ConvexPolygon2d();
      totalArea.set(planarRegion.getPolygonIntersectionAreaWhenSnapped(snappedPolygon, nodeToExpandTransform, footholdPolygon));

      nodeToExpand.setPercentageOfFoothold(totalArea.getDoubleValue() / footArea.getDoubleValue());

      if (nodeToExpand.isPartialFoothold())
      {
         nodeToExpand.setPartialFootholdPolygon(footholdPolygon);
      }

      if (totalArea.getDoubleValue() < parameters.getMinimumFootholdPercent() * footArea.getDoubleValue())
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return false;
      }

      return true;
   }

   private RigidBodyTransform getSnapAndWiggleTransform(double wiggleInsideDelta, BipedalFootstepPlannerNode bipedalFootstepPlannerNode,
                                                        PlanarRegion planarRegionToPack)
   {
      if (planarRegionsList == null)
      {
         throw new RuntimeException("Only call this if planarRegionsList exists. Check for null before calling.");
      }

      RobotSide nodeSide = bipedalFootstepPlannerNode.getRobotSide();
      RigidBodyTransform soleTransformBeforeSnap = new RigidBodyTransform();

      bipedalFootstepPlannerNode.getSoleTransform(soleTransformBeforeSnap);
      if (!isTransformZUp(soleTransformBeforeSnap))
      {
         throw new RuntimeException("Node needs to be flat (no pitch or roll) before calling this! bipedalFootstepPlannerNode = \n"
               + bipedalFootstepPlannerNode);
      }

      ConvexPolygon2d currentFootPolygon = new ConvexPolygon2d(footPolygonsInSoleFrame.get(nodeSide));
      currentFootPolygon.applyTransformAndProjectToXYPlane(soleTransformBeforeSnap);

      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(currentFootPolygon, planarRegionsList,
                                                                                                        planarRegionToPack);
      if (snapTransform == null)
      {
         notifyListenerNodeUnderConsiderationWasRejected(bipedalFootstepPlannerNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return null;
      }

      if (Math.abs(snapTransform.getM22()) < parameters.getMinimumSurfaceNormalZ())
      {
         notifyListenerNodeUnderConsiderationWasRejected(bipedalFootstepPlannerNode,
                                                         BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return null;
      }

      BipedalFootstepPlannerNode nodeAfterSnap = new BipedalFootstepPlannerNode(bipedalFootstepPlannerNode);
      nodeAfterSnap.transformSoleTransformWithSnapTransformFromZeroZ(snapTransform, planarRegionToPack);
      //      notifyListenerNodeSnappedAndStillSelectedForExpansion(nodeAfterSnap);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = wiggleInsideDelta;
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

      RigidBodyTransform wiggleTransformLocalToLocal = null;
      if (parameters.getWiggleIntoConvexHullOfPlanarRegions())
         wiggleTransformLocalToLocal = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(polygonToWiggleInRegionFrame, planarRegionToPack, wiggleParameters);
      else
         wiggleTransformLocalToLocal = PolygonWiggler.wigglePolygonIntoRegion(polygonToWiggleInRegionFrame, planarRegionToPack, wiggleParameters);

      if (wiggleTransformLocalToLocal == null)
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeAfterSnap, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_WIGGLE_INSIDE);

         //TODO: Possibly have different node costs depending on how firm on ground they are.
         if (parameters.getRejectIfCannotFullyWiggleInside())
         {
            return null;
         }

         else
         {
            return snapTransform;
         }
      }

      //      System.out.println("wiggleTransformLocalToLocal = \n" + wiggleTransformLocalToLocal);

      //      wiggleTransform = new RigidBodyTransform();
      //      wiggleTransform.setTranslation(0.2, 0.0, 0.0);

      Point3D wiggleTranslation = new Point3D();
      wiggleTransformLocalToLocal.transform(wiggleTranslation);
      Vector3D wiggleVector = new Vector3D(wiggleTranslation);
      if (wiggleVector.length() > parameters.getMaximumXYWiggleDistance())
      {
         wiggleVector.scale(parameters.getMaximumXYWiggleDistance() / wiggleVector.length());
      }

      Vector3D rotationEuler = new Vector3D();
      wiggleTransformLocalToLocal.getRotationEuler(rotationEuler);
      double yaw = rotationEuler.getZ();
      yaw = MathTools.clamp(yaw, parameters.getMaximumYawWiggle());

      rotationEuler.setZ(yaw);
      wiggleTransformLocalToLocal.setRotationEulerAndZeroTranslation(rotationEuler);
      wiggleTransformLocalToLocal.setTranslation(wiggleVector);

      //      System.out.println("Limited wiggleTransformLocalToLocal = \n" + wiggleTransformLocalToLocal);

      RigidBodyTransform wiggleTransformWorldToWorld = new RigidBodyTransform();
      RigidBodyTransform transformOne = new RigidBodyTransform();
      planarRegionToPack.getTransformToWorld(transformOne);
      RigidBodyTransform transformTwo = new RigidBodyTransform(transformOne);
      transformTwo.invert();

      wiggleTransformWorldToWorld.set(transformOne);
      wiggleTransformWorldToWorld.multiply(wiggleTransformLocalToLocal);
      wiggleTransformWorldToWorld.set(wiggleTransformWorldToWorld);
      wiggleTransformWorldToWorld.multiply(transformTwo);

      //      System.out.println("wiggleTransformWorldToWorld = \n" + wiggleTransformWorldToWorld);

      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform(wiggleTransformWorldToWorld);
      snapAndWiggleTransform.multiply(snapTransform);

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
            planarRegionIntersectingSnappedAndWiggledPolygon.getPolygonIntersectionsWhenProjectedVertically(checkFootPolygonInWorld,
                                                                                                            intersectionsInPlaneFrameToPack);

            // If any points are above the plane of the planarRegionToPack, then this is stepping into a v type problem.
            for (ConvexPolygon2d intersectionPolygon : intersectionsInPlaneFrameToPack)
            {
               int numberOfVertices = intersectionPolygon.getNumberOfVertices();
               for (int i = 0; i < numberOfVertices; i++)
               {
                  Point2DReadOnly vertex2d = intersectionPolygon.getVertex(i);
                  Point3D vertex3dInWorld = new Point3D(vertex2d.getX(), vertex2d.getY(), 0.0);
                  transformToWorldFromIntersectingPlanarRegion.transform(vertex3dInWorld);

                  double planeZGivenXY = planarRegionToPack.getPlaneZGivenXY(vertex3dInWorld.getX(), vertex3dInWorld.getY());

                  double zPenetration = vertex3dInWorld.getZ() - planeZGivenXY;
                  //               System.out.println("zPenetration = " + zPenetration);

                  if (zPenetration > parameters.getMaximumZPenetrationOnVRegions())
                  {
                     notifyListenerNodeUnderConsiderationWasRejected(bipedalFootstepPlannerNode,
                                                                     BipedalFootstepPlannerNodeRejectionReason.TOO_MUCH_PENETRATION_AFTER_WIGGLE);
                     return null;
                  }
               }
            }
         }
      }

      return snapAndWiggleTransform;
   }

   private boolean isTransformZUp(RigidBodyTransform soleTransformBeforeSnap)
   {
      return Math.abs(soleTransformBeforeSnap.getM22() - 1.0) < 1e-4;
   }

   private void notifyListenerNodeUnderConsideration(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeUnderConsideration(nodeToExpand);
      }
   }

   private void notifyListenerNodeUnderConsiderationWasSuccessful(BipedalFootstepPlannerNode node)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasSuccessful(node);
      }
   }

   private void notifyListenerNodeUnderConsiderationWasRejected(BipedalFootstepPlannerNode nodeToExpand, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasRejected(nodeToExpand, reason);
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
