package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;
import java.util.Deque;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionPotentialNextStepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final BipedalFootstepPlannerParameters parameters;

   private PlanarRegionsList planarRegionsList;
   private SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame;
   
   private FootstepPlannerGoalType footstepPlannerGoalType;
   private Point2d xyGoal;
   private double distanceFromXYGoal;

   private SideDependentList<Point3d> goalPositions;
   private SideDependentList<Double> goalYaws;
   private SideDependentList<RigidBodyTransform> goalFootstepPoses;

   protected BipedalFootstepPlannerNode startNode, goalNode;

   protected BipedalFootstepPlannerListener listener;

   
   PlanarRegionPotentialNextStepCalculator(YoVariableRegistry parentRegistry, BipedalFootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
     parentRegistry.addChild(registry); 
   }
   
   public void setFeetPolygons(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
         this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
   }
   
   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }
   
   public void setGoal(FootstepPlannerGoal goal)
   {
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

   public BipedalFootstepPlannerNode computeGoalNodeIfGoalIsReachable(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
   {
      BipedalFootstepPlannerNode goalNode = null;

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
   
   private BipedalFootstepPlannerNode findGoalNodeUsingCloseToXY(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
   {
      return null;
   }

   private BipedalFootstepPlannerNode findGoalNodeUsingSolePositions(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
   {
      Point3d currentSolePosition = nodeToExpand.getSolePosition();

      RobotSide currentSide = nodeToExpand.getRobotSide();
      RobotSide nextSide = currentSide.getOppositeSide();

      Point3d goalSolePosition = goalPositions.get(nextSide);

//      stepReach.set(goalSolePosition.distance(currentSolePosition));
      double stepReach = goalSolePosition.distance(currentSolePosition);
      if (stepReach < parameters.getMaximumStepReach())
      {
         double currentSoleYaw = nodeToExpand.getSoleYaw();
         double goalSoleYaw = goalYaws.get(nextSide);

         double stepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(goalSoleYaw, currentSoleYaw);

         if (Math.abs(stepYaw) < parameters.getMaximumStepYaw())
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

   public ArrayList<BipedalFootstepPlannerNode> computeChildrenNodes(RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand)
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


      double idealFootstepLength = parameters.getIdealFootstepLength();
      double idealFootstepWidth = parameters.getIdealFootstepWidth();
      
      if (distance > 2.0 * parameters.getMaximumStepReach())
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

      if (idealStepVector.length() > parameters.getMaximumStepReach())
      {
         idealStepVector.scale(parameters.getMaximumStepReach() / idealStepVector.length());
      }

      double minimumStepWidth = parameters.getMinimumStepWidth();
      
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

      if (distance > 2.0 * parameters.getMaximumStepReach())
      {
         idealYawInWorld = Math.atan2(vectorToGoal.getY(), vectorToGoal.getX());
      }
      else
      {
         idealYawInWorld = goalYaws.get(nextSide);
      }

      double idealStepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(idealYawInWorld, currentYaw);
      idealStepYaw = MathTools.clipToMinMax(idealStepYaw, parameters.getMaximumStepYaw());

      BipedalFootstepPlannerNode childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, idealStepVector, idealStepYaw);
      seeIfNodeIsAtGoal(childNode);
      nodesToAdd.add(childNode);
      
      for (double xStep = idealFootstepLength/2.0; xStep < 1.6 * idealFootstepLength; xStep = xStep + idealFootstepLength / 4.0)
      {
         for (double yStep = idealFootstepWidth; yStep < 1.6 * idealFootstepWidth; yStep = yStep + idealFootstepWidth / 4.0)
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

      // Add a side step.
      double xStep = 0.0;
      double yStep = parameters.getIdealFootstepWidth();
      Vector3d nextStepVector = new Vector3d(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
      double nextStepYaw = idealStepYaw;
      childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector, nextStepYaw);

      seeIfNodeIsAtGoal(childNode);
      nodesToAdd.add(childNode);
      return nodesToAdd;
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
   
 
//   public void computePotentialNextSteps(BipedalFootstepPlannerParameters parameters, FootstepPlannerGoal footstepPlannerGoal, 
//                                         SideDependentList<Point3d> goalPositions, SideDependentList<Double> goalYaws, BipedalFootstepPlannerNode nodeToExpand, 
//                                         ArrayList<BipedalFootstepPlannerNode> potentialNextStepsToPack)
//   {
//      RigidBodyTransform soleZUpTransform = new RigidBodyTransform();
//      nodeToExpand.getSoleTransform(soleZUpTransform);
//      setTransformZUpPreserveX(soleZUpTransform);
//      
//      RobotSide currentSide = nodeToExpand.getRobotSide();
//      RobotSide nextSide = currentSide.getOppositeSide();
//
//      Point3d goalPosition = goalPositions.get(nextSide);
//      Point3d currentPosition = nodeToExpand.getSolePosition();
//      Vector3d currentToGoalVector = new Vector3d();
//      currentToGoalVector.sub(goalPosition, currentPosition);
//
//      double distance = currentToGoalVector.length();
//
//      Vector3d idealStepVector;
//
//
//      double idealFootstepLength = parameters.getIdealFootstepLength();
//      double idealFootstepWidth = parameters.getIdealFootstepWidth();
//      
//      if (distance > 2.0 * parameters.getMaximumStepReach())
//      {
//         idealStepVector = new Vector3d(idealFootstepLength, currentSide.negateIfLeftSide(idealFootstepWidth), 0.0);
//      }
//      else
//      {
//         idealStepVector = new Vector3d(currentToGoalVector);
//         RigidBodyTransform inverseTransform = new RigidBodyTransform(soleZUpTransform);
//         inverseTransform.invert();
//         inverseTransform.transform(idealStepVector);
//      }
//
//      if (idealStepVector.length() > parameters.getMaximumStepReach())
//      {
//         idealStepVector.scale(parameters.getMaximumStepReach() / idealStepVector.length());
//      }
//
//      double minimumStepWidth = parameters.getMinimumStepWidth();
//      
//      if ((nextSide == RobotSide.LEFT) && (idealStepVector.getY() < minimumStepWidth))
//      {
//         idealStepVector.setY(minimumStepWidth);
//      }
//      else if ((nextSide == RobotSide.RIGHT) && (idealStepVector.getY() > -minimumStepWidth))
//      {
//         idealStepVector.setY(-minimumStepWidth);
//      }
//
//      Point3d idealStepLocationInWorld = new Point3d(idealStepVector);
//      soleZUpTransform.transform(idealStepLocationInWorld);
//
//      Vector3d vectorToGoal = new Vector3d();
//      vectorToGoal.sub(goalPosition, idealStepLocationInWorld);
//
//      Vector3d currentRotationEulerInWorld = new Vector3d();
//      soleZUpTransform.getRotationEuler(currentRotationEulerInWorld);
//      double currentYaw = currentRotationEulerInWorld.getZ();
//
//      double idealYawInWorld;
//
//      if (distance > 2.0 * parameters.getMaximumStepReach())
//      {
//         idealYawInWorld = Math.atan2(vectorToGoal.getY(), vectorToGoal.getX());
//      }
//      else
//      {
//         idealYawInWorld = goalYaws.get(nextSide);
//      }
//
//      double idealStepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(idealYawInWorld, currentYaw);
//      idealStepYaw = MathTools.clipToMinMax(idealStepYaw, parameters.getMaximumStepYaw());
//
//      BipedalFootstepPlannerNode childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, idealStepVector, idealStepYaw);
//      seeIfNodeIsAtGoal(footstepPlannerGoal, childNode);
//      potentialNextStepsToPack.add(childNode);
//      
//      for (double xStep = idealFootstepLength/2.0; xStep < 1.6 * idealFootstepLength; xStep = xStep + idealFootstepLength / 4.0)
//      {
//         for (double yStep = idealFootstepWidth; yStep < 1.6 * idealFootstepWidth; yStep = yStep + idealFootstepWidth / 4.0)
//         {
//            //for (double thetaStep = -0.1; thetaStep < 0.1; thetaStep = thetaStep + 0.1 / 2.0)
//            {
//               double nextStepYaw = idealStepYaw;
//               Vector3d nextStepVector = new Vector3d(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
//               childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector, nextStepYaw);
//
//               seeIfNodeIsAtGoal(footstepPlannerGoal, childNode);
//               potentialNextStepsToPack.add(childNode);
//            }
//         }
//      }
//
//      // Add a side step.
//      double xStep = 0.0;
//      double yStep = parameters.getIdealFootstepWidth();
//      Vector3d nextStepVector = new Vector3d(xStep, currentSide.negateIfLeftSide(yStep), 0.0);
//      double nextStepYaw = idealStepYaw;
//      childNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, nextStepVector, nextStepYaw);
//
//      seeIfNodeIsAtGoal(footstepPlannerGoal, childNode);
//      potentialNextStepsToPack.add(childNode);
//      
////      potentialNextStepsToPack.add(null);
//   }
//   
//   
//   protected BipedalFootstepPlannerNode addGoalNodeIfGoalIsReachable(BipedalFootstepPlannerParameters parameters, FootstepPlannerGoal footstepPlannerGoal, 
//                                                  BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
//   {
//      BipedalFootstepPlannerNode goalNode = null;
//
//      FootstepPlannerGoalType footstepPlannerGoalType = footstepPlannerGoal.getFootstepPlannerGoalType();
//      
//      if (footstepPlannerGoalType == FootstepPlannerGoalType.CLOSE_TO_XY_POSITION)
//      {
//         goalNode = findGoalNodeUsingCloseToXY(nodeToExpand, soleZUpTransform);
//      }
//      else
//      {
//         goalNode = findGoalNodeUsingSolePositions(parameters, footstepPlannerGoal, nodeToExpand, soleZUpTransform);
//      }
//
//      return goalNode;
//      
////      if (goalNode != null)
////      {
////         stack.push(goalNode);
////         return true;
////      }
////
////      return false;
//   }
//
//   private BipedalFootstepPlannerNode findGoalNodeUsingCloseToXY(BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
//   {
//      return null;
//   }
//
//   private BipedalFootstepPlannerNode findGoalNodeUsingSolePositions(BipedalFootstepPlannerParameters parameters, FootstepPlannerGoal footstepPlannerGoal, BipedalFootstepPlannerNode nodeToExpand, RigidBodyTransform soleZUpTransform)
//   {
//      Point3d currentSolePosition = nodeToExpand.getSolePosition();
//
//      RobotSide currentSide = nodeToExpand.getRobotSide();
//      RobotSide nextSide = currentSide.getOppositeSide();
//
//            
//      Point3d goalSolePosition = goalPositions.get(nextSide);
//
////      stepReach.set(goalSolePosition.distance(currentSolePosition));
//      double stepReach = goalSolePosition.distance(currentSolePosition);
//      if (stepReach < parameters.getMaximumStepReach())
//      {
//         double currentSoleYaw = nodeToExpand.getSoleYaw();
//         double goalSoleYaw = goalYaws.get(nextSide);
//
//         double stepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(goalSoleYaw, currentSoleYaw);
//
//         if (Math.abs(stepYaw) < parameters.getMaximumStepYaw())
//         {
//            Vector3d finishStep = new Vector3d();
//            finishStep.sub(goalSolePosition, currentSolePosition);
//
//            RigidBodyTransform inverseTransform = new RigidBodyTransform();
//            nodeToExpand.getSoleTransform(inverseTransform);
//            inverseTransform.invert();
//            inverseTransform.transform(finishStep);
//
//            BipedalFootstepPlannerNode goalNode = createAndAddNextNodeGivenStep(soleZUpTransform, nodeToExpand, finishStep, stepYaw);
//            goalNode.setIsAtGoal();
//
//            return goalNode;
//         }
//      }
//
//      return null;
//   }
//   
//
//   
//   public void setGoalPositionsAndYaws(BipedalFootstepPlannerParameters parameters, FootstepPlannerGoal goal)
//   {
//      FramePose goalPose = goal.getGoalPoseBetweenFeet();
//      goalPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
//
//      RigidBodyTransform goalLeftFootPose = new RigidBodyTransform();
//      RigidBodyTransform goalRightFootPose = new RigidBodyTransform();
//
//      goalPose.getPose(goalLeftFootPose);
//      goalPose.getPose(goalRightFootPose);
//
//      Vector3d toTheLeft;
//      Vector3d toTheRight;
//
//      double idealFootstepWidth = parameters.getIdealFootstepWidth();
//      
//      if(idealFootstepWidth == 0.0)
//      {
//         toTheLeft = new Vector3d(0.0, 0.15, 0.0);
//         toTheRight = new Vector3d(0.0, -0.15, 0.0);
//      }
//      else
//      {
//         toTheLeft = new Vector3d(0.0, idealFootstepWidth/2.0, 0.0);
//         toTheRight = new Vector3d(0.0, - idealFootstepWidth/2.0, 0.0);
//      }
//
//      goalLeftFootPose.applyTranslation(toTheLeft);
//      goalRightFootPose.applyTranslation(toTheRight);
//
//      goalFootstepPoses = new SideDependentList<>(goalLeftFootPose, goalRightFootPose);
//
//      Point3d goalLeftSolePosition = new Point3d();
//      goalLeftFootPose.transform(goalLeftSolePosition);
//
//      Point3d goalRightSolePosition = new Point3d();
//      goalRightFootPose.transform(goalRightSolePosition);
//
//      Vector3d eulerAngles = new Vector3d();
//      goalLeftFootPose.getRotationEuler(eulerAngles);
//      double goalLeftSoleYaw = eulerAngles.getZ();
//      goalRightFootPose.getRotationEuler(eulerAngles);
//      double goalRightSoleYaw = eulerAngles.getZ();
//
//      goalPositions = new SideDependentList<>(goalLeftSolePosition, goalRightSolePosition);
//      goalYaws = new SideDependentList<>(goalLeftSoleYaw, goalRightSoleYaw);
//
////      if (listener != null)
////      {
////         listener.goalWasSet(goalLeftFootPose, goalRightFootPose);
////      }
//   }
//   
//   private void seeIfNodeIsAtGoal(FootstepPlannerGoal footstepPlannerGoal, BipedalFootstepPlannerNode childNode)
//   {
//      FootstepPlannerGoalType footstepPlannerGoalType = footstepPlannerGoal.getFootstepPlannerGoalType();
//      Point2d xyGoal = footstepPlannerGoal.getXYGoal();
//      double distanceFromXYGoal = footstepPlannerGoal.getDistanceFromXYGoal();
//      
//      if (footstepPlannerGoalType == FootstepPlannerGoalType.CLOSE_TO_XY_POSITION)
//      {
//         Point3d solePosition = childNode.getSolePosition();
//
//         double deltaX = solePosition.getX() - xyGoal.getX();
//         double deltaY = solePosition.getY() - xyGoal.getY();
//         double distanceSquared = deltaX * deltaX + deltaY * deltaY;
//         double distanceFromXYGoalSquared = distanceFromXYGoal * distanceFromXYGoal;
//
////                  System.out.println("distanceSquared = " + distanceSquared);
////                  System.out.println("distanceFromXYGoalSquared = " + distanceFromXYGoalSquared);
//         if (distanceSquared < distanceFromXYGoalSquared)
//         {
////                     System.out.println("Setting at goal for child node!");
//            childNode.setIsAtGoal();
//         }
//      }
//   }
//   
//   protected final Vector3d xAxis = new Vector3d();
//   protected final Vector3d yAxis = new Vector3d();
//   protected final Vector3d zAxis = new Vector3d();
//   
//   protected void setTransformZUpPreserveX(RigidBodyTransform transform)
//   {
//      xAxis.set(transform.getM00(), transform.getM10(), 0.0);
//      xAxis.normalize();
//      zAxis.set(0.0, 0.0, 1.0);
//      yAxis.cross(zAxis, xAxis);
//
//      transform.setM00(xAxis.getX());
//      transform.setM10(xAxis.getY());
//      transform.setM20(xAxis.getZ());
//
//      transform.setM01(yAxis.getX());
//      transform.setM11(yAxis.getY());
//      transform.setM21(yAxis.getZ());
//
//      transform.setM02(zAxis.getX());
//      transform.setM12(zAxis.getY());
//      transform.setM22(zAxis.getZ());
//   }
//   
//   protected BipedalFootstepPlannerNode createAndAddNextNodeGivenStep(RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand, Vector3d stepVectorInSoleFrame, double stepYaw)
//   {
//      Point3d stepLocationInWorld = new Point3d(stepVectorInSoleFrame);
//      soleZUpTransform.transform(stepLocationInWorld);
//
//      Vector3d stepRotationEulerInWorld = new Vector3d();
//      soleZUpTransform.getRotationEuler(stepRotationEulerInWorld);
//      //      stepRotationEulerInWorld.setZ((stepRotationEulerInWorld.getZ() + stepYaw + 2.0 * Math.PI) % Math.PI);
//      stepRotationEulerInWorld.setZ(stepRotationEulerInWorld.getZ() + stepYaw);
//
//      RigidBodyTransform nextTransform = new RigidBodyTransform();
//      nextTransform.setRotationEulerAndZeroTranslation(stepRotationEulerInWorld);
//      nextTransform.setTranslation(stepLocationInWorld.getX(), stepLocationInWorld.getY(), stepLocationInWorld.getZ());
//
//      RobotSide nextSide = nodeToExpand.getRobotSide().getOppositeSide();
//
//      BipedalFootstepPlannerNode childNode = new BipedalFootstepPlannerNode(nextSide, nextTransform);
//      childNode.setParentNode(nodeToExpand);
//      nodeToExpand.addChild(childNode);
//      return childNode;
//   }


}
