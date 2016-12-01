package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionPotentialNextStepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   protected final DoubleYoVariable footArea = new DoubleYoVariable("footArea", registry);
   protected final DoubleYoVariable totalArea = new DoubleYoVariable("totalArea", registry);
   protected final DoubleYoVariable stepReach = new DoubleYoVariable("stepReach", registry);

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

   public ArrayList<BipedalFootstepPlannerNode> computeChildrenNodes(BipedalFootstepPlannerNode nodeToExpand)
   {
      ArrayList<BipedalFootstepPlannerNode> nodesToAdd = new ArrayList<>();

      RigidBodyTransform soleZUpTransform = computeSoleZUpTransform(nodeToExpand);

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

   public boolean checkNodeAcceptableToExpand(BipedalFootstepPlannerNode nodeToExpand)
   {

      if (nodeToExpand != startNode) // StartNode is from an actual footstep, so we don't need to snap it...
      {
         // Make sure popped node is a good one and can be expanded...
         boolean snapSucceded = snapToPlanarRegionAndCheckIfGoodSnap(nodeToExpand);
         if (!snapSucceded)
            return false;;

            boolean goodFootstep = checkIfGoodFootstep(nodeToExpand);
            if (!goodFootstep)
               return false;

            boolean differentFromParent = checkIfDifferentFromGrandParent(nodeToExpand);
            {
               if (!differentFromParent)
                  return false;
            }

      }

      return true;

   }
   
   
   protected boolean checkIfGoodFootstep(BipedalFootstepPlannerNode nodeToExpand)
   {
      RobotSide robotSide = nodeToExpand.getRobotSide();

      RigidBodyTransform transformToParent = nodeToExpand.getTransformToParent();
      if (transformToParent != null)
      {
         Point3d stepFromParentInSoleFrame = new Point3d();
         transformToParent.transform(stepFromParentInSoleFrame);

         double minimumStepWidth = parameters.getMinimumStepWidth();
         
         if (((robotSide == RobotSide.LEFT) && (stepFromParentInSoleFrame.getY() < minimumStepWidth)) || ((robotSide == RobotSide.RIGHT) && (stepFromParentInSoleFrame.getY() > -minimumStepWidth)))
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH);
            return false;
         }

         Vector3d stepFromParentInWorld = new Vector3d(stepFromParentInSoleFrame);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();

         nodeToExpand.getParentNode().getSoleTransform(transformToWorld);
         transformToWorld.transform(stepFromParentInWorld);

         if (Math.abs(stepFromParentInWorld.getZ()) > parameters.getMaximumStepZ())
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
            return false;
         }
         
         if ((stepFromParentInSoleFrame.getX() > parameters.getMaximumStepXWhenForwardAndDown()) && (stepFromParentInWorld.getZ() < -Math.abs(parameters.getMaximumStepZWhenForwardAndDown())))
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN);
            return false;
         }

         stepReach.set(stepFromParentInWorld.length());
         if (stepReach.getDoubleValue() > parameters.getMaximumStepReach())
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR);
            return false;
         }
      }

      return true;
   }
   
   
   protected boolean checkIfDifferentFromGrandParent(BipedalFootstepPlannerNode nodeToExpand)
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
   
   protected boolean snapToPlanarRegionAndCheckIfGoodSnap(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (planarRegionsList != null)
      {
         PlanarRegion planarRegion = new PlanarRegion();
         RigidBodyTransform nodeToExpandSnapTransform = getSnapAndWiggleTransform(parameters.getWiggleInsideDelta(), nodeToExpand, planarRegion);

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

         if (totalArea.getDoubleValue() < parameters.getMinimumFootholdPercent() * footArea.getDoubleValue())
         {
            notifyListenerNodeForExpansionWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
            return false;
         }
      }

      return true;
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

      if (Math.abs(snapTransform.getM22()) < parameters.getMinimumSurfaceNormalZ())
      {
         notifyListenerNodeForExpansionWasRejected(bipedalFootstepPlannerNode, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return null;
      }

      BipedalFootstepPlannerNode nodeAfterSnap = new BipedalFootstepPlannerNode(bipedalFootstepPlannerNode);
      nodeAfterSnap.transformSoleTransformWithSnapTransformFromZeroZ(snapTransform, planarRegionToPack);
      notifyListenerNodeSnappedAndStillSelectedForExpansion(nodeAfterSnap);

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
         notifyListenerNodeForExpansionWasRejected(nodeAfterSnap, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_WIGGLE_INSIDE);

         //TODO: Possibly have different node scores depending on how firm on ground they are.
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

      Point3d wiggleTranslation = new Point3d();
      wiggleTransformLocalToLocal.transform(wiggleTranslation);
      Vector3d wiggleVector = new Vector3d(wiggleTranslation);
      if (wiggleVector.length() > parameters.getMaximumXYWiggleDistance())
      {
         wiggleVector.scale(parameters.getMaximumXYWiggleDistance()/wiggleVector.length());
      }

      Vector3d rotationEuler = new Vector3d();
      wiggleTransformLocalToLocal.getRotationEuler(rotationEuler);
      double yaw = rotationEuler.getZ();
      yaw = MathTools.clipToMinMax(yaw, parameters.getMaximumYawWiggle());

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

                  if (zPenetration > parameters.getMaximumZPenetrationOnVRegions())
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
