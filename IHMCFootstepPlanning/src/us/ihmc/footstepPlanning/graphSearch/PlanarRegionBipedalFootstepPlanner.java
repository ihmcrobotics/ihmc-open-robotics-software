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

   protected final DoubleYoVariable stepReach = new DoubleYoVariable("stepReach", registry);

   protected final IntegerYoVariable numberOfNodesExpanded = new IntegerYoVariable("numberOfNodesExpanded", registry);

   protected final DoubleYoVariable footArea = new DoubleYoVariable("footArea", registry);
   protected final DoubleYoVariable totalArea = new DoubleYoVariable("totalArea", registry);

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

         
         boolean nodeIsAcceptableToExpand = checkNodeAcceptableToExpand(nodeToExpand);
         
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

         RigidBodyTransform soleZUpTransform = new RigidBodyTransform();
         nodeToExpand.getSoleTransform(soleZUpTransform);
         setTransformZUpPreserveX(soleZUpTransform);

         expandChildrenAndAddNodes(stack, soleZUpTransform, nodeToExpand);
      }

      notifyListenerSolutionWasNotFound();
      return FootstepPlanningResult.NO_PATH_EXISTS;
   }

   private boolean checkNodeAcceptableToExpand(BipedalFootstepPlannerNode nodeToExpand)
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

   protected void expandChildrenAndAddNodes(Deque<BipedalFootstepPlannerNode> stack, RigidBodyTransform soleZUpTransform, BipedalFootstepPlannerNode nodeToExpand)
   {
      BipedalFootstepPlannerNode goalNode = this.planarRegionPotentialNextStepCalculator.computeGoalNodeIfGoalIsReachable(nodeToExpand, soleZUpTransform);
      if (goalNode != null)
      {
         stack.push(goalNode);
         return;
      }
      
      ArrayList<BipedalFootstepPlannerNode> nodesToAdd = planarRegionPotentialNextStepCalculator.computeChildrenNodes(soleZUpTransform, nodeToExpand);

      //      Collections.shuffle(nodesToAdd);
      Collections.reverse(nodesToAdd);

      for (BipedalFootstepPlannerNode node : nodesToAdd)
      {
         stack.push(node);
      } 
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
