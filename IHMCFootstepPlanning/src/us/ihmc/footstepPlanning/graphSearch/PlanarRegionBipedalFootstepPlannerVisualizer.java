package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPlanarRegionsList;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.TickAndUpdatable;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionBipedalFootstepPlannerVisualizer implements BipedalFootstepPlannerListener
{
   private static final double FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT = 0.01;
   private final int numberOfSolutionPolygons;

   private boolean verbose = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final IntegerYoVariable plannerUpdateIndex = new IntegerYoVariable("plannerUpdateIndex", registry);
   private final IntegerYoVariable planarRegionUpdateIndex = new IntegerYoVariable("planarRegionUpdateIndex", registry);
   private final DoubleYoVariable plannerTime = new DoubleYoVariable("plannerTime", registry);

   private final YoFrameConvexPolygon2d leftFootstepGoal, rightFootstepGoal;
   private final YoFrameConvexPolygon2d leftFootstepToExpand, rightFootstepToExpand;
   private final YoFrameConvexPolygon2d leftAcceptedFootstep, rightAcceptedFootstep;
   private final YoFrameConvexPolygon2d leftRejectedFootstep, rightRejectedFootstep;

   private final YoGraphicPolygon leftFootstepGoalViz, rightFootstepGoalViz;
   private final YoGraphicPolygon leftFootstepToExpandViz, rightFootstepToExpandViz;
   private final YoGraphicPolygon leftAcceptedFootstepViz, rightAcceptedFootstepViz;
   private final YoGraphicPolygon leftRejectedFootstepViz, rightRejectedFootstepViz;

   private final SideDependentList<ArrayList<YoGraphicPolygon>> solvedPlanFootstepsViz;
   private final SideDependentList<ArrayList<YoFrameConvexPolygon2d>> solvedPlanFootstepsPolygons;
   
   private final SideDependentList<YoGraphicPolygon> footstepGoalsViz, footstepsToExpandViz, acceptedFootstepsViz, rejectedFootstepsViz;

   private final BooleanYoVariable leftNodeIsAtGoal, rightNodeIsAtGoal;
   private final SideDependentList<BooleanYoVariable> nodeIsAtGoal;

   private final EnumYoVariable<BipedalFootstepPlannerNodeRejectionReason> nodeRejectedReason;

   private final YoFrameVector leftAcceptedFootstepSurfaceNormal, rightAcceptedFootstepSurfaceNormal;
   private final SideDependentList<YoFrameVector> acceptedFootstepSurfaceNormals;

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private TickAndUpdatable tickAndUpdatable;

   public PlanarRegionBipedalFootstepPlannerVisualizer(int numberOfSolutionPolygons, SideDependentList<ConvexPolygon2d> feetPolygonsInSoleFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.numberOfSolutionPolygons = numberOfSolutionPolygons;

      ConvexPolygon2d leftFootInSoleFrame = feetPolygonsInSoleFrame.get(RobotSide.LEFT);
      ConvexPolygon2d rightFootInSoleFrame = feetPolygonsInSoleFrame.get(RobotSide.RIGHT);

      int maxNumberOfVertices = leftFootInSoleFrame.getNumberOfVertices();

      leftFootstepGoal = new YoFrameConvexPolygon2d("leftFootstepGoal", worldFrame, maxNumberOfVertices, registry);
      rightFootstepGoal = new YoFrameConvexPolygon2d("rightFootstepGoal", worldFrame, maxNumberOfVertices, registry);
      leftFootstepToExpand = new YoFrameConvexPolygon2d("leftFootstepToExpand", worldFrame, maxNumberOfVertices, registry);
      rightFootstepToExpand = new YoFrameConvexPolygon2d("rightFootstepToExpand", worldFrame, maxNumberOfVertices, registry);
      leftAcceptedFootstep = new YoFrameConvexPolygon2d("leftAcceptedFootstep", worldFrame, maxNumberOfVertices, registry);
      rightAcceptedFootstep = new YoFrameConvexPolygon2d("rightAcceptedFootstep", worldFrame, maxNumberOfVertices, registry);
      leftRejectedFootstep = new YoFrameConvexPolygon2d("leftRejectedFootstep", worldFrame, maxNumberOfVertices, registry);
      rightRejectedFootstep = new YoFrameConvexPolygon2d("rightRejectedFootstep", worldFrame, maxNumberOfVertices, registry);

      leftFootstepGoal.setConvexPolygon2d(leftFootInSoleFrame);
      rightFootstepGoal.setConvexPolygon2d(rightFootInSoleFrame);
      leftFootstepToExpand.setConvexPolygon2d(leftFootInSoleFrame);
      rightFootstepToExpand.setConvexPolygon2d(rightFootInSoleFrame);
      leftAcceptedFootstep.setConvexPolygon2d(leftFootInSoleFrame);
      rightAcceptedFootstep.setConvexPolygon2d(rightFootInSoleFrame);
      leftRejectedFootstep.setConvexPolygon2d(leftFootInSoleFrame);
      rightRejectedFootstep.setConvexPolygon2d(rightFootInSoleFrame);

      leftFootstepGoalViz = new YoGraphicPolygon("leftFootstepGoalViz", leftFootstepGoal, "leftFootstepGoalPose", "", registry, 1.0, YoAppearance.Chocolate());
      rightFootstepGoalViz = new YoGraphicPolygon("rightFootstepGoalViz", rightFootstepGoal, "rightFootstepGoalPose", "", registry, 1.0, YoAppearance.GreenYellow());
      leftFootstepToExpandViz = new YoGraphicPolygon("leftFootstepToExpandViz", leftFootstepToExpand, "leftFootstepToExpandPose", "", registry, 1.0, YoAppearance.Yellow());
      rightFootstepToExpandViz = new YoGraphicPolygon("rightFootstepToExpandViz", rightFootstepToExpand, "rightFootstepToExpandPose", "", registry, 1.0, YoAppearance.Pink());
      leftAcceptedFootstepViz = new YoGraphicPolygon("leftAcceptedFootstepViz", leftAcceptedFootstep, "leftAcceptedFootstepPose", "", registry, 1.0, YoAppearance.Green());
      rightAcceptedFootstepViz = new YoGraphicPolygon("rightAcceptedFootstepViz", rightAcceptedFootstep, "rightAcceptedFootstepPose", "", registry, 1.0, YoAppearance.DarkGreen());
      leftRejectedFootstepViz = new YoGraphicPolygon("leftRejectedFootstepViz", leftRejectedFootstep, "leftRejectedFootstepPose", "", registry, 1.0, YoAppearance.Red());
      rightRejectedFootstepViz = new YoGraphicPolygon("rightRejectedFootstepViz", rightRejectedFootstep, "rightRejectedFootstepPose", "", registry, 1.0, YoAppearance.DarkRed());

      leftFootstepGoalViz.setPoseToNaN();
      rightFootstepGoalViz.setPoseToNaN();
      leftFootstepToExpandViz.setPoseToNaN();
      rightFootstepToExpandViz.setPoseToNaN();
      leftAcceptedFootstepViz.setPoseToNaN();
      rightAcceptedFootstepViz.setPoseToNaN();
      leftRejectedFootstepViz.setPoseToNaN();
      rightRejectedFootstepViz.setPoseToNaN();

      footstepGoalsViz = new SideDependentList<>(leftFootstepGoalViz, rightFootstepGoalViz);
      footstepsToExpandViz = new SideDependentList<>(leftFootstepToExpandViz, rightFootstepToExpandViz);
      acceptedFootstepsViz = new SideDependentList<>(leftAcceptedFootstepViz, rightAcceptedFootstepViz);
      rejectedFootstepsViz = new SideDependentList<>(leftRejectedFootstepViz, rightRejectedFootstepViz);

      leftNodeIsAtGoal = new BooleanYoVariable("leftNodeIsAtGoal", registry);
      rightNodeIsAtGoal = new BooleanYoVariable("rightNodeIsAtGoal", registry);
      nodeIsAtGoal = new SideDependentList<>(leftNodeIsAtGoal, rightNodeIsAtGoal);

      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepGoalViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepGoalViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepToExpandViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepToExpandViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftAcceptedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightAcceptedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftRejectedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightRejectedFootstepViz);

      solvedPlanFootstepsViz = new SideDependentList<ArrayList<YoGraphicPolygon>>(new ArrayList<YoGraphicPolygon>(), new ArrayList<YoGraphicPolygon>());
      solvedPlanFootstepsPolygons = new SideDependentList<ArrayList<YoFrameConvexPolygon2d>>(new ArrayList<YoFrameConvexPolygon2d>(), new ArrayList<YoFrameConvexPolygon2d>());
      
      for (int i=0; i<numberOfSolutionPolygons; i++)
      {
         YoFrameConvexPolygon2d leftFootstepSolvedPlan = new YoFrameConvexPolygon2d("leftFootstepSolvedPlan" + i, worldFrame, maxNumberOfVertices, registry);
         leftFootstepSolvedPlan.setConvexPolygon2d(leftFootInSoleFrame);

         YoGraphicPolygon leftFootstepSolvedPlanViz = new YoGraphicPolygon("leftFootstepSolvedPlanViz" + i, leftFootstepSolvedPlan, "leftFootstepSolvedPlan" + i, "", registry, 1.0, YoAppearance.Green());
         graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepSolvedPlanViz);
         
         YoFrameConvexPolygon2d rightFootstepSolvedPlan = new YoFrameConvexPolygon2d("rightFootstepSolvedPlan" + i, worldFrame, maxNumberOfVertices, registry);
         rightFootstepSolvedPlan.setConvexPolygon2d(rightFootInSoleFrame);

         YoGraphicPolygon rightFootstepSolvedPlanViz = new YoGraphicPolygon("rightFootstepSolvedPlanViz" + i, rightFootstepSolvedPlan, "rightFootstepSolvedPlan" + i, "", registry, 1.0, YoAppearance.DarkGreen());
         graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepSolvedPlanViz);

         solvedPlanFootstepsViz.get(RobotSide.LEFT).add(leftFootstepSolvedPlanViz);
         solvedPlanFootstepsViz.get(RobotSide.RIGHT).add(rightFootstepSolvedPlanViz);
         
         solvedPlanFootstepsPolygons.get(RobotSide.LEFT).add(leftFootstepSolvedPlan);
         solvedPlanFootstepsPolygons.get(RobotSide.RIGHT).add(rightFootstepSolvedPlan);
         
         leftFootstepSolvedPlanViz.setPoseToNaN();
         rightFootstepSolvedPlanViz.setPoseToNaN();
         
      }
      
      leftAcceptedFootstepSurfaceNormal = new YoFrameVector("leftAcceptedFootstepSurfaceNormal", worldFrame, registry);
      rightAcceptedFootstepSurfaceNormal = new YoFrameVector("rightAcceptedFootstepSurfaceNormal", worldFrame, registry);
      acceptedFootstepSurfaceNormals = new SideDependentList<>(leftAcceptedFootstepSurfaceNormal, rightAcceptedFootstepSurfaceNormal);

      nodeRejectedReason = new EnumYoVariable<>("nodeRejectedReason", registry, BipedalFootstepPlannerNodeRejectionReason.class, true);
      nodeRejectedReason.set(null);

      int vertexBufferSize = 100;
      int meshBufferSize = 2000;
      yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("planarRegionsList", vertexBufferSize, meshBufferSize, registry);
      graphicsListRegistry.registerYoGraphic("PlanarRegionsList", yoGraphicPlanarRegionsList);

      parentRegistry.addChild(registry);
   }
   
   /**
    * Sets the TickAndUpdatable that should be tickAndUpdated each viz step.
    * @param tickAndUpdatable
    */
   public void setTickAndUpdatable(TickAndUpdatable tickAndUpdatable)
   {
      this.tickAndUpdatable = tickAndUpdatable;
   }

   public TickAndUpdatable getTickAndUpdatable()
   {
      return tickAndUpdatable;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   @Override
   public void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose)
   {
      plannerUpdateIndex.increment();

      footstepGoalsViz.get(RobotSide.LEFT).setTransformToWorld(goalLeftFootPose);
      footstepGoalsViz.get(RobotSide.RIGHT).setTransformToWorld(goalRightFootPose);

      footstepGoalsViz.get(RobotSide.LEFT).update();
      footstepGoalsViz.get(RobotSide.RIGHT).update();
      tickAndUpdate();
   }

   @Override
   public void nodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand)
   {
      plannerUpdateIndex.increment();
      RobotSide robotSide = nodeToExpand.getRobotSide();

      BipedalFootstepPlannerNode parentNode = nodeToExpand.getParentNode();

      if (parentNode != null)
      {
         drawAcceptedFootstepNode(parentNode);
      }
      else
      {
         acceptedFootstepsViz.get(robotSide.getOppositeSide()).setPoseToNaN();
      }

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      nodeToExpand.getSoleTransform(soleTransform);

      nodeIsAtGoal.get(robotSide).set(nodeToExpand.isAtGoal());

      if (verbose)
      {
         System.out.println("Node selected for expansion:");
         System.out.println(nodeToExpand);
      }

      acceptedFootstepsViz.get(robotSide).setPoseToNaN();
      rejectedFootstepsViz.get(robotSide).setPoseToNaN();
      nodeRejectedReason.set(null);

      YoGraphicPolygon footstepToExpandViz = footstepsToExpandViz.get(robotSide);
      footstepToExpandViz.setTransformToWorld(soleTransform);

      moveUpSlightlyToEnsureVisible(footstepToExpandViz);
      footstepToExpandViz.update();
      tickAndUpdate();
   }

   @Override
   public void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode acceptedNode)
   {
      plannerUpdateIndex.increment();

      drawAcceptedFootstepNode(acceptedNode);
      tickAndUpdate();
   }

   private void drawAcceptedFootstepNode(BipedalFootstepPlannerNode acceptedNode)
   {
      RobotSide robotSide = acceptedNode.getRobotSide();
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      acceptedNode.getSoleTransform(soleTransform);

      nodeIsAtGoal.get(robotSide).set(acceptedNode.isAtGoal());

      if (verbose)
      {
         System.out.println("Node accepted:");
         System.out.println(acceptedNode);
      }

      footstepsToExpandViz.get(robotSide).setPoseToNaN();
      rejectedFootstepsViz.get(robotSide).setPoseToNaN();

      YoGraphicPolygon acceptedFootstepViz = acceptedFootstepsViz.get(robotSide);
      acceptedFootstepViz.setTransformToWorld(soleTransform);

      Vector3d surfaceNormal = new Vector3d(0.0, 0.0, 1.0);
      soleTransform.transform(surfaceNormal);
      acceptedFootstepSurfaceNormals.get(robotSide).set(surfaceNormal);

      moveUpSlightlyToEnsureVisible(acceptedFootstepViz);
      acceptedFootstepViz.update();
   }

   @Override
   public void nodeForExpansionWasRejected(BipedalFootstepPlannerNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      plannerUpdateIndex.increment();

      RobotSide robotSide = rejectedNode.getRobotSide();
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      rejectedNode.getSoleTransform(soleTransform);

      nodeIsAtGoal.get(robotSide).set(rejectedNode.isAtGoal());

      if (verbose)
      {
         System.out.println("Node rejected:");
         System.out.println(rejectedNode);
      }

      footstepsToExpandViz.get(robotSide).setPoseToNaN();
      acceptedFootstepsViz.get(robotSide).setPoseToNaN();

      YoGraphicPolygon rejectedFootstepViz = rejectedFootstepsViz.get(robotSide);
      rejectedFootstepViz.setTransformToWorld(soleTransform);

      moveUpSlightlyToEnsureVisible(rejectedFootstepViz);
      rejectedFootstepViz.update();

      nodeRejectedReason.set(reason);
      tickAndUpdate();
   }

   private void moveUpSlightlyToEnsureVisible(YoGraphicPolygon footstepToExpandViz)
   {
      FramePoint framePointToPack = new FramePoint(worldFrame);
      footstepToExpandViz.getPosition(framePointToPack);
      framePointToPack.setZ(framePointToPack.getZ() + 0.0025);
      footstepToExpandViz.setPosition(framePointToPack);
   }

   @Override
   public void notifyListenerSolutionWasFound(FootstepPlan footstepPlan)
   {
      plannerUpdateIndex.set(0);

      hideExpandingFootstepViz();
      
      setPlanVizToNaN();

      int numberOfSteps = footstepPlan.getNumberOfSteps();
      FramePose soleFramePose = new FramePose();

      int leftIndex = 0;
      int rightIndex = 0;
      
      for (int i=0; i<numberOfSteps; i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);
         RobotSide robotSide = footstep.getRobotSide();
         footstep.getSoleFramePose(soleFramePose);
         soleFramePose.changeFrame(worldFrame);
         
         int index;
         if (robotSide == RobotSide.LEFT)
         {
            index = leftIndex;
         }
         else
         {
            index = rightIndex;
         }
         
         ArrayList<YoGraphicPolygon> polygonsViz = solvedPlanFootstepsViz.get(robotSide);
         YoGraphicPolygon yoGraphicPolygon = polygonsViz.get(index);
         yoGraphicPolygon.setPose(soleFramePose);
         index++;

         if (index >= numberOfSolutionPolygons)
         {
            leftIndex = 0;
            rightIndex = 0;
            tickAndUpdate();
         }
         
         else
         {
            if (robotSide == RobotSide.LEFT)
            {
               leftIndex = index;
            }
            else
            {
               rightIndex = index;
            }
         }
      }

      if (verbose)
      {
         System.out.println("Solution Found!");
      }
      
      tickAndUpdate();
   }

   private void hideExpandingFootstepViz()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footstepsToExpandViz.get(robotSide).setPoseToNaN();
         acceptedFootstepsViz.get(robotSide).setPoseToNaN();
         rejectedFootstepsViz.get(robotSide).setPoseToNaN();
      }
   }

   @Override
   public void notifyListenerSolutionWasNotFound()
   {
      plannerUpdateIndex.set(0);

      hideExpandingFootstepViz();

      setPlanVizToNaN();
      if (verbose)
      {
         System.out.println("Solution Found!");
      }
      
      tickAndUpdate();
   }

   private void setPlanVizToNaN()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<YoGraphicPolygon> polygonsViz = solvedPlanFootstepsViz.get(robotSide);
         for (YoGraphicPolygon polygonViz : polygonsViz)
         {
            polygonViz.setPoseToNaN();
         }
      }
   }

   @Override
   public void planarRegionsListSet(PlanarRegionsList planarRegionsList)
   {
      planarRegionUpdateIndex.set(0);

      yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList);
      while (!yoGraphicPlanarRegionsList.isQueueEmpty())
      {
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
         planarRegionUpdateIndex.increment();
         tickAndUpdate();
      }
      
      tickAndUpdate();
   }

   private void tickAndUpdate()
   {
      plannerTime.add(FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT);

      if (tickAndUpdatable != null)
      {
         tickAndUpdatable.tickAndUpdate(plannerTime.getDoubleValue());
      }
   }

}
