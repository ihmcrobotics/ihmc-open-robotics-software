package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class PlanarRegionBipedalFootstepPlannerVisualizer implements BipedalFootstepPlannerListener
{
   private static final double FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT = 0.01;
   private final int numberOfSolutionPolygons;

   private boolean verbose = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final YoInteger plannerUpdateIndex = new YoInteger("plannerUpdateIndex", registry);
   private final YoInteger planarRegionUpdateIndex = new YoInteger("planarRegionUpdateIndex", registry);
   private final YoDouble plannerTime = new YoDouble("plannerTime", registry);
   private FootstepNodeSnapperReadOnly snapper = null;

   private final YoFrameConvexPolygon2D leftFootstepStart, rightFootstepStart;
   private final YoFrameConvexPolygon2D leftFootstepGoal, rightFootstepGoal;
   private final YoFrameConvexPolygon2D leftFootstepUnderConsideration, rightFootstepUnderConsideration;
   private final YoFrameConvexPolygon2D leftAcceptedFootstep, rightAcceptedFootstep;
   private final YoFrameConvexPolygon2D leftRejectedFootstep, rightRejectedFootstep;

   private final YoGraphicPolygon leftFootstepStartViz, rightFootstepStartViz;
   private final YoGraphicPolygon leftFootstepGoalViz, rightFootstepGoalViz;
   private final YoGraphicPolygon leftFootstepToExpandViz, rightFootstepToExpandViz;
   private final YoGraphicPolygon leftAcceptedFootstepViz, rightAcceptedFootstepViz;
   private final YoGraphicPolygon leftRejectedFootstepViz, rightRejectedFootstepViz;

   private final SideDependentList<ArrayList<YoGraphicPolygon>> solvedPlanFootstepsViz;
   private final SideDependentList<ArrayList<YoFrameConvexPolygon2D>> solvedPlanFootstepsPolygons;
   
   private final SideDependentList<YoGraphicPolygon> footstepStartsViz, footstepGoalsViz, footstepsUnderConsiderationViz, acceptedFootstepsViz, rejectedFootstepsViz;
   private final YoEnum<BipedalFootstepPlannerNodeRejectionReason> nodeRejectedReason;

   private final YoFrameVector3D leftAcceptedFootstepSurfaceNormal, rightAcceptedFootstepSurfaceNormal;
   private final SideDependentList<YoFrameVector3D> acceptedFootstepSurfaceNormals;

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private final EnumMap<BipedalFootstepPlannerNodeRejectionReason, YoInteger> rejectionReasonCount = new EnumMap<BipedalFootstepPlannerNodeRejectionReason, YoInteger>(BipedalFootstepPlannerNodeRejectionReason.class);

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private TickAndUpdatable tickAndUpdatable;

   public PlanarRegionBipedalFootstepPlannerVisualizer(int numberOfSolutionPolygons, SideDependentList<ConvexPolygon2D> feetPolygonsInSoleFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.numberOfSolutionPolygons = numberOfSolutionPolygons;

      ConvexPolygon2D leftFootInSoleFrame = feetPolygonsInSoleFrame.get(RobotSide.LEFT);
      ConvexPolygon2D rightFootInSoleFrame = feetPolygonsInSoleFrame.get(RobotSide.RIGHT);

      for(BipedalFootstepPlannerNodeRejectionReason reason : BipedalFootstepPlannerNodeRejectionReason.values())
      {
         rejectionReasonCount.put(reason, new YoInteger(reason.name().toLowerCase() + "_count", registry));
      }

      int maxNumberOfVertices = leftFootInSoleFrame.getNumberOfVertices();

      leftFootstepStart = new YoFrameConvexPolygon2D("leftFootstepStart", worldFrame, maxNumberOfVertices, registry);
      rightFootstepStart = new YoFrameConvexPolygon2D("rightFootstepStart", worldFrame, maxNumberOfVertices, registry);
      leftFootstepGoal = new YoFrameConvexPolygon2D("leftFootstepGoal", worldFrame, maxNumberOfVertices, registry);
      rightFootstepGoal = new YoFrameConvexPolygon2D("rightFootstepGoal", worldFrame, maxNumberOfVertices, registry);
      leftFootstepUnderConsideration = new YoFrameConvexPolygon2D("leftFootstepUnderConsideration", worldFrame, maxNumberOfVertices, registry);
      rightFootstepUnderConsideration = new YoFrameConvexPolygon2D("rightFootstepUnderConsideration", worldFrame, maxNumberOfVertices, registry);
      leftAcceptedFootstep = new YoFrameConvexPolygon2D("leftAcceptedFootstep", worldFrame, maxNumberOfVertices, registry);
      rightAcceptedFootstep = new YoFrameConvexPolygon2D("rightAcceptedFootstep", worldFrame, maxNumberOfVertices, registry);
      leftRejectedFootstep = new YoFrameConvexPolygon2D("leftRejectedFootstep", worldFrame, maxNumberOfVertices, registry);
      rightRejectedFootstep = new YoFrameConvexPolygon2D("rightRejectedFootstep", worldFrame, maxNumberOfVertices, registry);

      leftFootstepStart.set(leftFootInSoleFrame);
      rightFootstepStart.set(leftFootInSoleFrame);
      leftFootstepGoal.set(leftFootInSoleFrame);
      rightFootstepGoal.set(rightFootInSoleFrame);
      leftFootstepUnderConsideration.set(leftFootInSoleFrame);
      rightFootstepUnderConsideration.set(rightFootInSoleFrame);
      leftAcceptedFootstep.set(leftFootInSoleFrame);
      rightAcceptedFootstep.set(rightFootInSoleFrame);
      leftRejectedFootstep.set(leftFootInSoleFrame);
      rightRejectedFootstep.set(rightFootInSoleFrame);

      leftFootstepStartViz = new YoGraphicPolygon("leftFootstepStartViz", leftFootstepStart, "leftFootstepStartPose", "", registry, true, 1.0, YoAppearance.Gold());
      rightFootstepStartViz = new YoGraphicPolygon("rightFootstepStartViz", rightFootstepStart, "rightFootstepStartPose", "", registry, true, 1.0, YoAppearance.Gold());
      leftFootstepGoalViz = new YoGraphicPolygon("leftFootstepGoalViz", leftFootstepGoal, "leftFootstepGoalPose", "", registry, true, 1.0, YoAppearance.Chocolate());
      rightFootstepGoalViz = new YoGraphicPolygon("rightFootstepGoalViz", rightFootstepGoal, "rightFootstepGoalPose", "", registry, true, 1.0, YoAppearance.Chocolate());
      leftFootstepToExpandViz = new YoGraphicPolygon("leftFootstepToExpandViz", leftFootstepUnderConsideration, "leftFootstepToExpandPose", "", registry, true, 1.0, YoAppearance.Yellow());
      rightFootstepToExpandViz = new YoGraphicPolygon("rightFootstepToExpandViz", rightFootstepUnderConsideration, "rightFootstepToExpandPose", "", registry, true, 1.0, YoAppearance.Yellow());
      leftAcceptedFootstepViz = new YoGraphicPolygon("leftAcceptedFootstepViz", leftAcceptedFootstep, "leftAcceptedFootstepPose", "", registry, true, 1.0, YoAppearance.Green());
      rightAcceptedFootstepViz = new YoGraphicPolygon("rightAcceptedFootstepViz", rightAcceptedFootstep, "rightAcceptedFootstepPose", "", registry, true, 1.0, YoAppearance.DarkGreen());
      leftRejectedFootstepViz = new YoGraphicPolygon("leftRejectedFootstepViz", leftRejectedFootstep, "leftRejectedFootstepPose", "", registry, true, 1.0, YoAppearance.Red());
      rightRejectedFootstepViz = new YoGraphicPolygon("rightRejectedFootstepViz", rightRejectedFootstep, "rightRejectedFootstepPose", "", registry, true, 1.0, YoAppearance.DarkRed());

      leftFootstepStartViz.setPoseToNaN();
      rightFootstepStartViz.setPoseToNaN();
      leftFootstepGoalViz.setPoseToNaN();
      rightFootstepGoalViz.setPoseToNaN();
      leftFootstepToExpandViz.setPoseToNaN();
      rightFootstepToExpandViz.setPoseToNaN();
      leftAcceptedFootstepViz.setPoseToNaN();
      rightAcceptedFootstepViz.setPoseToNaN();
      leftRejectedFootstepViz.setPoseToNaN();
      rightRejectedFootstepViz.setPoseToNaN();

      footstepStartsViz = new SideDependentList<>(leftFootstepStartViz, rightFootstepStartViz);
      footstepGoalsViz = new SideDependentList<>(leftFootstepGoalViz, rightFootstepGoalViz);
      footstepsUnderConsiderationViz = new SideDependentList<>(leftFootstepToExpandViz, rightFootstepToExpandViz);
      acceptedFootstepsViz = new SideDependentList<>(leftAcceptedFootstepViz, rightAcceptedFootstepViz);
      rejectedFootstepsViz = new SideDependentList<>(leftRejectedFootstepViz, rightRejectedFootstepViz);

      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepStartViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepStartViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepGoalViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepGoalViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepToExpandViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepToExpandViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftAcceptedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightAcceptedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftRejectedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightRejectedFootstepViz);

      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(leftFootstepStartViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(rightFootstepStartViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(leftFootstepGoalViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(rightFootstepGoalViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(leftFootstepToExpandViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(rightFootstepToExpandViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(leftAcceptedFootstepViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(rightAcceptedFootstepViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(leftRejectedFootstepViz);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(rightRejectedFootstepViz);

      solvedPlanFootstepsViz = new SideDependentList<ArrayList<YoGraphicPolygon>>(new ArrayList<YoGraphicPolygon>(), new ArrayList<YoGraphicPolygon>());
      solvedPlanFootstepsPolygons = new SideDependentList<ArrayList<YoFrameConvexPolygon2D>>(new ArrayList<YoFrameConvexPolygon2D>(), new ArrayList<YoFrameConvexPolygon2D>());

      for (int i=0; i<numberOfSolutionPolygons; i++)
      {
         YoFrameConvexPolygon2D leftFootstepSolvedPlan = new YoFrameConvexPolygon2D("leftFootstepSolvedPlan" + i, worldFrame, maxNumberOfVertices, registry);
         leftFootstepSolvedPlan.set(leftFootInSoleFrame);

         YoGraphicPolygon leftFootstepSolvedPlanViz = new YoGraphicPolygon("leftFootstepSolvedPlanViz" + i, leftFootstepSolvedPlan, "leftFootstepSolvedPlan" + i, "", registry, true, 1.0, YoAppearance.Purple());
         graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepSolvedPlanViz);
         
         YoFrameConvexPolygon2D rightFootstepSolvedPlan = new YoFrameConvexPolygon2D("rightFootstepSolvedPlan" + i, worldFrame, maxNumberOfVertices, registry);
         rightFootstepSolvedPlan.set(rightFootInSoleFrame);

         YoGraphicPolygon rightFootstepSolvedPlanViz = new YoGraphicPolygon("rightFootstepSolvedPlanViz" + i, rightFootstepSolvedPlan, "rightFootstepSolvedPlan" + i, "", registry, true, 1.0, YoAppearance.Purple());
         graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepSolvedPlanViz);

         solvedPlanFootstepsViz.get(RobotSide.LEFT).add(leftFootstepSolvedPlanViz);
         solvedPlanFootstepsViz.get(RobotSide.RIGHT).add(rightFootstepSolvedPlanViz);
         
         solvedPlanFootstepsPolygons.get(RobotSide.LEFT).add(leftFootstepSolvedPlan);
         solvedPlanFootstepsPolygons.get(RobotSide.RIGHT).add(rightFootstepSolvedPlan);
         
         leftFootstepSolvedPlanViz.setPoseToNaN();
         rightFootstepSolvedPlanViz.setPoseToNaN();
         
      }
      
      leftAcceptedFootstepSurfaceNormal = new YoFrameVector3D("leftAcceptedFootstepSurfaceNormal", worldFrame, registry);
      rightAcceptedFootstepSurfaceNormal = new YoFrameVector3D("rightAcceptedFootstepSurfaceNormal", worldFrame, registry);
      acceptedFootstepSurfaceNormals = new SideDependentList<>(leftAcceptedFootstepSurfaceNormal, rightAcceptedFootstepSurfaceNormal);

      nodeRejectedReason = new YoEnum<>("nodeRejectedReason", registry, BipedalFootstepPlannerNodeRejectionReason.class, true);
      nodeRejectedReason.set(null);

      int vertexBufferSize = 100;
      int meshBufferSize = 2000;
      yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("planarRegionsList", vertexBufferSize, meshBufferSize, registry);
      graphicsListRegistry.registerYoGraphic("PlanarRegionsList", yoGraphicPlanarRegionsList);
      graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(yoGraphicPlanarRegionsList);

      parentRegistry.addChild(registry);
   }

   public void setFootstepSnapper(FootstepNodeSnapperReadOnly snapper)
   {
      this.snapper = snapper;
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
   public void nodeUnderConsideration(FootstepNode nodeToExpand)
   {
      plannerUpdateIndex.increment();
      RobotSide robotSide = nodeToExpand.getRobotSide();

      acceptedFootstepsViz.get(robotSide).setPoseToNaN();
      rejectedFootstepsViz.get(robotSide).setPoseToNaN();
      nodeRejectedReason.set(null);

      if (verbose)
      {
         System.out.println("Node selected for expansion:");
         System.out.println(nodeToExpand);
      }

      YoGraphicPolygon footstepUnderConsiderationViz = footstepsUnderConsiderationViz.get(robotSide);
      setSoleGraphicTransform(nodeToExpand, footstepUnderConsiderationViz);

      moveUpSlightlyToEnsureVisible(footstepUnderConsiderationViz);
      footstepUnderConsiderationViz.update();
      tickAndUpdate();
   }

   private final RigidBodyTransform soleTransform = new RigidBodyTransform();

   private void setSoleGraphicTransform(FootstepNode nodeToExpand, YoGraphicPolygon footstepUnderConsiderationViz)
   {
      if(snapper != null)
      {
         RigidBodyTransform snapTransform = snapper.getSnapData(nodeToExpand).getSnapTransform();
         FootstepNodeTools.getSnappedNodeTransform(nodeToExpand, snapTransform, soleTransform);
      }
      else
      {
         FootstepNodeTools.getNodeTransform(nodeToExpand, soleTransform);
      }

      footstepUnderConsiderationViz.setTransformToWorld(soleTransform);
   }

   @Override
   public void startNodeWasAdded(FootstepNode startNode)
   {
      plannerUpdateIndex.increment();
      
      RobotSide robotSide = startNode.getRobotSide();
      hideExpandingFootstepViz();
      
      YoGraphicPolygon startNodeViz = footstepStartsViz.get(robotSide);
      setSoleGraphicTransform(startNode, startNodeViz);

      moveUpSlightlyToEnsureVisible(startNodeViz);
      startNodeViz.update();
      
      YoGraphicPolygon otherStartNodesViz = footstepStartsViz.get(robotSide.getOppositeSide());
      otherStartNodesViz.setPoseToNaN();

      tickAndUpdate();
   }

   @Override
   public void nodeIsBeingExpanded(FootstepNode nodeToExpand)
   {
      plannerUpdateIndex.increment();

      YoGraphicPolygon footstepViz = nodeToExpand.getRobotSide().equals(RobotSide.LEFT) ? leftFootstepToExpandViz : rightFootstepToExpandViz;
      setSoleGraphicTransform(nodeToExpand, footstepViz);
      tickAndUpdate();
   }

   private void drawAcceptedFootstepNode(FootstepNode acceptedNode)
   {
      RobotSide robotSide = acceptedNode.getRobotSide();

      if (verbose)
      {
         System.out.println("Node accepted:");
         System.out.println(acceptedNode);
      }

      footstepsUnderConsiderationViz.get(robotSide).setPoseToNaN();
      rejectedFootstepsViz.get(robotSide).setPoseToNaN();

      YoGraphicPolygon acceptedFootstepViz = acceptedFootstepsViz.get(robotSide);
      setSoleGraphicTransform(acceptedNode, acceptedFootstepViz);

      Vector3D surfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      soleTransform.transform(surfaceNormal);
      acceptedFootstepSurfaceNormals.get(robotSide).set(surfaceNormal);

      moveUpSlightlyToEnsureVisible(acceptedFootstepViz);
      acceptedFootstepViz.update();
   }

   @Override
   public void nodeUnderConsiderationWasRejected(FootstepNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      plannerUpdateIndex.increment();
      rejectionReasonCount.get(reason).increment();

      RobotSide robotSide = rejectedNode.getRobotSide();

      if (verbose)
      {
         System.out.println("Node rejected:");
         System.out.println(rejectedNode);
      }

      footstepsUnderConsiderationViz.get(robotSide).setPoseToNaN();
      acceptedFootstepsViz.get(robotSide).setPoseToNaN();

      YoGraphicPolygon rejectedFootstepViz = rejectedFootstepsViz.get(robotSide);
      rejectedFootstepViz.setTransformToWorld(soleTransform);
      setSoleGraphicTransform(rejectedNode, rejectedFootstepViz);

      moveUpSlightlyToEnsureVisible(rejectedFootstepViz);
      rejectedFootstepViz.update();

      nodeRejectedReason.set(reason);
      tickAndUpdate();
   }

   @Override
   public void nodeUnderConsiderationWasSuccessful(FootstepNode successfulNode)
   {
      drawAcceptedFootstepNode(successfulNode);
      tickAndUpdate();
   }

   private void moveUpSlightlyToEnsureVisible(YoGraphicPolygon footstepToExpandViz)
   {
      FramePoint3D framePointToPack = new FramePoint3D(worldFrame);
      footstepToExpandViz.getPosition(framePointToPack);
      framePointToPack.setZ(framePointToPack.getZ() + 0.0025);
      footstepToExpandViz.setPosition(framePointToPack);
   }

   @Override
   public void solutionWasFound(FootstepPlan footstepPlan)
   {
      plannerUpdateIndex.set(0);

      hideExpandingFootstepViz();
      
      setPlanVizToNaN();

      int numberOfSteps = footstepPlan.getNumberOfSteps();
      FramePose3D soleFramePose = new FramePose3D();

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
         footstepsUnderConsiderationViz.get(robotSide).setPoseToNaN();
         acceptedFootstepsViz.get(robotSide).setPoseToNaN();
         rejectedFootstepsViz.get(robotSide).setPoseToNaN();
      }
   }

   @Override
   public void solutionWasNotFound()
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

      if(!yoGraphicPlanarRegionsList.isQueueEmpty())
      {
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
         planarRegionUpdateIndex.increment();
      }
   }

}
