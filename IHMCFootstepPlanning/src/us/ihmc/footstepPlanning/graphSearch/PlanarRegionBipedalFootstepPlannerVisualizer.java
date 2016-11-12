package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanarRegionBipedalFootstepPlannerVisualizer implements BipedalFootstepPlannerListener
{
   private boolean verbose = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameConvexPolygon2d leftFootstepGoal, rightFootstepGoal;
   private final YoFrameConvexPolygon2d leftFootstepToExpand, rightFootstepToExpand;
   private final YoFrameConvexPolygon2d leftAcceptedFootstep, rightAcceptedFootstep;
   private final YoFrameConvexPolygon2d leftRejectedFootstep, rightRejectedFootstep;

   private final YoGraphicPolygon leftFootstepGoalViz, rightFootstepGoalViz;
   private final YoGraphicPolygon leftFootstepToExpandViz, rightFootstepToExpandViz;
   private final YoGraphicPolygon leftAcceptedFootstepViz, rightAcceptedFootstepViz;
   private final YoGraphicPolygon leftRejectedFootstepViz, rightRejectedFootstepViz;

   private final SideDependentList<YoGraphicPolygon> footstepGoalsViz, footstepsToExpandViz, acceptedFootstepsViz, rejectedFootstepsViz;

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public PlanarRegionBipedalFootstepPlannerVisualizer(SideDependentList<ConvexPolygon2d> feetPolygonsInSoleFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
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

      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepGoalViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepGoalViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftFootstepToExpandViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightFootstepToExpandViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftAcceptedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightAcceptedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", leftRejectedFootstepViz);
      graphicsListRegistry.registerYoGraphic("FootstepPlanner", rightRejectedFootstepViz);

      parentRegistry.addChild(registry);
   }

   public void setVerbose(boolean verbose)
   {
      this.verbose = verbose;
   }

   @Override
   public void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose)
   {
      footstepGoalsViz.get(RobotSide.LEFT).setTransformToWorld(goalLeftFootPose);
      footstepGoalsViz.get(RobotSide.RIGHT).setTransformToWorld(goalRightFootPose);

      footstepGoalsViz.get(RobotSide.LEFT).update();
      footstepGoalsViz.get(RobotSide.RIGHT).update();
   }

   @Override
   public void nodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand)
   {
      RobotSide robotSide = nodeToExpand.getRobotSide();
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      nodeToExpand.getSoleTransform(soleTransform);

      if (verbose)
      {
         System.out.println("Node selected for expansion:");
         System.out.println(nodeToExpand);
      }

      acceptedFootstepsViz.get(robotSide).setPoseToNaN();
      rejectedFootstepsViz.get(robotSide).setPoseToNaN();

      YoGraphicPolygon footstepToExpandViz = footstepsToExpandViz.get(robotSide);
      footstepToExpandViz.setTransformToWorld(soleTransform);
      footstepToExpandViz.update();
   }

   @Override
   public void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode acceptedNode)
   {
      RobotSide robotSide = acceptedNode.getRobotSide();
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      acceptedNode.getSoleTransform(soleTransform);

      if (verbose)
      {
         System.out.println("Node accepted:");
         System.out.println(acceptedNode);
      }

      footstepsToExpandViz.get(robotSide).setPoseToNaN();
      rejectedFootstepsViz.get(robotSide).setPoseToNaN();

      YoGraphicPolygon footstepToExpandViz = acceptedFootstepsViz.get(robotSide);
      footstepToExpandViz.setTransformToWorld(soleTransform);
      footstepToExpandViz.update();
   }

   @Override
   public void nodeForExpansionWasRejected(BipedalFootstepPlannerNode rejectedNode)
   {
      RobotSide robotSide = rejectedNode.getRobotSide();
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      rejectedNode.getSoleTransform(soleTransform);

      if (verbose)
      {
         System.out.println("Node rejected:");
         System.out.println(rejectedNode);
      }

      footstepsToExpandViz.get(robotSide).setPoseToNaN();
      acceptedFootstepsViz.get(robotSide).setPoseToNaN();

      YoGraphicPolygon footstepToExpandViz = rejectedFootstepsViz.get(robotSide);
      footstepToExpandViz.setTransformToWorld(soleTransform);
      footstepToExpandViz.update();
   }

   @Override
   public void notifyListenerSolutionWasFound()
   {
      if (verbose)
      {
         System.out.println("Solution Found!");
      }
   }

   @Override
   public void notifyListenerSolutionWasNotFound()
   {
      if (verbose)
      {
         System.out.println("Solution Found!");
      }
   }

   @Override
   public void planarRegionsListSet(PlanarRegionsList planarRegionsList)
   {
   }

}
