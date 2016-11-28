package us.ihmc.footstepPlanning.graphSearch;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
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

   private final BooleanYoVariable leftNodeIsAtGoal, rightNodeIsAtGoal;
   private final SideDependentList<BooleanYoVariable> nodeIsAtGoal;

   private final EnumYoVariable<BipedalFootstepPlannerNodeRejectionReason> nodeRejectedReason;

   private final YoFrameVector leftAcceptedFootstepSurfaceNormal, rightAcceptedFootstepSurfaceNormal;
   private final SideDependentList<YoFrameVector> acceptedFootstepSurfaceNormals;

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

      leftAcceptedFootstepSurfaceNormal = new YoFrameVector("leftAcceptedFootstepSurfaceNormal", worldFrame, registry);
      rightAcceptedFootstepSurfaceNormal = new YoFrameVector("rightAcceptedFootstepSurfaceNormal", worldFrame, registry);
      acceptedFootstepSurfaceNormals = new SideDependentList<>(leftAcceptedFootstepSurfaceNormal, rightAcceptedFootstepSurfaceNormal);

      nodeRejectedReason = new EnumYoVariable<>("nodeRejectedReason", registry, BipedalFootstepPlannerNodeRejectionReason.class, true);
      nodeRejectedReason.set(null);

      parentRegistry.addChild(registry);
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
   }

   @Override
   public void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode acceptedNode)
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
   }

   private void moveUpSlightlyToEnsureVisible(YoGraphicPolygon footstepToExpandViz)
   {
      FramePoint framePointToPack = new FramePoint(worldFrame);
      footstepToExpandViz.getPosition(framePointToPack);
      framePointToPack.setZ(framePointToPack.getZ() + 0.0025);
      footstepToExpandViz.setPosition(framePointToPack);
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
