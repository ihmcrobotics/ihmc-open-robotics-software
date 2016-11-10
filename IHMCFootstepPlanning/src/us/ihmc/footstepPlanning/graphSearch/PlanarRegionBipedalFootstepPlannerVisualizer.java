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
   private final YoFrameConvexPolygon2d leftFootstepToExpand, rightFootstepToExpand;
   private final YoFrameConvexPolygon2d leftAcceptedFootstep, rightAcceptedFootstep;
   private final YoFrameConvexPolygon2d leftRejectedFootstep, rightRejectedFootstep;

   private final YoGraphicPolygon leftFootstepToExpandViz, rightFootstepToExpandViz;
   private final YoGraphicPolygon leftAcceptedFootstepViz, rightAcceptedFootstepViz;
   private final YoGraphicPolygon leftRejectedFootstepViz, rightRejectedFootstepViz;

   private final SideDependentList<YoGraphicPolygon> footstepsToExpandViz, acceptedFootstepsViz, rejectedFootstepsViz;

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public PlanarRegionBipedalFootstepPlannerVisualizer(SideDependentList<ConvexPolygon2d> feetPolygonsInSoleFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      ConvexPolygon2d leftFootInSoleFrame = feetPolygonsInSoleFrame.get(RobotSide.LEFT);
      ConvexPolygon2d rightFootInSoleFrame = feetPolygonsInSoleFrame.get(RobotSide.RIGHT);

      int maxNumberOfVertices = leftFootInSoleFrame.getNumberOfVertices();

      leftFootstepToExpand = new YoFrameConvexPolygon2d("leftFootstepToExpand", worldFrame, maxNumberOfVertices, registry);
      rightFootstepToExpand = new YoFrameConvexPolygon2d("rightFootstepToExpand", worldFrame, maxNumberOfVertices, registry);
      leftAcceptedFootstep = new YoFrameConvexPolygon2d("leftAcceptedFootstep", worldFrame, maxNumberOfVertices, registry);
      rightAcceptedFootstep = new YoFrameConvexPolygon2d("rightAcceptedFootstep", worldFrame, maxNumberOfVertices, registry);
      leftRejectedFootstep = new YoFrameConvexPolygon2d("leftRejectedFootstep", worldFrame, maxNumberOfVertices, registry);
      rightRejectedFootstep = new YoFrameConvexPolygon2d("rightRejectedFootstep", worldFrame, maxNumberOfVertices, registry);

      leftFootstepToExpand.setConvexPolygon2d(leftFootInSoleFrame);
      rightFootstepToExpand.setConvexPolygon2d(rightFootInSoleFrame);
      leftAcceptedFootstep.setConvexPolygon2d(leftFootInSoleFrame);
      rightAcceptedFootstep.setConvexPolygon2d(rightFootInSoleFrame);
      leftRejectedFootstep.setConvexPolygon2d(leftFootInSoleFrame);
      rightRejectedFootstep.setConvexPolygon2d(rightFootInSoleFrame);

      leftFootstepToExpandViz = new YoGraphicPolygon("leftFootstepToExpandViz", leftFootstepToExpand, "leftFootstepToExpandPose", "", registry, 1.0, YoAppearance.Yellow());
      rightFootstepToExpandViz = new YoGraphicPolygon("rightFootstepToExpandViz", rightFootstepToExpand, "rightFootstepToExpandPose", "", registry, 1.0, YoAppearance.Yellow());
      leftAcceptedFootstepViz = new YoGraphicPolygon("leftAcceptedFootstepViz", leftAcceptedFootstep, "leftAcceptedFootstepPose", "", registry, 1.0, YoAppearance.Green());
      rightAcceptedFootstepViz = new YoGraphicPolygon("rightAcceptedFootstepViz", rightAcceptedFootstep, "rightAcceptedFootstepPose", "", registry, 1.0, YoAppearance.Green());
      leftRejectedFootstepViz = new YoGraphicPolygon("leftRejectedFootstepViz", leftRejectedFootstep, "leftRejectedFootstepPose", "", registry, 1.0, YoAppearance.Red());
      rightRejectedFootstepViz = new YoGraphicPolygon("rightRejectedFootstepViz", rightRejectedFootstep, "rightRejectedFootstepPose", "", registry, 1.0, YoAppearance.Red());

      leftFootstepToExpandViz.setPoseToNaN();
      rightFootstepToExpandViz.setPoseToNaN();
      leftAcceptedFootstepViz.setPoseToNaN();
      rightAcceptedFootstepViz.setPoseToNaN();
      leftRejectedFootstepViz.setPoseToNaN();
      rightRejectedFootstepViz.setPoseToNaN();

      footstepsToExpandViz = new SideDependentList<>(leftFootstepToExpandViz, rightFootstepToExpandViz);
      acceptedFootstepsViz = new SideDependentList<>(leftAcceptedFootstepViz, rightAcceptedFootstepViz);
      rejectedFootstepsViz = new SideDependentList<>(leftRejectedFootstepViz, rightRejectedFootstepViz);

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
