package us.ihmc.footstepPlanning.testTools;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanHolder;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PlanningTestTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final AppearanceDefinition[] appearances = {YoAppearance.LightGray(), YoAppearance.Grey(), YoAppearance.DarkGray()};

   private static final double footLength = 0.2;
   private static final double footWidth = 0.1;



   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose3D goalPose, BodyPathPlanHolder bodyPath)
   {
      visualizeAndSleep(planarRegionsList, footseps, goalPose, bodyPath, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose3D goalPose)
   {
      visualizeAndSleep(planarRegionsList, footseps, goalPose, null, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps)
   {
      visualizeAndSleep(planarRegionsList, footseps, null, null, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose3D goalPose, BodyPathPlanHolder bodyPath,
                                        YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      visualizeAndSleep(planarRegionsList, footseps, goalPose, bodyPath, PlannerTools.createDefaultFootPolygon(), registry, graphicsListRegistry);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footsteps, FramePose3D goalPose, BodyPathPlanHolder bodyPath,
                                        ConvexPolygon2D defaultFoothold, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      if (registry != null)
         scs.addYoRegistry(registry);
      if (graphicsListRegistry != null)
         scs.addYoGraphicsListRegistry(graphicsListRegistry, true);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
//      graphics3DObject.addCoordinateSystem(0.3);
      if (planarRegionsList != null)
      {
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, planarRegionsList, appearances);
         scs.setGroundVisible(false);
      }
      scs.addStaticLinkGraphics(graphics3DObject);
      scs.setCameraPosition(-4.0, -4.0, 6.0);
      scs.setCameraFix(0.5, 0.0, 0.1);

      YoRegistry vizRegistry = new YoRegistry("FootstepPlanningResult");
      YoGraphicsListRegistry vizGraphicsListRegistry = new YoGraphicsListRegistry();

      if (goalPose != null)
         PlannerTools.addGoalViz(goalPose, vizRegistry, vizGraphicsListRegistry);

      if (bodyPath != null)
      {
         int markers = 100;
         for (int i = 0; i < markers; i++)
         {
            double alpha = (double) i / (double) (markers - 1);
            Pose3D pose = new Pose3D();
            bodyPath.getPointAlongPath(alpha, pose);
            YoFramePoint3D yoPoint = new YoFramePoint3D("BodyPathPoint" + i, worldFrame, vizRegistry);
            yoPoint.set(pose.getPosition());
            YoGraphicPosition pointVis = new YoGraphicPosition("BodyPathPoint" + i, yoPoint, 0.025, YoAppearance.Blue());
            vizGraphicsListRegistry.registerYoGraphic("viz", pointVis);
         }
      }

      if (footsteps != null)
      {
         YoFrameConvexPolygon2D yoDefaultFootPolygon = new YoFrameConvexPolygon2D("DefaultFootPolygon", worldFrame, 4, vizRegistry);
         yoDefaultFootPolygon.set(defaultFoothold);

         int numberOfSteps = footsteps.getNumberOfSteps();

         for (int i = 0; i < numberOfSteps; i++)
         {
            PlannedFootstep footstep = footsteps.getFootstep(i);
            FramePose3D footstepPose = new FramePose3D();
            footstep.getFootstepPose(footstepPose);

            AppearanceDefinition appearance = footstep.getRobotSide() == RobotSide.RIGHT ? YoAppearance.Green() : YoAppearance.Red();
            YoFramePoseUsingYawPitchRoll yoFootstepPose = new YoFramePoseUsingYawPitchRoll("footPose" + i, worldFrame, vizRegistry);
            yoFootstepPose.set(footstepPose);
            yoFootstepPose.setZ(yoFootstepPose.getZ() + (footstep.getRobotSide() == RobotSide.RIGHT ? 0.001 : 0.0));

            if (!footstep.hasFoothold())
            {
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + i, yoDefaultFootPolygon, yoFootstepPose, 1.0, appearance);
               vizGraphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
            else
            {
               YoGraphicPolygon fullFootstepViz = new YoGraphicPolygon("fullFootstep" + i, yoDefaultFootPolygon, yoFootstepPose, 1.0, YoAppearance.Glass(0.7));
               vizGraphicsListRegistry.registerYoGraphic("viz", fullFootstepViz);

               ConvexPolygon2D foothold = new ConvexPolygon2D(footstep.getFoothold());
               foothold.update();

               ConvexPolygonTools.limitVerticesConservative(foothold, 4);
               YoFrameConvexPolygon2D yoFoothold = new YoFrameConvexPolygon2D("Foothold" + i, worldFrame, 4, vizRegistry);
               yoFoothold.set(foothold);
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + i, yoFoothold, yoFootstepPose, 1.0, appearance);
               vizGraphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
         }
      }

      scs.addYoRegistry(vizRegistry);
      scs.addYoGraphicsListRegistry(vizGraphicsListRegistry, true);
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }
}
