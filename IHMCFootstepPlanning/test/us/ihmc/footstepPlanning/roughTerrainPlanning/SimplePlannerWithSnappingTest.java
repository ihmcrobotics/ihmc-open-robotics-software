package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPolygon;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public class SimplePlannerWithSnappingTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000000)
   public void testSimpleExample()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(1.0 + boxSize/2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, -0.001);
      generator.addRectangle(5.0, 5.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, createDefaultFootPolygon());
      PlanThenSnapPlanner planner = new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), footPolygons);

      FramePose initialStanceFootPose = new FramePose(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(1.0 + boxSize/2.0, 0.0, 0.0);

      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceSide);
      planner.setGoalPose(goalPose);
      planner.setPlanarRegions(planarRegionsList);

      ArrayList<FramePose> footstepPlan = new ArrayList<>();
      FootstepPlanningResult result = planner.plan(footstepPlan);

      if (!visualize)
         assertTrue("Planner was not able to provide valid result.", result.validForExecution());
      else
         visualizeAndSleep(planarRegionsList, footstepPlan, initialStanceSide.getOppositeSide());
   }

   private static ConvexPolygon2d createDefaultFootPolygon()
   {
      double footLength = 0.2;
      double footWidth = 0.1;

      ConvexPolygon2d footPolygon = new ConvexPolygon2d();
      footPolygon.addVertex(footLength/2.0, footWidth/2.0);
      footPolygon.addVertex(footLength/2.0, -footWidth/2.0);
      footPolygon.addVertex(-footLength/2.0, footWidth/2.0);
      footPolygon.addVertex(-footLength/2.0, -footWidth/2.0);
      footPolygon.update();

      return footPolygon;
   }

   private void visualizeAndSleep(PlanarRegionsList planarRegionsList, ArrayList<FramePose> footseps, RobotSide firstStepSide)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.3);
      graphics3DObject.addPlanarRegionsList(planarRegionsList, YoAppearance.Black());
      scs.addStaticLinkGraphics(graphics3DObject);

      int i = 0;
      RobotSide stepSide = firstStepSide;
      YoFrameConvexPolygon2d yoDefaultFootPolygon = new YoFrameConvexPolygon2d("DefaultFootPolygon", worldFrame, 4, registry);
      yoDefaultFootPolygon.setConvexPolygon2d(createDefaultFootPolygon());
      for (FramePose footstepPose : footseps)
      {
         AppearanceDefinition appearance = stepSide == RobotSide.RIGHT ? YoAppearance.Green() : YoAppearance.Red();
         YoFramePose yoFootstepPose = new YoFramePose("footPose" + (i++), worldFrame, registry);
         yoFootstepPose.set(footstepPose);

         YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + (i++), yoDefaultFootPolygon, yoFootstepPose, 1.0, appearance);
         graphicsListRegistry.registerYoGraphic("viz", footstepViz);
         stepSide = stepSide.getOppositeSide();
      }
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry, true);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

}
