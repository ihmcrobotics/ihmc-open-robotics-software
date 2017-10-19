package us.ihmc.footstepPlanning.occlusion;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.planners.VisibilityGraphWithAStarPlanner;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleOcclusionTests
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int maxSteps = 100;
   private static final int maxPolygons = 10;

   @Rule
   public TestName name = new TestName();

   @Test
   public void testSimpleOcclusions()
   {
      YoVariableRegistry registry = new YoVariableRegistry(name.getMethodName());

      FramePose startPose = new FramePose();
      FramePose goalPose = new FramePose();
      PlanarRegionsList regions = createSimpleOcclusionField(startPose, goalPose);

      FootstepPlannerParameters parameters = getParameters();
      FootstepPlanner planner = getPlanner(parameters, registry);

      FramePose stancePose = new FramePose();
      RobotSide stanceSide = computeStanceFootPose(startPose, parameters, stancePose);

      FootstepPlannerGoal goal = createPlannerGoal(goalPose);

      int stepsToShowPerSide = 4;
      SimulationConstructionSet scs = null;
      SideDependentList<List<YoFramePose>> solePosesForVisualization = null;
      List<YoFramePose> stepPosesTaken = null;
      YoFramePose startStep = null;

      List<YoFrameConvexPolygon2d> visiblePolygons = null;
      List<YoFramePose> visiblePolygonPoses = null;
      List<YoGraphicPolygon> polygonVisualizations = null;

      if (visualize)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
         YoFrameConvexPolygon2d defaultPolygon = new YoFrameConvexPolygon2d("DefaultFootPolygon", worldFrame, 4, registry);
         defaultPolygon.setConvexPolygon2d(PlanningTestTools.createDefaultFootPolygon());

         solePosesForVisualization = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
         for (int i = 0; i < stepsToShowPerSide; i++)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               AppearanceDefinition appearance = robotSide == RobotSide.RIGHT ? YoAppearance.Green() : YoAppearance.Red();
               String sideName = robotSide.getCamelCaseName();
               YoFramePose yoPose = new YoFramePose("footPose" + sideName + i, worldFrame, registry);
               yoPose.setToNaN();
               solePosesForVisualization.get(robotSide).add(yoPose);
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + sideName + i, defaultPolygon, yoPose, 1.0, appearance);
               graphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
         }

         startStep = new YoFramePose("startFootPose", worldFrame, registry);
         startStep.setToNaN();
         YoGraphicPolygon stanceViz = new YoGraphicPolygon("startFootPose", defaultPolygon, startStep, 1.0, YoAppearance.Black());
         graphicsListRegistry.registerYoGraphic("viz", stanceViz);

         stepPosesTaken = new ArrayList<>();
         for (int i = 0; i < maxSteps; i++)
         {
            YoFramePose step = new YoFramePose("step" + i, worldFrame, registry);
            step.setToNaN();
            stepPosesTaken.add(step);
            YoGraphicPolygon polygon = new YoGraphicPolygon("step" + i, defaultPolygon, step, 1.0, YoAppearance.Gray());
            graphicsListRegistry.registerYoGraphic("viz", polygon);
         }

         visiblePolygons = new ArrayList<>();
         visiblePolygonPoses = new ArrayList<>();
         polygonVisualizations = new ArrayList<>();
         for (int i = 0; i < maxPolygons; i++)
         {
            YoFrameConvexPolygon2d polygon = new YoFrameConvexPolygon2d("Polygon" + i, worldFrame, 10, registry);
            YoFramePose pose = new YoFramePose("PolygonPose" + i, worldFrame, registry);
            pose.setToNaN();
            visiblePolygons.add(polygon);
            visiblePolygonPoses.add(pose);
            YoGraphicPolygon visualization = new YoGraphicPolygon("Polygon" + i, polygon, pose.getPosition(), pose.getOrientation(), 1.0, 0.02,
                                                                  new YoAppearanceRGBColor(Color.BLUE, 0.8));
            polygonVisualizations.add(visualization);
            graphicsListRegistry.registerYoGraphic("viz", visualization);
         }

         scs = setupSCS(name.getMethodName(), registry, regions, startPose, goalPose);
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.setInPoint();
      }

      FootstepPlan plan = null;
      int failCount = 0;
      double maxSolveTime = 0.0;
      boolean reachedGoal = false;

      for (int i = 0; i < maxSteps; i++)
      {
         Point3D observer = computeBodyPoint(stancePose, stanceSide, parameters, 0.5);
         PlanarRegionsList visiblePlanarRegions = createVisibleRegions(regions, observer);

         if (visualize)
         {
            for (int polygonIdx = 0; polygonIdx < maxPolygons; polygonIdx++)
            {
               visiblePolygonPoses.get(polygonIdx).setToNaN();
            }
            int polygons = Math.min(maxPolygons, visiblePlanarRegions.getNumberOfPlanarRegions());
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            FramePose pose = new FramePose();
            for (int polygonIdx = 0; polygonIdx < polygons; polygonIdx++)
            {
               PlanarRegion planarRegion = visiblePlanarRegions.getPlanarRegion(polygonIdx);
               planarRegion.getTransformToWorld(transformToWorld);
               pose.setPose(transformToWorld);
               visiblePolygonPoses.get(polygonIdx).set(pose);
               visiblePolygons.get(polygonIdx).setConvexPolygon2d(planarRegion.getConvexHull());
            }
            for (int polygonIdx = 0; polygonIdx < maxPolygons; polygonIdx++)
            {
               polygonVisualizations.get(polygonIdx).update();
            }
         }

         planner.setPlanarRegions(visiblePlanarRegions);
         planner.setInitialStanceFoot(stancePose, stanceSide);
         planner.setGoal(goal);
         planner.setTimeout(1.0);

         try
         {
            long startTime = System.currentTimeMillis();
            FootstepPlanningResult result = planner.plan();
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;

            if (seconds > maxSolveTime)
            {
               maxSolveTime = seconds;
            }

            if (result.validForExecution())
            {
               plan = planner.getPlan();
            }
         }
         catch (Exception e)
         {
            plan.remove(0);
            failCount++;
         }

         if (visualize)
         {
            for (int hideIdx = 0; hideIdx < stepsToShowPerSide; hideIdx++)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  solePosesForVisualization.get(robotSide).get(hideIdx).setToNaN();
               }
            }

            startStep.set(stancePose);
            int stepsToShow = Math.min(plan.getNumberOfSteps(), 2 * stepsToShowPerSide);
            for (int stepIdx = 0; stepIdx < stepsToShow; stepIdx++)
            {
               SimpleFootstep footstep = plan.getFootstep(stepIdx);
               FramePose footstepPose = new FramePose();
               footstep.getSoleFramePose(footstepPose);

               List<YoFramePose> listOfPoses = solePosesForVisualization.get(footstep.getRobotSide());
               YoFramePose yoSolePose = listOfPoses.get(stepIdx / 2);
               yoSolePose.set(footstepPose);
            }

            scs.setTime(i);
            scs.tickAndUpdate();

            stepPosesTaken.get(i).set(stancePose);
         }

         SimpleFootstep firstStep = plan.getFootstep(0);
         firstStep.getSoleFramePose(stancePose);
         stanceSide = firstStep.getRobotSide();

         Point3D bodyPoint = computeBodyPoint(stancePose, stanceSide, parameters, 0.0);
         if (bodyPoint.epsilonEquals(goalPose.getPosition(), 0.1))
         {
            reachedGoal = true;
            break;
         }
      }

      PrintTools.info("Planner failed " + failCount + " times.");
      PrintTools.info("Maximum solve time was " + maxSolveTime + "s.");
      PrintTools.info("Reached goal: " + reachedGoal);

      if (visualize)
      {
         scs.setOutPoint();
         scs.cropBuffer();
         scs.setPlaybackRealTimeRate(0.001);
         scs.play();
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
      else
      {
         double maxAllowedSolveTime = 1.0;
         Assert.assertTrue("Planner took too long: " + maxSolveTime + "s.", maxSolveTime < maxAllowedSolveTime);
         Assert.assertTrue("Did not reach goal.", reachedGoal);

         // Add that after the visibility graph is fixed.
//         Assert.assertTrue("Planner failed at least once.", failCount == 0);
      }
   }

   private static SimulationConstructionSet setupSCS(String testName, YoVariableRegistry testRegistry, PlanarRegionsList regions, FramePose startPose,
                                                     FramePose goalPose)
   {
      Robot robot = new Robot(SimpleOcclusionTests.class.getSimpleName());
      robot.addYoVariableRegistry(testRegistry);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.8);
      if (regions != null)
      {
         graphics3DObject.addPlanarRegionsList(regions, YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray());
         scs.setGroundVisible(false);
      }

      RigidBodyTransform tempTransform = new RigidBodyTransform();

      graphics3DObject.identity();
      startPose.getRigidBodyTransform(tempTransform);
      graphics3DObject.transform(tempTransform);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.rotate(Math.PI / 2.0, new Vector3D(0.0, 1.0, 0.0));
      graphics3DObject.addArrow(0.8, YoAppearance.Green(), YoAppearance.Green());

      graphics3DObject.identity();
      goalPose.getRigidBodyTransform(tempTransform);
      graphics3DObject.transform(tempTransform);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.rotate(Math.PI / 2.0, new Vector3D(0.0, 1.0, 0.0));
      graphics3DObject.addArrow(0.8, YoAppearance.Red(), YoAppearance.Red());

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraPosition(-7.0, -1.0, 25.0);
      scs.setCameraFix(0.0, 0.0, 0.0);

      return scs;
   }

   private FootstepPlannerGoal createPlannerGoal(FramePose goalPose)
   {
      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      return goal;
   }

   private RobotSide computeStanceFootPose(FramePose startPose, FootstepPlannerParameters parameters, FramePose stancePoseToPack)
   {
      RobotSide side = RobotSide.LEFT;

      double stanceWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame bodyFrame = new PoseReferenceFrame("stanceFrame", startPose);
      FramePoint3D footPosition = new FramePoint3D(bodyFrame);
      footPosition.setY(side.negateIfRightSide(stanceWidth / 2.0));
      footPosition.changeFrame(ReferenceFrame.getWorldFrame());

      stancePoseToPack.setToZero(ReferenceFrame.getWorldFrame());
      stancePoseToPack.setPosition(footPosition);
      stancePoseToPack.setOrientation(startPose.getOrientation());

      return side;
   }

   private Point3D computeBodyPoint(FramePose solePose, RobotSide side, FootstepPlannerParameters parameters, double bodyHeight)
   {
      double stanceWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame soleFrame = new PoseReferenceFrame("stanceFrame", solePose);
      FramePoint3D bodyPosition = new FramePoint3D(soleFrame);
      bodyPosition.setY(side.negateIfLeftSide(stanceWidth / 2.0));
      bodyPosition.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D bodyPoint = new Point3D(bodyPosition);
      bodyPoint.addZ(bodyHeight);
      return bodyPoint;
   }

   private PlanarRegionsList createVisibleRegions(PlanarRegionsList regions, Point3D observer)
   {
      PlanarRegionsList ret = new PlanarRegionsList();
      ret.addPlanarRegion(regions.getPlanarRegion(0));
      ret.addPlanarRegion(regions.getPlanarRegion(1));

      if (observer.getX() > -1.0)
      {
         ret.addPlanarRegion(regions.getPlanarRegion(2));
      }

      return ret;
   }

   private FootstepPlanner getPlanner(FootstepPlannerParameters parameters, YoVariableRegistry registry)
   {
      SideDependentList<ConvexPolygon2D> footPloygons = PlanningTestTools.createDefaultFootPolygons();
      return new VisibilityGraphWithAStarPlanner(parameters, footPloygons, registry);
   }

   private FootstepPlannerParameters getParameters()
   {
      return new DefaultFootstepPlanningParameters();
   }

   private PlanarRegionsList createSimpleOcclusionField(FramePose startPoseToPack, FramePose goalPoseToPack)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.addRectangle(6.0, 6.0);
      generator.translate(-1.0, -1.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 4.0);
      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(1.0, 1.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 4.0);

      startPoseToPack.setToZero(ReferenceFrame.getWorldFrame());
      startPoseToPack.setYawPitchRoll(Math.PI / 2.0, 0.0, 0.0);
      startPoseToPack.setPosition(-2.0, -2.0, 0.0);
      startPoseToPack.prependRollRotation(Math.toRadians(10.0));

      goalPoseToPack.setToZero(ReferenceFrame.getWorldFrame());
      goalPoseToPack.setYawPitchRoll(Math.PI / 2.0, 0.0, 0.0);
      goalPoseToPack.setPosition(2.0, 2.0, 0.0);
      goalPoseToPack.prependRollRotation(Math.toRadians(10.0));

      return generator.getPlanarRegionsList();
   }
}
