package us.ihmc.footstepPlanning.occlusion;

import java.awt.Color;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleOcclusionTests
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int maxSteps = 100;
   private static final int rays = 1000;
   private static final int maxPolygonsToVisualize = 10;
   private static final int maxPolygonsVertices = 50;
   private static final int stepsPerSideToVisualize = 4;
   private static final double defaultMaxAllowedSolveTime = 1.0;

   @Rule
   public TestName name = new TestName();

   @Test(timeout = 300000)
   @ContinuousIntegrationTest(estimatedDuration = 10.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   public void testSimpleOcclusions()
   {
      FramePose startPose = new FramePose();
      FramePose goalPose = new FramePose();
      PlanarRegionsList regions = createSimpleOcclusionField(startPose, goalPose);
      runTest(startPose, goalPose, regions, defaultMaxAllowedSolveTime);
   }

   @Test(timeout = 300000)
   @Ignore // Resource file does not seem to exist.
   public void testOcclusionsFromData()
   {
      FramePose startPose = new FramePose(worldFrame);
      startPose.setPosition(0.25, -0.25, 0.0);

      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(2.75, 0.95, 0.0);
      BestEffortPlannerParameters parameters = new BestEffortPlannerParameters();

      Path path = Paths.get(getClass().getClassLoader().getResource("PlanarRegions_20171114_090937").getPath());
      PlanarRegionsList regions = PlanarRegionFileTools.importPlanarRegionData(path.toFile());

      runTest(startPose, goalPose, regions, parameters, 2.0);
   }

   private class BestEffortPlannerParameters extends DefaultFootstepPlanningParameters
   {
      @Override
      public boolean getReturnBestEffortPlan()
      {
         return true;
      }

      @Override
      public int getMinimumStepsForBestEffortPlan()
      {
         return 3;
      }
   }

   @Test(timeout = 300000)
   @Ignore
   public void testMazeWithOcclusions()
   {
      FramePose startPose = new FramePose();
      FramePose goalPose = new FramePose();
      PlanarRegionsList regions = createMazeOcclusionField(startPose, goalPose);
      runTest(startPose, goalPose, regions, defaultMaxAllowedSolveTime);
   }

   private void runTest(FramePose startPose, FramePose goalPose, PlanarRegionsList regions, double maxAllowedSolveTime)
   {
      runTest(startPose, goalPose, regions, getParameters(), maxAllowedSolveTime);
   }

   private void runTest(FramePose startPose, FramePose goalPose, PlanarRegionsList regions, FootstepPlannerParameters parameters, double maxAllowedSolveTime)
   {
      YoVariableRegistry registry = new YoVariableRegistry(name.getMethodName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      FootstepPlanner planner = getPlanner(parameters, graphicsListRegistry, registry);
      FootstepPlannerGoal goal = createPlannerGoal(goalPose);

      FramePose stancePose = new FramePose();
      RobotSide stanceSide = computeStanceFootPose(startPose, parameters, stancePose);

      SimulationConstructionSet scs = null;
      SideDependentList<List<YoFramePose>> solePosesForVisualization = null;
      List<YoFramePose> stepPosesTaken = null;
      YoFramePose startStep = null;

      YoFramePoint observerPoint = null;
      List<YoFramePoint> rayIntersectionVisualizations = null;
      List<YoFrameConvexPolygon2d> visiblePolygons = null;
      List<YoFramePose> visiblePolygonPoses = null;
      List<YoGraphicPolygon> polygonVisualizations = null;

      YoBoolean plannerFailed = new YoBoolean("PlannerFailed", registry);
      YoDouble solveTime = new YoDouble("SolveTime", registry);

      if (visualize)
      {
         YoFrameConvexPolygon2d defaultPolygon = new YoFrameConvexPolygon2d("DefaultFootPolygon", worldFrame, 4, registry);
         defaultPolygon.setConvexPolygon2d(PlanningTestTools.createDefaultFootPolygon());

         solePosesForVisualization = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
         for (int i = 0; i < stepsPerSideToVisualize; i++)
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
         for (int i = 0; i < maxPolygonsToVisualize; i++)
         {
            YoFrameConvexPolygon2d polygon = new YoFrameConvexPolygon2d("Polygon" + i, worldFrame, maxPolygonsVertices, registry);
            YoFramePose pose = new YoFramePose("PolygonPose" + i, worldFrame, registry);
            pose.setToNaN();
            visiblePolygons.add(polygon);
            visiblePolygonPoses.add(pose);
            YoGraphicPolygon visualization = new YoGraphicPolygon("Polygon" + i, polygon, pose.getPosition(), pose.getOrientation(), 1.0, 0.02,
                                                                  new YoAppearanceRGBColor(Color.BLUE, 0.8));
            polygonVisualizations.add(visualization);
            graphicsListRegistry.registerYoGraphic("viz", visualization);
            graphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(visualization);
         }

         rayIntersectionVisualizations = new ArrayList<>();
         for (int i = 0; i < rays; i++)
         {
            YoFramePoint point = new YoFramePoint("RayIntersection" + i, ReferenceFrame.getWorldFrame(), registry);
            point.setToNaN();
            YoGraphicPosition visualization = new YoGraphicPosition("RayIntersection" + i, point, 0.02, YoAppearance.Blue());
            rayIntersectionVisualizations.add(point);
            graphicsListRegistry.registerYoGraphic("viz", visualization);
         }

         observerPoint = new YoFramePoint("Observer", worldFrame, registry);
         observerPoint.setToNaN();
         YoGraphicPosition observerVisualization = new YoGraphicPosition("Observer", observerPoint, 0.05, YoAppearance.Red());
         graphicsListRegistry.registerYoGraphic("viz", observerVisualization);

         scs = setupSCS(name.getMethodName(), registry, regions, startPose, goalPose);
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.setInPoint();
      }

      FootstepPlan plan = null;
      int failCount = 0;
      double maxSolveTime = 0.0;
      boolean reachedGoal = false;

      // Add the ground plane here so the visibility graph works. Remove that later.
      PlanarRegionsList visiblePlanarRegions = new PlanarRegionsList(regions.getPlanarRegion(0));
//      PlanarRegionsList visiblePlanarRegions = new PlanarRegionsList();

      for (int i = 0; i < maxSteps; i++)
      {
         Point3D observer = computeBodyPoint(stancePose, stanceSide, parameters, 0.8);
         visiblePlanarRegions = createVisibleRegions(regions, observer, visiblePlanarRegions, rayIntersectionVisualizations);

         if (visualize)
         {
            observerPoint.set(PlanarRegionTools.projectPointToPlanesVertically(observer, regions));

            for (int polygonIdx = 0; polygonIdx < maxPolygonsToVisualize; polygonIdx++)
            {
               visiblePolygonPoses.get(polygonIdx).setToNaN();
            }
            int polygons = Math.min(maxPolygonsToVisualize, visiblePlanarRegions.getNumberOfPlanarRegions());
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            FramePose pose = new FramePose();
            for (int polygonIdx = 0; polygonIdx < polygons; polygonIdx++)
            {
               PlanarRegion planarRegion = visiblePlanarRegions.getPlanarRegion(polygonIdx);
               if (planarRegion.getConvexHull().getNumberOfVertices() > visiblePolygons.get(polygonIdx).getMaxNumberOfVertices())
               {
                  throw new RuntimeException("Increase max number of vertices for visualization.");
               }
               planarRegion.getTransformToWorld(transformToWorld);
               pose.setPose(transformToWorld);
               visiblePolygonPoses.get(polygonIdx).set(pose);
               visiblePolygons.get(polygonIdx).setConvexPolygon2d(planarRegion.getConvexHull());
            }
         }

         planner.setPlanarRegions(visiblePlanarRegions);
         planner.setInitialStanceFoot(stancePose, stanceSide);
         planner.setGoal(goal);
         planner.setTimeout(maxAllowedSolveTime + 5.0);

         boolean haveNewPlan = false;
         try
         {
            long startTime = System.currentTimeMillis();
            FootstepPlanningResult result = planner.plan();
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            solveTime.set(seconds);

            if (seconds > maxSolveTime)
            {
               maxSolveTime = seconds;
            }

            if (result.validForExecution())
            {
               haveNewPlan = true;
               plan = planner.getPlan();
            }
            else
            {
               PrintTools.info("Planner failed: " + result);
            }
         }
         catch (Exception e)
         {
            // The catch needs to be removed once the visibility graph is improved.
            PrintTools.info("Planner threw exception:");
            e.printStackTrace();
         }

         if (plan == null)
         {
            if (visualize)
            {
               scs.setTime(i);
               scs.tickAndUpdate();
            }
            PrintTools.info("Failed");
            break;
         }

         plannerFailed.set(!haveNewPlan);
         if (plannerFailed.getBooleanValue())
         {
            plan.remove(0);
            failCount++;
         }

         if (plan.getNumberOfSteps() < 1)
         {
            if (visualize)
            {
               scs.setTime(i);
               scs.tickAndUpdate();
            }
            PrintTools.info("Failed");
            break;
         }

         if (visualize)
         {
            for (int hideIdx = 0; hideIdx < stepsPerSideToVisualize; hideIdx++)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  solePosesForVisualization.get(robotSide).get(hideIdx).setToNaN();
               }
            }

            startStep.set(stancePose);
            int stepsToShow = Math.min(plan.getNumberOfSteps(), 2 * stepsPerSideToVisualize);
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
         Assert.assertTrue("Planner took too long: " + maxSolveTime + "s.", maxSolveTime < maxAllowedSolveTime);
         Assert.assertTrue("Did not reach goal.", reachedGoal);

         // Add that after the visibility graph is fixed to deal with start points in no-go-zones.
         // Assert.assertTrue("Planner failed at least once.", failCount == 0);
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

   private PlanarRegionsList createVisibleRegions(PlanarRegionsList regions, Point3D observer, PlanarRegionsList knownRegions,
                                                  List<YoFramePoint> rayPointsToPack)
   {
      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(observer, 1.0, rays);
      List<ConvexPolygon2D> visiblePolygons = new ArrayList<>();
      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         visiblePolygons.add(new ConvexPolygon2D());
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      for (int rayIndex = 0; rayIndex < rays; rayIndex++)
      {
         Point3D pointOnSphere = pointsOnSphere[rayIndex];
         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnSphere, observer);
         Point3D intersection = PlanarRegionTools.intersectRegionsWithRay(regions, observer, rayDirection);
         if (intersection == null)
         {
            if (rayPointsToPack != null)
            {
               rayPointsToPack.get(rayIndex).setToNaN();
            }
            continue;
         }

         if (rayPointsToPack != null)
         {
            rayPointsToPack.get(rayIndex).set(intersection);
         }
         for (int regionIdx = 0; regionIdx < regions.getNumberOfPlanarRegions(); regionIdx++)
         {
            PlanarRegion region = regions.getPlanarRegion(regionIdx);
            if (PlanarRegionTools.isPointOnRegion(region, intersection, 0.01))
            {
               region.getTransformToWorld(transform);
               Point3D pointOnPlane = new Point3D(intersection);
               pointOnPlane.applyInverseTransform(transform);

               Point2D newVertex = new Point2D();
               newVertex.set(pointOnPlane);

               visiblePolygons.get(regionIdx).addVertex(newVertex);
            }
         }
      }

      PlanarRegionsList visible = new PlanarRegionsList();
      for (int i = 0; i < visiblePolygons.size(); i++)
      {
         ConvexPolygon2D polygon = visiblePolygons.get(i);
         polygon.update();
         if (polygon.getNumberOfVertices() < 2)
         {
            continue;
         }

         PlanarRegion originalRegion = regions.getPlanarRegion(i);
         originalRegion.getTransformToWorld(transform);
         PlanarRegion newRegion = new PlanarRegion(transform, polygon);
         visible.addPlanarRegion(newRegion);
      }

      return combine(knownRegions, visible);
   }

   private PlanarRegionsList combine(PlanarRegionsList regionsA, PlanarRegionsList regionsB)
   {
      PlanarRegionsList ret = new PlanarRegionsList();

      boolean[] added = new boolean[regionsB.getNumberOfPlanarRegions()];
      for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
      {
         added[regionBIdx] = false;
      }

      for (PlanarRegion regionA : regionsA.getPlanarRegionsAsList())
      {
         RigidBodyTransform transformA = new RigidBodyTransform();
         regionA.getTransformToWorld(transformA);
         boolean foundMatchingRegion = false;

         for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
         {
            PlanarRegion regionB = regionsB.getPlanarRegion(regionBIdx);
            RigidBodyTransform transformB = new RigidBodyTransform();
            regionB.getTransformToWorld(transformB);
            if (transformA.epsilonEquals(transformB, 0.01))
            {
               ConvexPolygon2D newHull = new ConvexPolygon2D(regionA.getConvexHull(), regionB.getConvexHull());
               ret.addPlanarRegion(new PlanarRegion(transformA, newHull));
               foundMatchingRegion = true;
               added[regionBIdx] = true;
            }
         }

         if (!foundMatchingRegion)
         {
            ret.addPlanarRegion(new PlanarRegion(transformA, new ConvexPolygon2D(regionA.getConvexHull())));
         }
      }

      for (int regionBIdx = 0; regionBIdx < regionsB.getNumberOfPlanarRegions(); regionBIdx++)
      {
         if (!added[regionBIdx])
         {
            ret.addPlanarRegion(regionsB.getPlanarRegion(regionBIdx));
         }
      }

      return ret;
   }

   private FootstepPlanner getPlanner(FootstepPlannerParameters parameters, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      SideDependentList<ConvexPolygon2D> footPloygons = PlanningTestTools.createDefaultFootPolygons();
      return new VisibilityGraphWithAStarPlanner(parameters, footPloygons, graphicsListRegistry, registry);
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

   private PlanarRegionsList createMazeOcclusionField(FramePose startPoseToPack, FramePose goalPoseToPack)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.addRectangle(6.0, 12.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(-1.0, -2.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 8.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(1.0, 0.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.Y);
      generator.addRectangle(1.0, 8.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(0.0, -4.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.X);
      generator.addRectangle(2.0, 1.0);

      generator.identity();
      generator.rotate(Math.toRadians(10.0), Axis.X);
      generator.translate(0.0, 4.0, 0.5);
      generator.rotate(-Math.PI / 2.0, Axis.X);
      generator.addRectangle(2.0, 1.0);

      startPoseToPack.setToZero(ReferenceFrame.getWorldFrame());
      startPoseToPack.setYawPitchRoll(Math.PI / 2.0, 0.0, 0.0);
      startPoseToPack.setPosition(-2.0, -5.0, 0.0);
      startPoseToPack.prependRollRotation(Math.toRadians(10.0));

      goalPoseToPack.setToZero(ReferenceFrame.getWorldFrame());
      goalPoseToPack.setYawPitchRoll(-Math.PI / 2.0, 0.0, 0.0);
      goalPoseToPack.setPosition(0.0, -5.0, 0.0);
      goalPoseToPack.prependRollRotation(Math.toRadians(10.0));

      return generator.getPlanarRegionsList();
   }
}
