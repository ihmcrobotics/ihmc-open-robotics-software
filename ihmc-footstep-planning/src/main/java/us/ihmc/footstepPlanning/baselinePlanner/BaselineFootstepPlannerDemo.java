package us.ihmc.footstepPlanning.baselinePlanner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsMinimumJerkTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.PoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class BaselineFootstepPlannerDemo
{
   private class Controller implements RobotController
   {
      private final YoRegistry registry = new YoRegistry("visualization");
      private final DoubleProvider timeProvider;
      private final PoseTrajectoryGenerator baselinePoseTrajectory;
      private final TimedFootstepPlanVisualization timedFootstepPlanViz;
      private final YoFramePoseUsingYawPitchRoll baselineVizPose;

      public Controller(SideDependentList<ConvexPolygon2D> footPolygons, PoseTrajectoryGenerator baselinePoseTrajectory, List<SimpleTimedFootstep> plannedSteps,
                        int numberOfValidSteps, DoubleProvider timeProvider, YoGraphicsListRegistry graphicsListRegistry)
      {
         this.timeProvider = timeProvider;
         this.baselinePoseTrajectory = baselinePoseTrajectory;

         this.timedFootstepPlanViz = new TimedFootstepPlanVisualization(plannedSteps, numberOfValidSteps, footPolygons, registry, graphicsListRegistry);

         // Create baseline pose graphics.
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         baselineVizPose = new YoFramePoseUsingYawPitchRoll("baselinePose", worldFrame, registry);
         graphicsListRegistry.registerYoGraphic("baseline", new YoGraphicCoordinateSystem("baselineViz", baselineVizPose, 0.2));
         baselineVizPose.setZ(-1000);
      }

      @Override
      public void doControl()
      {
         double time = timeProvider.getValue();

         // Update footstep graphics.
         timedFootstepPlanViz.update(time);

         // Update baseline graphics.
         baselinePoseTrajectory.compute(time);
         baselineVizPose.set(baselinePoseTrajectory.getPose());
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return getClass().getSimpleName();
      }

      @Override
      public String getDescription()
      {
         return getClass().getSimpleName();
      }
   }

   public BaselineFootstepPlannerDemo(BaselineFootstepPlannerParameters plannerParameters)
   {
      YoRegistry registry = new YoRegistry("visualizer");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      SimulationConstructionSet scs = new SimulationConstructionSet();

      BaselineFootstepPlanner planner = new BaselineFootstepPlanner(plannerParameters);

      // Initialize sole frames.
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SideDependentList<FramePose3D> initialFootholds = new SideDependentList<>();
      initialFootholds.put(RobotSide.LEFT, new FramePose3D(worldFrame, new Point3D(0, 0.08, 0), new Quaternion()));
      initialFootholds.put(RobotSide.RIGHT, new FramePose3D(worldFrame, new Point3D(0, -0.08, 0), new Quaternion()));

      // Initialize baseline pose trajectory.
      double initialTime = 0.0;
      double finalTime = 20.0;
      double duration = finalTime - initialTime;
      MultipleWaypointsMinimumJerkTrajectoryGenerator xInputTrajectory = new MultipleWaypointsMinimumJerkTrajectoryGenerator("xInputTrajectory", 7, registry);
      MultipleWaypointsMinimumJerkTrajectoryGenerator yInputTrajectory = new MultipleWaypointsMinimumJerkTrajectoryGenerator("yInputTrajectory", 7, registry);
      MultipleWaypointsMinimumJerkTrajectoryGenerator aInputTrajectory = new MultipleWaypointsMinimumJerkTrajectoryGenerator("aInputTrajectory", 7, registry);
      xInputTrajectory.appendWaypoint(initialTime, 0, 0);
      xInputTrajectory.appendWaypoint(initialTime + 0.2 * duration, 0, 0.0);
      xInputTrajectory.appendWaypoint(initialTime + 0.4 * duration, 0.5, 0.2);
      xInputTrajectory.appendWaypoint(initialTime + 0.6 * duration, 1.0, 0.2);
      xInputTrajectory.appendWaypoint(initialTime + 0.7 * duration, 1.25, 0);
      xInputTrajectory.appendWaypoint(finalTime, 1.25, 0);
      xInputTrajectory.initialize();
      yInputTrajectory.appendWaypoint(initialTime, 0, 0);
      yInputTrajectory.appendWaypoint(initialTime + 0.2 * duration, 0.25, 0);
      yInputTrajectory.appendWaypoint(finalTime, 0.25, 0);
      yInputTrajectory.initialize();
      aInputTrajectory.appendWaypoint(initialTime, 0, 0);
      aInputTrajectory.appendWaypoint(initialTime + 0.2 * duration, 0, 0);
      aInputTrajectory.appendWaypoint(initialTime + 0.4 * duration, 1, 0);
      aInputTrajectory.appendWaypoint(initialTime + 0.7 * duration, -1, 0);
      aInputTrajectory.appendWaypoint(initialTime + 0.9 * duration, 0, 0);
      aInputTrajectory.appendWaypoint(finalTime, 0, 0);
      aInputTrajectory.initialize();

      double baselineDT = 0.01;
      int windowSize = (int) (finalTime / baselineDT);
      PreviewWindowPoseTrajectoryGenerator baselinePoseTrajectory = new PreviewWindowPoseTrajectoryGenerator(worldFrame, windowSize, baselineDT);
      baselinePoseTrajectory.reset(new FramePose3D());
      for (int i = 0; i < windowSize; i++)
      {
         double time = initialTime + (finalTime - initialTime) * i / windowSize;
         xInputTrajectory.compute(time);
         yInputTrajectory.compute(time);
         aInputTrajectory.compute(time);
         baselinePoseTrajectory.append(xInputTrajectory.getVelocity(), yInputTrajectory.getVelocity(), aInputTrajectory.getVelocity());
      }
      baselinePoseTrajectory.initialize();

      // Compute timed footstep plan.
      int maxSteps = 30;
      List<SimpleTimedFootstep> plannedSteps = new ArrayList<>();
      for (int i = 0; i < maxSteps; i++)
      {
         plannedSteps.add(new SimpleTimedFootstep());
      }
      RobotSide initialStep = RobotSide.LEFT;
      int numSteps = planner.compute(plannedSteps, baselinePoseTrajectory, initialFootholds, initialStep, initialTime, finalTime);

      // Create visualizer controller.
      Robot robot = new Robot("robot");
      robot.setController(new Controller(PlannerTools.createDefaultFootPolygons(), baselinePoseTrajectory, plannedSteps, numSteps, scs::getTime, graphicsListRegistry));

      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.addRobot(robot);
      scs.startOnAThread();
      scs.setRecordDT(0.005);
      scs.simulate(finalTime);

      // Draw pose trajectory.
      int nPoints = 40;
      for (int i = 0; i < nPoints; i++)
      {
         double time = initialTime + (finalTime - initialTime) * i / nPoints;
         baselinePoseTrajectory.compute(time);
         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(baselinePoseTrajectory.getPosition());
         sphere.addSphere(0.01, YoAppearance.Black());
         scs.addStaticLinkGraphics(sphere);
      }
      baselinePoseTrajectory.compute(finalTime);
      Graphics3DObject sphere = new Graphics3DObject();
      sphere.translate(baselinePoseTrajectory.getPosition());
      sphere.addSphere(0.01, YoAppearance.Red());
      scs.addStaticLinkGraphics(sphere);
   }

   static public void main(String args[])
   {
      new BaselineFootstepPlannerDemo(new BaselineFootstepPlannerParameters());
   }
}
