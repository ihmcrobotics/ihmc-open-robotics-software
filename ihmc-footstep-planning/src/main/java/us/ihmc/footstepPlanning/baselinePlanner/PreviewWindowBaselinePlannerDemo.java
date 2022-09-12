package us.ihmc.footstepPlanning.baselinePlanner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.interfaces.PoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PreviewWindowBaselinePlannerDemo
{
   private class Controller implements RobotController
   {
      private final int TRAJECTORY_WINDOW_SIZE = 100;
      private final int VIZ_SAMPLE_COUNT = 20;
      private final double MAX_X_VELOCITY = 1.0;
      private final double MAX_Y_VELOCITY = 1.0;
      private final double MAX_YAW_VELOCITY = Math.PI;

      private final YoRegistry registry = new YoRegistry("visualization");
      private final DoubleProvider timeProvider;
      private final PreviewWindowPoseTrajectoryGenerator trajectory;
      private final TimedFootstepPlanVisualization timedFootstepPlanViz;
      private final YoFramePoseUsingYawPitchRoll baselineVizPose;

      public Controller(SideDependentList<ConvexPolygon2D> footPolygons, List<SimpleTimedFootstep> plannedSteps,
                        int numberOfValidSteps, DoubleProvider timeProvider, YoGraphicsListRegistry graphicsListRegistry)
      {
         this.timeProvider = timeProvider;

         this.trajectory = new PreviewWindowPoseTrajectoryGenerator(ReferenceFrame.getWorldFrame(), TRAJECTORY_WINDOW_SIZE, dt);
         this.trajectory.reset(new FramePose3D(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 0.0), new FrameQuaternion()));
         this.dt = dt;
         this.viz = new SamplingPoseTrajectoryVisualizer("snake", trajectory, ReferenceFrame.getWorldFrame(), VIZ_SAMPLE_COUNT, 0.01, false, registry, graphicsListRegistry);

         joystick.addJoystickEventListener(event ->
                                           {
                                              if (event.getComponent().getIdentifier() == XBoxOneMapping.LEFT_STICK_Y.getIdentifier())
                                                 xdot = MAX_X_VELOCITY * event.getValue();
                                              else if (event.getComponent().getIdentifier() == XBoxOneMapping.LEFT_STICK_X.getIdentifier())
                                                 ydot = MAX_Y_VELOCITY * event.getValue();
                                              else if (event.getComponent().getIdentifier() == XBoxOneMapping.RIGHT_STICK_X.getIdentifier())
                                                 yawdot = MAX_YAW_VELOCITY * event.getValue();
                                           });


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
         trajectory.compute(time);
         baselineVizPose.set(trajectory.getPose());
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

   public PreviewWindowBaselinePlannerDemo(BaselineFootstepPlannerParameters plannerParameters)
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

      Robot robot;
      Joystick joystick

      try
      {
         Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
         joystick.setStandalone();
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1));

         robot = new Robot("robot");
         robot.setController(new PreviewWindowPoseTrajectoryGeneratorDemo(joystick, graphicsListRegistry, DT));
         robot.setDynamic(false);
      }
      catch (IOException e)
      {
         throw new RuntimeException("error opening joystick: " + e);
      }



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
      robot.setController(new Controller(PlannerTools.createDefaultFootPolygons(),
                                         joystick,
                                         plannedSteps,
                                         numSteps,
                                         scs::getTime,
                                         graphicsListRegistry));

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
      new PreviewWindowBaselinePlannerDemo(new BaselineFootstepPlannerParameters());
   }
}