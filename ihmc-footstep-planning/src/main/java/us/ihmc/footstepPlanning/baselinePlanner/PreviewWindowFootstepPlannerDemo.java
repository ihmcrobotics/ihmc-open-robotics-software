package us.ihmc.footstepPlanning.baselinePlanner;

import com.google.common.util.concurrent.AtomicDouble;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.tools.SCS1GraphicConversionTools;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PreviewWindowFootstepPlannerDemo
{
   private static final double dt = 0.01;

   private class Controller implements us.ihmc.scs2.definition.controller.interfaces.Controller
   {
      private final int TRAJECTORY_WINDOW_SIZE = 100;
      private final int VIZ_SAMPLE_COUNT = 20;
      private final double MAX_X_VELOCITY = 1.0;
      private final double MAX_Y_VELOCITY = 0.5;
      private final double MAX_YAW_VELOCITY = Math.PI / 2.0;

      // Compute timed footstep plan.
      int maxSteps = 30;

      private final YoRegistry registry = new YoRegistry("visualization");
      private final DoubleProvider timeProvider;
      private final PreviewWindowFootstepPlanner planner;

      private final List<SimpleTimedFootstep> plannedSteps = new ArrayList<>();

      private final AtomicDouble xdot = new AtomicDouble();
      private final AtomicDouble ydot = new AtomicDouble();
      private final AtomicDouble yawdot = new AtomicDouble();
      private final double dt;

      private final SideDependentList<FramePose3D> previousFootholds = new SideDependentList<>(new FramePose3D(), new FramePose3D());

      public Controller(Joystick joystick,
                        SideDependentList<ConvexPolygon2D> footPolygons,
                        SideDependentList<FramePose3D> initialFootholds,
                        BaselineFootstepPlannerParameters plannerParameters,
                        DoubleProvider timeProvider,
                        double dt,
                        YoGraphicsListRegistry graphicsListRegistry)
      {
         this.timeProvider = timeProvider;
         this.dt = dt;

         for (int i = 0; i < maxSteps; i++)
         {
            plannedSteps.add(new SimpleTimedFootstep());
         }
         for (RobotSide robotSide : RobotSide.values)
         {
            previousFootholds.get(robotSide).set(initialFootholds.get(robotSide));
         }

         planner = new PreviewWindowFootstepPlanner(plannerParameters, 5.0, dt, 5,  footPolygons, registry, graphicsListRegistry);


         joystick.addJoystickEventListener(event ->
                                           {
                                              if (event.getComponent().getIdentifier() == XBoxOneMapping.LEFT_STICK_Y.getIdentifier())
                                                 xdot.set(MAX_X_VELOCITY * event.getValue());
                                              else if (event.getComponent().getIdentifier() == XBoxOneMapping.LEFT_STICK_X.getIdentifier())
                                                 ydot.set(MAX_Y_VELOCITY * event.getValue());
                                              else if (event.getComponent().getIdentifier() == XBoxOneMapping.RIGHT_STICK_X.getIdentifier())
                                                 yawdot.set(MAX_YAW_VELOCITY * event.getValue());
                                           });

      }

      @Override
      public void doControl()
      {
         double time = timeProvider.getValue();
         // Update baseline graphics.
         planner.update(time, xdot.get(), ydot.get(), yawdot.get());
      }

      @Override
      public void initialize()
      {
         planner.initialize(previousFootholds);
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

   }

   public PreviewWindowFootstepPlannerDemo(BaselineFootstepPlannerParameters plannerParameters)
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.setDT(dt);

      // Initialize sole frames.
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SideDependentList<FramePose3D> initialFootholds = new SideDependentList<>();
      initialFootholds.put(RobotSide.LEFT, new FramePose3D(worldFrame, new Point3D(0, 0.08, 0), new Quaternion()));
      initialFootholds.put(RobotSide.RIGHT, new FramePose3D(worldFrame, new Point3D(0, -0.08, 0), new Quaternion()));

      Controller controller;
      Joystick joystick;

      try
      {
         joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
         joystick.setStandalone();
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1));

         controller = new Controller(joystick, PlannerTools.createDefaultFootPolygons(), initialFootholds, plannerParameters, scs.getTime(), dt, graphicsListRegistry);
         scs.addAfterPhysicsCallback((t) -> controller.doControl());
         scs.getRootRegistry().addChild(controller.getYoRegistry());

      }
      catch (IOException e)
      {
         throw new RuntimeException("error opening joystick: " + e);
      }

      // Create visualizer controller.

      scs.addYoGraphics(SCS1GraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));
      scs.setPlaybackRealTimeRate(1.0);
      scs.setRealTimeRateSimulation(true);
      scs.setBufferRecordTimePeriod(0.005);
      scs.start(true, false, false);

      controller.initialize();
      scs.simulate();
   }

   static public void main(String args[])
   {
      new PreviewWindowFootstepPlannerDemo(new BaselineFootstepPlannerParameters());
   }
}