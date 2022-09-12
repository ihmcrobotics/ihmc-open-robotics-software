package us.ihmc.footstepPlanning.baselinePlanner;
import java.awt.Color;
import java.io.IOException;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DSpotLight;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.PlaybackListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class PreviewWindowPoseTrajectoryGeneratorDemo
{
   private class SnakeGameController implements RobotController
   {
      private final int TRAJECTORY_WINDOW_SIZE = 100;
      private final int VIZ_SAMPLE_COUNT = 20;
      private final double MAX_X_VELOCITY = 1.0;
      private final double MAX_Y_VELOCITY = 1.0;
      private final double MAX_YAW_VELOCITY = Math.PI;

      private final YoRegistry registry = new YoRegistry(SnakeGameController.class.getSimpleName());
      private final PreviewWindowPoseTrajectoryGenerator trajectory;
      private final SamplingPoseTrajectoryVisualizer viz;
      private final double dt;

      private final FramePose3D currentPose = new FramePose3D();
      private final YoDouble q_x = new YoDouble("q_x", registry);
      private final YoDouble q_y = new YoDouble("q_y", registry);
      private final YoDouble q_z = new YoDouble("q_z", registry);

      private double xdot;
      private double ydot;
      private double yawdot;

      public SnakeGameController(Joystick joystick, YoGraphicsListRegistry graphicsListRegistry, double dt)
      {
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
         return "robot";
      }

      @Override
      public String getDescription()
      {
         return "";
      }

      @Override
      public void doControl()
      {
         trajectory.append(xdot, ydot, yawdot);
         trajectory.compute(TRAJECTORY_WINDOW_SIZE * dt);
         currentPose.set(trajectory.getPose());
         q_x.set(currentPose.getX());
         q_y.set(currentPose.getY());
         q_z.set(currentPose.getZ());
         viz.redraw(0, (TRAJECTORY_WINDOW_SIZE - 1) * dt);
      }
   }

   private final SimulationConstructionSet scs;

   public PreviewWindowPoseTrajectoryGeneratorDemo()
   {
      final double DT = 0.01;

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      Robot robot;

      try
      {
         Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
         joystick.setStandalone();
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1));
         joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1));

         robot = new Robot("robot");
         robot.setController(new SnakeGameController(joystick, graphicsListRegistry, DT));
         robot.setDynamic(false);
      }
      catch (IOException e)
      {
         throw new RuntimeException("error opening joystick: " + e);
      }

      this.scs = new SimulationConstructionSet(robot);
      scs.setDT(DT, 1);

      // Build the ground model
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("ground");
      YoAppearanceTexture texture = new YoAppearanceTexture("textures/grid.png");

      RigidBodyTransform location = new RigidBodyTransform();
      location.getTranslation().set(new Vector3D(0, 0, -0.5));

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, 25, 25, 1), texture);
      combinedTerrainObject.addTerrainObject(newBox);

      scs.addStaticLinkGraphics(combinedTerrainObject.getLinkGraphics());
      scs.setGroundVisible(false); // don't show default ground

      // Add a skylight
      scs.clearDirectionalLights();
      scs.setAmbientLight(new Color(100, 100, 100, 100));

      String skybox_resource = "skybox/dark_skybox.png";
      scs.setupSky(skybox_resource, skybox_resource, skybox_resource, skybox_resource, skybox_resource, skybox_resource);

      Graphics3DSpotLight lightAbove = new Graphics3DSpotLight();
      lightAbove.setPosition(new Point3D(0, 0, 2));
      lightAbove.setDirection(new Vector3D(0, 0, -1));
      lightAbove.setColor(new Color(210, 210, 255, 255));
      lightAbove.setSpotInnerAngle(Math.PI / (4.0 * 2.0));
      lightAbove.setSpotOuterAngle(Math.PI / (4.0 * 1.0));
      YoVariable q_x = scs.findVariable("q_x");
      YoVariable q_y = scs.findVariable("q_y");
      scs.addSpotLight(lightAbove);
      scs.attachPlaybackListener(new PlaybackListener()
      {
         @Override
         public void stop()
         {
         }

         @Override
         public void play(double realTimeRate)
         {
         }

         @Override
         public void indexChanged(int newIndex)
         {
            lightAbove.setPosition(new Point3D(q_x.getValueAsDouble(), q_y.getValueAsDouble(), 2.0));
         }
      });

      // Set camera pose
      scs.setCameraPosition(1.5, 1.5, 0.5);
      scs.setCameraFix(0.0, 0.0, 0.2);
      scs.setCameraTracking(true, true, true, false);

      scs.setSimulateNoFasterThanRealTime(true);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
   }

   public void start()
   {
      scs.startOnAThread();
      scs.simulate();
   }

   public static void main(String[] args)
   {
      new PreviewWindowPoseTrajectoryGeneratorDemo().start();
   }
}
