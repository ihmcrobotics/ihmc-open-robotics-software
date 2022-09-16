package us.ihmc.gdx.ui.yo;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.baselinePlanner.BaselineFootstepPlannerParameters;
import us.ihmc.footstepPlanning.baselinePlanner.JoystickFootstepPlanner;
import us.ihmc.footstepPlanning.baselinePlanner.SimpleTimedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class GDXJoystickStepper
{
   private static final double dt = 0.05;

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
      private final JoystickFootstepPlanner planner;

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

         planner = new JoystickFootstepPlanner(plannerParameters, 5.0, dt, 10, footPolygons, registry, graphicsListRegistry);


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

   DRCRobotModel robotModel;
   ROS2SyncedRobotModel syncedRobot;
   ROS2ControllerHelper controllerHelper;
   CommunicationHelper communicationHelper;
   private final SimpleTimedFootstep completedFootstep = new SimpleTimedFootstep();
   private boolean activated = false;
   Controller controller;
   FootstepPlannerParametersBasics footstepPlannerParameters;

   public GDXJoystickStepper(DRCRobotModel robotModel,
                             ROS2SyncedRobotModel syncedRobot,
                             ROS2ControllerHelper controllerHelper,
                             CommunicationHelper communicationHelper,
                             FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.controllerHelper = controllerHelper;
      this.communicationHelper = communicationHelper;
      this.footstepPlannerParameters = footstepPlannerParameters;
   }

   public void initiate()
   {
      if (!activated)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

         // Initialize sole frames.
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

         RigidBodyTransform leftFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame();
         RigidBodyTransform rightFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();

         Vector3DBasics initialLeftPosition = leftFootTransform.getTranslation();
         Vector3DBasics initialRightPosition = rightFootTransform.getTranslation();
         RotationMatrixBasics initialLeftRotation = leftFootTransform.getRotation();
         RotationMatrixBasics initialRightRotation = leftFootTransform.getRotation();

         SideDependentList<FramePose3D> initialFootholds = new SideDependentList<>();
         initialFootholds.put(RobotSide.LEFT, new FramePose3D(worldFrame, new Point3D(initialLeftPosition), new Quaternion(initialLeftRotation)));
         initialFootholds.put(RobotSide.RIGHT, new FramePose3D(worldFrame, new Point3D(initialRightPosition), new Quaternion(initialRightRotation)));


         Joystick joystick;

         BaselineFootstepPlannerParameters plannerParameters = new BaselineFootstepPlannerParameters(footstepPlannerParameters);

         try
         {
            joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
            joystick.setStandalone();
            joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1));
            joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1));
            joystick.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1));

            controller = new Controller(joystick, PlannerTools.createDefaultFootPolygons(), initialFootholds, plannerParameters, new DoubleProvider()
            {
               @Override
               public double getValue()
               {
                  return Conversions.nanosecondsToSeconds(System.nanoTime());
               }
            }, dt, graphicsListRegistry);
         }
         catch (IOException e)
         {
            throw new RuntimeException("error opening joystick: " + e);
         }
         controller.initialize();
         activated = true;
      }

   }

   public void stop()
   {
      activated = false;
   }

   public FramePose3DReadOnly getCurrentPose()
   {
      return (FramePose3DReadOnly) controller.planner.getPreviewTrajectory().getPose();
   }

   public void run()
   {
      if (activated)
      {
         controller.doControl();
         FootstepDataListMessage footstepDataListMessage = controller.planner.getPlannedFootsteps();

         int j = footstepDataListMessage.getFootstepDataList().size();
         if (j>0)
            controllerHelper.publishToController(footstepDataListMessage);

         for (int i = 0; i < j;  ++i)
         {
            controller.planner.removePublishedFootSteps();
         }
      }
   }
}
