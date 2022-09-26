package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.baselinePlanner.BaselineFootstepPlannerParameters;
import us.ihmc.footstepPlanning.baselinePlanner.ContinuousTrackingFootstepPlanner;
import us.ihmc.footstepPlanning.baselinePlanner.SimpleTimedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.visualizers.GDXSphereAndArrowGraphic;
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

public class GDXPoseTracking
{
   private class Controller implements us.ihmc.scs2.definition.controller.interfaces.Controller
   {
      // Compute timed footstep plan.
      int maxSteps = 30;

      private final YoRegistry registry = new YoRegistry("visualization");
      private final DoubleProvider timeProvider;
      private final ContinuousTrackingFootstepPlanner planner;

      private final List<SimpleTimedFootstep> plannedSteps = new ArrayList<>();

      private final AtomicDouble xdot = new AtomicDouble();
      private final AtomicDouble ydot = new AtomicDouble();
      private final AtomicDouble yawdot = new AtomicDouble();

      private final SideDependentList<FramePose3D> previousFootholds = new SideDependentList<>(new FramePose3D(), new FramePose3D());

      public Controller(Joystick joystick,
                        SideDependentList<FramePose3D> initialFootholds,
                        BaselineFootstepPlannerParameters plannerParameters,
                        DoubleProvider timeProvider,
                        double dt)
      {
         this.timeProvider = timeProvider;

         for (int i = 0; i < maxSteps; i++)
         {
            plannedSteps.add(new SimpleTimedFootstep());
         }
         for (RobotSide robotSide : RobotSide.values)
         {
            previousFootholds.get(robotSide).set(initialFootholds.get(robotSide));
         }

         planner = new ContinuousTrackingFootstepPlanner(plannerParameters, 5.0, dt, 10, MAX_X_VELOCITY, MAX_Y_VELOCITY, MAX_YAW_VELOCITY);

         if (joystick!=null)
         {
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
      }

      @Override
      public void doControl()
      {
         double time = timeProvider.getValue();
         planner.update(time, xdot.get(), ydot.get(), yawdot.get());
      }

      public void doControl(double deltaTime)
      {
         double time = timeProvider.getValue();
         // Update baseline graphics.
         planner.update(time, deltaTime, xdot.get(), ydot.get(), yawdot.get());
      }

      public void doControl(double deltaTime, FramePose3D framePose3D)
      {
         double time = timeProvider.getValue();
         planner.update(time, deltaTime, framePose3D);
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

      public MODE getTestMode()
      {
         return mode;
      }
   }

   // NOTE: GDXJoystickStepper stuff starts here
   private static final double dt = 0.05;
   private static final double MAX_X_VELOCITY = 0.3;  // m/s
   private static final double MAX_Y_VELOCITY = 0.15;  // m/s
   private static final double MAX_YAW_VELOCITY = Math.PI / 6.0;  // rad/s
   private GDXImGuiBasedUI baseUI;

   private final GDXSphereAndArrowGraphic sphereAndArrowGraphic;
   private enum MODE
   {
      joystick, other
   };
   MODE mode = MODE.joystick;

   DRCRobotModel robotModel;
   ROS2SyncedRobotModel syncedRobot;
   ROS2ControllerHelper controllerHelper;
   CommunicationHelper communicationHelper;
   private boolean activated = false;
   Controller controller;
   FootstepPlannerParametersBasics footstepPlannerParameters;
   private FramePose3D goalPoseWithZOffset = new FramePose3D();

   public GDXPoseTracking(GDXImGuiBasedUI baseUI,
                          DRCRobotModel robotModel,
                          ROS2SyncedRobotModel syncedRobot,
                          ROS2ControllerHelper controllerHelper,
                          CommunicationHelper communicationHelper,
                          FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      this.baseUI = baseUI;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.controllerHelper = controllerHelper;
      this.communicationHelper = communicationHelper;
      this.footstepPlannerParameters = footstepPlannerParameters;
      sphereAndArrowGraphic = new GDXSphereAndArrowGraphic();
      sphereAndArrowGraphic.create(0.05f, 0.4f, Color.PURPLE);
   }

   public void initiate()
   {
      if (!activated)
      {
         // Initialize sole frames.
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

         RigidBodyTransform leftFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame();
         RigidBodyTransform rightFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();

         // INITIAL
         Vector3DBasics initialLeftPosition = leftFootTransform.getTranslation();         // LEFT FOOT X,Y,Z
         Vector3DBasics initialRightPosition = rightFootTransform.getTranslation();       // RIGHT FOOT X,Y,Z
         RotationMatrixBasics initialLeftRotation = leftFootTransform.getRotation();      // LEFT FOOT ORIENTATION
         RotationMatrixBasics initialRightRotation = rightFootTransform.getRotation();    // RIGHT FOOT ORIENTATION

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

            controller = new Controller(joystick, initialFootholds, plannerParameters, () -> Conversions.nanosecondsToSeconds(System.nanoTime()), dt);
         }
         catch (IOException e)
         {
            controller = new Controller(null, initialFootholds, plannerParameters, () -> Conversions.nanosecondsToSeconds(System.nanoTime()), dt);
//            throw new RuntimeException("error opening joystick: " + e);
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

   public void run(FramePose3D framePose3D)
   {
      if (activated)
      {
         double deltaTime = Gdx.graphics.getDeltaTime();
         if (controller.getTestMode() == MODE.joystick)
            controller.doControl(deltaTime);
         else
         {
            controller.doControl(deltaTime, framePose3D);
         }
         FootstepDataListMessage footstepDataListMessage = controller.planner.getPlannedFootsteps();
         goalPoseWithZOffset.set(getCurrentPose());
         goalPoseWithZOffset.appendTranslation(0.0f,0.0f,0.1f);
         sphereAndArrowGraphic.setToPose(goalPoseWithZOffset);
         int j = footstepDataListMessage.getFootstepDataList().size();
         if (j>0)
            controllerHelper.publishToController(footstepDataListMessage);

         for (int i = 0; i < j;  ++i)
         {
            controller.planner.removePublishedFootSteps();
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      sphereAndArrowGraphic.getRenderables(renderables, pool);
   }

   public boolean isJoystickMode()
   {
      return this.mode == MODE.joystick;
   }

   public MODE getMode()
   {
      return mode;
   }

   public void setMode(boolean incoming)
   {
      if(incoming && mode == MODE.other)
      {
         mode = MODE.joystick;
         stop();
         initiate();
      }
      else if(!incoming && mode == MODE.joystick)
      {
         mode = MODE.other;
         stop();
         initiate();
      }
   }
}
