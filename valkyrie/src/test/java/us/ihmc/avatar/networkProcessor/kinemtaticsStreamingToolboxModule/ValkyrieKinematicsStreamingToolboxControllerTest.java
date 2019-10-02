package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.createFullRobotModelAtInitialConfiguration;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.RelativeEndEffectorControlTest.circlePositionAt;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleFunction;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidable;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollisionResult;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePose3D;

public class ValkyrieKinematicsStreamingToolboxControllerTest extends KinematicsStreamingToolboxControllerTest
{
   private DRCRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel newRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   private ScheduledExecutorService executor;

   @Test
   public void testStreamingToController() throws SimulationExceededMaximumTimeException
   {
      YoVariableRegistry spyRegistry = new YoVariableRegistry("spy");
      YoDouble handPositionMeanError = new YoDouble("HandsPositionMeanError", spyRegistry);
      YoDouble handOrientationMeanError = new YoDouble("HandsOrientationMeanError", spyRegistry);

      setupWithWalkingController(new RobotController()
      {
         private final SideDependentList<YoFramePose3D> handDesiredPoses = new SideDependentList<>(side -> new YoFramePose3D(side.getCamelCaseName()
               + "HandDesired", worldFrame, spyRegistry));
         private final SideDependentList<YoFramePose3D> handCurrentPoses = new SideDependentList<>(side -> new YoFramePose3D(side.getCamelCaseName()
               + "HandCurrent", worldFrame, spyRegistry));
         private final SideDependentList<YoDouble> handPositionErrors = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseName()
               + "HandPositionError", spyRegistry));
         private final SideDependentList<YoDouble> handOrientationErrors = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseName()
               + "HandOrientationError", spyRegistry));
         private YoDouble time;
         private YoBoolean isStreaming;
         private YoDouble streamingStartTime;
         private YoDouble streamingBlendingDuration;
         private YoDouble mainStateMachineSwitchTime;
         private YoEnum<KSTState> mainStateMachineCurrentState;

         private boolean needsToInitialize = true;

         @SuppressWarnings("unchecked")
         @Override
         public void initialize()
         {
            if (!needsToInitialize)
               return;

            time = (YoDouble) toolboxRegistry.getVariable("time");
            isStreaming = (YoBoolean) toolboxRegistry.getVariable("isStreaming");
            streamingStartTime = (YoDouble) toolboxRegistry.getVariable("streamingStartTime");
            streamingBlendingDuration = (YoDouble) toolboxRegistry.getVariable("streamingBlendingDuration");
            mainStateMachineSwitchTime = (YoDouble) toolboxRegistry.getVariable("mainStateMachineSwitchTime");
            mainStateMachineCurrentState = (YoEnum<KSTState>) toolboxRegistry.getVariable("mainStateMachineCurrentState");

            needsToInitialize = false;
         }

         private final Mean positionMean = new Mean();
         private final Mean orientationMean = new Mean();

         @Override
         public void doControl()
         {
            initialize();

            if (mainStateMachineCurrentState.getEnumValue() != KSTState.STREAMING || !isStreaming.getValue())
            {
               handDesiredPoses.values().forEach(YoFramePose3D::setToNaN);
               handCurrentPoses.values().forEach(YoFramePose3D::setToNaN);
               return;
            }

            double timeInStream = time.getValue() - mainStateMachineSwitchTime.getValue() - streamingStartTime.getValue();

            if (timeInStream < streamingBlendingDuration.getValue() + 10.0 * toolboxControllerPeriod)
            {
               handDesiredPoses.values().forEach(YoFramePose3D::setToNaN);
               handCurrentPoses.values().forEach(YoFramePose3D::setToNaN);
               return;
            }

            for (RobotSide robotSide : RobotSide.values)
            {
               YoFramePose3D handDesiredPose = handDesiredPoses.get(robotSide);
               YoFramePose3D handCurrentPose = handCurrentPoses.get(robotSide);
               YoDouble handPositionError = handPositionErrors.get(robotSide);
               YoDouble handOrientationError = handOrientationErrors.get(robotSide);

               handDesiredPose.setFromReferenceFrame(desiredFullRobotModel.getHandControlFrame(robotSide));
               handCurrentPose.setFromReferenceFrame(toolboxController.getTools().getCurrentFullRobotModel().getHandControlFrame(robotSide));
               handPositionError.set(handDesiredPose.getPositionDistance(handCurrentPose));
               handOrientationError.set(handDesiredPose.getOrientationDistance(handCurrentPose));
               positionMean.increment(handPositionError.getValue());
               orientationMean.increment(handOrientationError.getValue());
            }

            handPositionMeanError.set(positionMean.getResult());
            handOrientationMeanError.set(orientationMean.getResult());
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return spyRegistry;
         }
      });

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      wakeupToolbox();

      ScheduledFuture<?> scheduleMessageGenerator = scheduleMessageGenerator(0.01, circleMessageGenerator(true, 0.125));

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
      assertTrue(success);

      scheduleMessageGenerator.cancel(true);

      KinematicsStreamingToolboxInputMessage message = new KinematicsStreamingToolboxInputMessage();
      message.setStreamToController(false);
      inputPublisher.publish(message);

      sleepToolbox();

      executor.shutdownNow();

      // Asserts that the spy did run and that the toolbox or something did not just hang
      assertNotEquals(0.0, handPositionMeanError.getValue());
      assertNotEquals(0.0, handOrientationMeanError.getValue());
      // TODO Pretty bad assertions here, need to figure out how to improve this test later.
      System.out.println("Position error avg: " + handPositionMeanError.getValue() + ", orientation error avg: " + handOrientationMeanError.getValue());
      assertTrue(handPositionMeanError.getValue() < 0.15, "Mean position error is: " + handPositionMeanError.getValue());
      assertTrue(handOrientationMeanError.getValue() < 0.20, "Mean orientation error is: " + handOrientationMeanError.getValue());
   }

   private void wakeupToolbox()
   {
      ToolboxStateMessage wakeupMessage = new ToolboxStateMessage();
      wakeupMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      statePublisher.publish(wakeupMessage);
   }

   private void sleepToolbox()
   {
      ToolboxStateMessage wakeupMessage = new ToolboxStateMessage();
      wakeupMessage.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      statePublisher.publish(wakeupMessage);
   }

   private ScheduledFuture<?> scheduleMessageGenerator(double dt, DoubleFunction<KinematicsStreamingToolboxInputMessage> messageGenerator)
   {
      if (executor == null)
         executor = ThreadTools.newSingleDaemonThreadScheduledExecutor("inputs-generator");

      return executor.scheduleAtFixedRate(new Runnable()
      {
         double time = 0.0;

         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;
            inputPublisher.publish(messageGenerator.apply(time));
            time += dt;
         }
      }, 0, (int) (dt * 1000), TimeUnit.MILLISECONDS);
   }

   private DoubleFunction<KinematicsStreamingToolboxInputMessage> circleMessageGenerator(boolean streamToController, double frequency)
   {
      FullHumanoidRobotModel fullRobotModel = valkyrieRobotModel.createFullRobotModel();
      double circleRadius = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.3, side.negateIfRightSide(0.225), 0.9));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ? new Vector3D(0.0, 0.0, 0.0)
            : new Vector3D());

      return new DoubleFunction<KinematicsStreamingToolboxInputMessage>()
      {
         @Override
         public KinematicsStreamingToolboxInputMessage apply(double time)
         {
            KinematicsStreamingToolboxInputMessage input = new KinematicsStreamingToolboxInputMessage();
            input.setStreamToController(streamToController);

            for (RobotSide robotSide : RobotSide.values)
            {
               FramePoint3D position = circlePositionAt(time,
                                                        robotSide.negateIfRightSide(frequency),
                                                        circleRadius,
                                                        circleCenters.get(robotSide),
                                                        circleCenterVelocities.get(robotSide));
               KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(fullRobotModel.getHand(robotSide), position);
               input.getInputs().add().set(message);
            }
            return input;
         }
      };
   }

   @Test
   public void testHandMotionWithCollision()
   {
      DRCRobotModel robotModel = newRobotModel();
      setupNoWalkingController(robotModel.getHumanoidRobotKinematicsCollisionModel());
      FullHumanoidRobotModel fullRobotModelAtInitialConfiguration = createFullRobotModelAtInitialConfiguration(robotModel);
      toolboxController.updateRobotConfigurationData(extractRobotConfigurationData(fullRobotModelAtInitialConfiguration));
      toolboxController.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(fullRobotModelAtInitialConfiguration, robotModel, true, true));

      List<KinematicsCollidable> collidables = robotModel.getHumanoidRobotKinematicsCollisionModel().getRobotCollidables(desiredFullRobotModel);

      assertTrue(toolboxController.initialize());
      snapSCSRobotToFullRobotModel(toolboxController.getDesiredFullRobotModel(), robot);
      if (visualize)
         scs.tickAndUpdate();

      double circleRadius = 0.25;
      double circleFrequency = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.2, side.negateIfRightSide(0.225), 0.9));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ? new Vector3D(0.0, 0.0, 0.0)
            : new Vector3D());

      double toolboxControllerPeriod = toolboxController.getTools().getToolboxControllerPeriod();

      for (double t = 0.0; t < 10.0; t += toolboxControllerPeriod)
      {
         KinematicsStreamingToolboxInputMessage input = new KinematicsStreamingToolboxInputMessage();
         input.getInputs().add().set(KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(fullRobotModelAtInitialConfiguration.getPelvis()));

         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint3D position = circlePositionAt(t,
                                                     robotSide.negateIfRightSide(circleFrequency),
                                                     circleRadius,
                                                     circleCenters.get(robotSide),
                                                     circleCenterVelocities.get(robotSide));
            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(desiredFullRobotModel.getHand(robotSide),
                                                                                                             position);
            input.getInputs().add().set(message);
         }

         commandInputManager.submitMessage(input);
         toolboxController.update();
         snapSCSRobotToFullRobotModel(desiredFullRobotModel, robot);
         if (visualize)
         {
            Arrays.asList(scs.getRobots()).forEach(robot -> robot.getYoTime().add(toolboxControllerPeriod));
            scs.tickAndUpdate();
         }

         for (int collidable1Index = 0; collidable1Index < collidables.size(); collidable1Index++)
         {
            KinematicsCollidable collidable1 = collidables.get(collidable1Index);

            for (int collidable2Index = 0; collidable2Index < collidables.size(); collidable2Index++)
            {
               KinematicsCollidable collidable2 = collidables.get(collidable2Index);

               if (collidable1.isCollidableWith(collidable2))
               {
                  KinematicsCollisionResult collision = collidable1.evaluateCollision(collidable2);
                  assertTrue(collision.getSignedDistance() > -1.0e-3);
               }
            }
         }
      }
   }
}
