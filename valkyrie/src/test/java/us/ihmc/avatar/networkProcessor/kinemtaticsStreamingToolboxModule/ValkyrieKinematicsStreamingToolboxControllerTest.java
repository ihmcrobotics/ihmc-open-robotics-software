package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

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

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidable;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollisionResult;
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
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

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
      simulationTestingParameters.setKeepSCSUp(true);
      setupWithWalkingController();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      wakeupToolbox();

      ScheduledFuture<?> scheduleMessageGenerator = scheduleMessageGenerator(0.1, circleMessageGenerator(true));

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
      assertTrue(success);

      scheduleMessageGenerator.cancel(true);

      KinematicsStreamingToolboxInputMessage message = new KinematicsStreamingToolboxInputMessage();
      message.setStreamToController(false);
      inputPublisher.publish(message);

      sleepToolbox();

      executor.shutdownNow();
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

   private DoubleFunction<KinematicsStreamingToolboxInputMessage> circleMessageGenerator(boolean streamToController)
   {
      FullHumanoidRobotModel fullRobotModel = valkyrieRobotModel.createFullRobotModel();
      double circleRadius = 0.25;
      double circleFrequency = 0.25;
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
                                                        robotSide.negateIfRightSide(circleFrequency),
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
