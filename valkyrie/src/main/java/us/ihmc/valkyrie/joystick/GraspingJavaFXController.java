package us.ihmc.valkyrie.joystick;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.avatar.handControl.HandFingerTrajectoryMessagePublisher;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.humanoidRobotics.communication.packets.WholeBodyTrajectoryToolboxOutputConverter;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;

/**
 * What to do this.
 * Handler grasping object.
 * Communicate with WholeBodyTrajectoryToolbox.
 * Control fingers.
 * Visualize object and preview of motion.
 */

public class GraspingJavaFXController
{
   private final FullHumanoidRobotModel fullRobotModel;

   private final WholeBodyTrajectoryToolboxOutputConverter outputConverter;

   private final SideDependentList<AtomicReference<Boolean>> sendFingerMessages = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbRolls = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbPitchs = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbPitch2s = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredIndexes = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredMiddles = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredPinkys = new SideDependentList<>();

   private final AnimationTimer animationTimer;

   private final Group rootNode = new Group();

   private final static double timeDurationForFinger = 2.0;

   private final IHMCROS2Publisher<WholeBodyTrajectoryMessage> wholeBodyTrajectoryPublisher;
   private final HandFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher;

   public GraspingJavaFXController(String robotName, JavaFXMessager messager, Ros2Node ros2Node, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                   JavaFXRobotVisualizer javaFXRobotVisualizer, HandFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher)
   {
      fullRobotModel = javaFXRobotVisualizer.getFullRobotModel();
      outputConverter = new WholeBodyTrajectoryToolboxOutputConverter(fullRobotModelFactory);

      sendFingerMessages.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightSendMessage, false));
      sendFingerMessages.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftSendMessage, false));

      desiredThumbRolls.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightThumbRoll, 0.0));
      desiredThumbPitchs.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightThumb, 0.0));
      desiredThumbPitch2s.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightThumb2, 0.0));
      desiredIndexes.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightIndex, 0.0));
      desiredMiddles.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightMiddle, 0.0));
      desiredPinkys.put(RobotSide.RIGHT, messager.createInput(GraspingJavaFXTopics.RightPinky, 0.0));

      desiredThumbRolls.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftThumbRoll, 0.0));
      desiredThumbPitchs.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftThumb, 0.0));
      desiredThumbPitch2s.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftThumb2, 0.0));
      desiredIndexes.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftIndex, 0.0));
      desiredMiddles.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftMiddle, 0.0));
      desiredPinkys.put(RobotSide.LEFT, messager.createInput(GraspingJavaFXTopics.LeftPinky, 0.0));

      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      wholeBodyTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, WholeBodyTrajectoryMessage.class, subscriberTopicNameGenerator);
      this.handFingerTrajectoryMessagePublisher = handFingerTrajectoryMessagePublisher;

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long arg0)
         {
            submitDesiredFingerConfigurationMessage();
         }
      };
   }

   private void submitDesiredFingerConfigurationMessage()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (sendFingerMessages.get(robotSide).get())
         {
            sendFingerMessages.get(robotSide).set(false);

            TDoubleArrayList desiredPositions = new TDoubleArrayList();
            TDoubleArrayList trajectoryTimes = new TDoubleArrayList();

            desiredPositions.add(desiredThumbRolls.get(robotSide).get());
            desiredPositions.add(desiredThumbPitchs.get(robotSide).get());
            desiredPositions.add(desiredThumbPitch2s.get(robotSide).get());
            desiredPositions.add(desiredIndexes.get(robotSide).get());
            desiredPositions.add(desiredMiddles.get(robotSide).get());
            desiredPositions.add(desiredPinkys.get(robotSide).get());

            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);
            trajectoryTimes.add(timeDurationForFinger);

            handFingerTrajectoryMessagePublisher.sendFingerTrajectoryMessage(robotSide, desiredPositions, trajectoryTimes);
         }
      }
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}