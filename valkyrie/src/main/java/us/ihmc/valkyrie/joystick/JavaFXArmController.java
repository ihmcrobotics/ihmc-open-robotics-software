package us.ihmc.valkyrie.joystick;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import javafx.animation.AnimationTimer;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.avatar.handControl.HandFingerTrajectoryMessagePublisher;
import us.ihmc.avatar.joystickBasedJavaFXController.ButtonState;
import us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsPlanningToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXVisualizers.JavaFXRobotHandVisualizer;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;

public class JavaFXArmController
{
   private final FullHumanoidRobotModel fullRobotModel;

   private final KinematicsPlanningToolboxOutputConverter outputConverter;

   private final SideDependentList<AtomicReference<Boolean>> sendFingerMessages = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbRolls = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbPitchs = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredThumbPitch2s = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredIndexes = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredMiddles = new SideDependentList<>();
   private final SideDependentList<AtomicReference<Double>> desiredPinkys = new SideDependentList<>();

   private final AnimationTimer animationTimer;

   private final Group rootNode = new Group();

   private final static double timeDurationForFinger = 5.0;
   private final static double timeDurationForMotion = 10.0;
   private final static double ratioJoyStickToPosition = 0.02;
   private final static double ratioJoyStickToRotation = 0.04;
   private final static double lengthOfkeyFrameReferenceFrame = 0.15;
   private final static double defaultWeightForRigidBodyMessage = 20.0;

   private final HandFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<WholeBodyTrajectoryMessage> wholeBodyTrajectoryPublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   //private final IHMCROS2Publisher<KinematicsPlanningToolboxInputMessage> toolboxMessagePublisher;
   private final IHMCROS2Publisher<KinematicsToolboxRigidBodyMessage> toolboxMessagePublisher;

   private final AtomicReference<List<Node>> objectsToVisualizeReference = new AtomicReference<>(new ArrayList<>());
   private final RigidBodyTransform controlTransform = new RigidBodyTransform();
   private RobotSide controlSide = null;

   private final ReferenceFrame pelvisZUpFrame;

   private final DoubleProperty velocityXProperty = new SimpleDoubleProperty(this, "velocityXProperty", 0.0);
   private final DoubleProperty velocityYProperty = new SimpleDoubleProperty(this, "velocityYProperty", 0.0);
   private final DoubleProperty velocityZProperty = new SimpleDoubleProperty(this, "velocityZProperty", 0.0);
   private final DoubleProperty velocityRollProperty = new SimpleDoubleProperty(this, "velocityRollProperty", 0.0);
   private final DoubleProperty velocityPitchProperty = new SimpleDoubleProperty(this, "velocityPitchProperty", 0.0);
   private final DoubleProperty velocityYawProperty = new SimpleDoubleProperty(this, "velocityYawProperty", 0.0);

   private int indexOfSelectedKeyFrame = 0;
   private final SideDependentList<List<RigidBodyTransform>> sideDependentKeyFramePoses = new SideDependentList<>();
   private final SideDependentList<JavaFXRobotHandVisualizer> sideDependentHandVizs = new SideDependentList<>();

   //private final AtomicReference<KinematicsPlanningToolboxOutputStatus> toolboxOutputPacket = new AtomicReference<>(null);
   private final AtomicReference<KinematicsToolboxOutputStatus> toolboxOutputPacket = new AtomicReference<>(null);
   private final ValkyrieJavaFXInverseKinematicsVisualizer motionPreviewVisualizer;

   FullHumanoidRobotModelFactory fullRobotModelFactory;

   public JavaFXArmController(String robotName, JavaFXMessager messager, Ros2Node ros2Node, FullHumanoidRobotModelFactory fullRobotModelFactory,
                              JavaFXRobotVisualizer javaFXRobotVisualizer, HandFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher)
   {
      this.fullRobotModelFactory = fullRobotModelFactory;
      fullRobotModel = javaFXRobotVisualizer.getFullRobotModel();
      outputConverter = new KinematicsPlanningToolboxOutputConverter(fullRobotModelFactory);

      motionPreviewVisualizer = new ValkyrieJavaFXInverseKinematicsVisualizer(fullRobotModelFactory);
      motionPreviewVisualizer.enable(false);

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

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

      messager.registerTopicListener(XBoxOneJavaFXController.ButtonAState, state -> clearKeyFrame(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonSelectState, state -> createKeyFrame(state, RobotSide.LEFT));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonStartState, state -> createKeyFrame(state, RobotSide.RIGHT));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonBState, state -> switchSelectedObject(state));

      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftStickYAxis, this::appendingXAxis);
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftStickXAxis, this::appendingYAxis);
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonLeftBumperState, state -> appendingZAxisPositive(state));
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.LeftTriggerAxis, this::appendingZAxisNegative);

      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightStickYAxis, this::appendingPitch);
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightStickXAxis, this::appendingRoll);
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonRightBumperState, state -> appendingYawPositive(state));
      messager.registerJavaFXSyncedTopicListener(XBoxOneJavaFXController.RightTriggerAxis, this::appendingYawNegative);

      messager.registerTopicListener(XBoxOneJavaFXController.ButtonXState, state -> submitReachingManifoldsToToolbox(state));
      messager.registerTopicListener(XBoxOneJavaFXController.ButtonYState, state -> confirmReachingMotion(state));

      ROS2Tools.MessageTopicNameGenerator toolboxRequestTopicNameGenerator = KinematicsToolboxModule.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.MessageTopicNameGenerator toolboxResponseTopicNameGenerator = KinematicsToolboxModule.getPublisherTopicNameGenerator(robotName);

      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      wholeBodyTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, WholeBodyTrajectoryMessage.class, subscriberTopicNameGenerator);
      toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, toolboxRequestTopicNameGenerator);
      toolboxMessagePublisher = ROS2Tools.createPublisher(ros2Node, KinematicsToolboxRigidBodyMessage.class, toolboxRequestTopicNameGenerator);
      this.handFingerTrajectoryMessagePublisher = handFingerTrajectoryMessagePublisher;

      ROS2Tools.createCallbackSubscription(ros2Node, KinematicsToolboxOutputStatus.class, toolboxResponseTopicNameGenerator,
                                           s -> consumeToolboxOutputStatus(s.takeNextData()));

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long arg0)
         {
            updateSelectedKeyFrame();
            submitDesiredFingerConfigurationMessage();
            rootNode.getChildren().clear();
            rootNode.getChildren().add(motionPreviewVisualizer.getRootNode());
            updateVisualizedKeyFrames();
            updateControlHand();
         }
      };

      for (RobotSide robotSide : RobotSide.values)
      {
         sideDependentKeyFramePoses.put(robotSide, new ArrayList<>());
         sideDependentHandVizs.put(robotSide, new JavaFXRobotHandVisualizer(fullRobotModelFactory, robotSide));
      }

   }

   private void consumeToolboxOutputStatus(KinematicsToolboxOutputStatus packet)
   {
      toolboxOutputPacket.set(packet);
      
      if(toolboxOutputPacket.get() != null)
      {
         motionPreviewVisualizer.enable(true);
         motionPreviewVisualizer.submitKinematicsToolboxOutputStatus(toolboxOutputPacket.get());
      }
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

   private void submitReachingManifoldsToToolbox(ButtonState state)
   {
      if (state == ButtonState.RELEASED)
      {
         motionPreviewVisualizer.enable(false);
         toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

         RobotSide robotSide = RobotSide.RIGHT;
         double appendingYawOrientationValue = robotSide.negateIfLeftSide(Math.PI / 2);

         if (sideDependentKeyFramePoses.get(robotSide).size() == 0)
         {
            return;
         }
         else
         {
            Point3D desiredPosition = new Point3D(sideDependentKeyFramePoses.get(robotSide).get(0).getTranslationVector());
            Quaternion desiredOrientation = new Quaternion(sideDependentKeyFramePoses.get(robotSide).get(0).getRotationMatrix());
            desiredOrientation.appendYawRotation(appendingYawOrientationValue);

            LogTools.info("desiredPosition " + desiredPosition);
            LogTools.info("desiredOrientation " + desiredOrientation);

            KinematicsToolboxRigidBodyMessage handMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(fullRobotModel.getHand(robotSide), desiredPosition,
                                                                                                             desiredOrientation);

            handMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(defaultWeightForRigidBodyMessage));
            handMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(defaultWeightForRigidBodyMessage));

            handMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());
            
            toolboxMessagePublisher.publish(handMessage);
            
            KinematicsToolboxRigidBodyMessage chestMessage = KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(fullRobotModel.getChest());
            chestMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage((double) 20f));
            chestMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20f));
            chestMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());
            
            KinematicsToolboxRigidBodyMessage pelvisMessage = KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(fullRobotModel.getPelvis());
            pelvisMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage((double) 20f));
            pelvisMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20f));
            pelvisMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

            toolboxMessagePublisher.publish(chestMessage);
            toolboxMessagePublisher.publish(pelvisMessage);
         }
      }
   }

   private void confirmReachingMotion(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
      {
         if (toolboxOutputPacket.get().getSolutionQuality() > 0.0)
         {
            LogTools.info("confirmReachingMotion");

            motionPreviewVisualizer.enable(false);
            WholeBodyTrajectoryMessage message = new WholeBodyTrajectoryMessage();

            message.setDestination(PacketDestination.CONTROLLER.ordinal());
            outputConverter.setMessageToCreate(message);
            //outputConverter.computeWholeBodyTrajectoryMessage(toolboxOutputPacket.get());

            wholeBodyTrajectoryPublisher.publish(message);
         }
         else
         {
            LogTools.info("bad solution");
         }
      }
   }

   private void updateSelectedKeyFrame()
   {
      pelvisZUpFrame.update();
      controlTransform.appendTranslation(velocityXProperty.getValue(), velocityYProperty.getValue(), velocityZProperty.getValue());
      controlTransform.appendRollRotation(velocityRollProperty.getValue());
      controlTransform.appendPitchRotation(velocityPitchProperty.getValue());
      controlTransform.appendYawRotation(velocityYawProperty.getValue());
   }

   private void appendingRoll(double alpha)
   {
      velocityRollProperty.set(-alpha * ratioJoyStickToRotation);
   }

   private void appendingPitch(double alpha)
   {
      velocityPitchProperty.set(alpha * ratioJoyStickToRotation);
   }

   private void appendingYawNegative(double alpha)
   {
      velocityYawProperty.set(alpha * ratioJoyStickToRotation);
   }

   private void appendingYawPositive(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
         velocityYawProperty.set(ratioJoyStickToRotation);
      else
         velocityYawProperty.set(0.0);
   }

   private void appendingXAxis(double alpha)
   {
      velocityXProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingYAxis(double alpha)
   {
      velocityYProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingZAxisNegative(double alpha)
   {
      velocityZProperty.set(alpha * ratioJoyStickToPosition);
   }

   private void appendingZAxisPositive(ButtonState state)
   {
      if (state == ButtonState.PRESSED)
         velocityZProperty.set(ratioJoyStickToPosition);
      else
         velocityZProperty.set(0.0);
   }

   private void createKeyFrame(ButtonState state, RobotSide preferredSide)
   {
      if (state == ButtonState.PRESSED)
      {
         motionPreviewVisualizer.enable(false);

         List<RigidBodyTransform> keyFramePoses = sideDependentKeyFramePoses.get(preferredSide);

         RigidBodyTransform transformToCreateKeyFrame = new RigidBodyTransform();
         int numberOfKeyFrames = keyFramePoses.size();

         RigidBodyTransform controlFrameToWorld = new RigidBodyTransform(fullRobotModel.getHandControlFrame(preferredSide).getTransformToWorldFrame());

         if (numberOfKeyFrames == 0)
            transformToCreateKeyFrame.set(controlFrameToWorld);
         else
            transformToCreateKeyFrame.set(keyFramePoses.get(numberOfKeyFrames - 1));

         controlSide = preferredSide;
         controlTransform.set(transformToCreateKeyFrame);
         keyFramePoses.add(transformToCreateKeyFrame);
         indexOfSelectedKeyFrame = keyFramePoses.size() - 1;
      }
   }

   private void switchSelectedObject(ButtonState state)
   {
      List<RigidBodyTransform> keyFramePoses = sideDependentKeyFramePoses.get(controlSide);

      int numberOfObjects = keyFramePoses.size();

      if (numberOfObjects < 1)
         return;

      if (state == ButtonState.RELEASED)
         return;

      indexOfSelectedKeyFrame++;
      if (indexOfSelectedKeyFrame == numberOfObjects)
         indexOfSelectedKeyFrame = 0;

      snapControlTransformToSelectedKeyFrame();
   }

   private void clearKeyFrame(ButtonState state)
   {
      if (state == ButtonState.RELEASED)
      {
         motionPreviewVisualizer.enable(false);
         List<RigidBodyTransform> keyFramePoses = sideDependentKeyFramePoses.get(controlSide);
         if (keyFramePoses.size() > 0)
         {
            keyFramePoses.remove(indexOfSelectedKeyFrame);
            indexOfSelectedKeyFrame = keyFramePoses.size() - 1;

            if (keyFramePoses.size() > 0)
               snapControlTransformToSelectedKeyFrame();
            if (indexOfSelectedKeyFrame < 0)
               snapControlTransformToCurrentHand();
         }
      }
   }

   private void snapControlTransformToSelectedKeyFrame()
   {
      List<RigidBodyTransform> keyFramePoses = sideDependentKeyFramePoses.get(controlSide);
      controlTransform.set(keyFramePoses.get(indexOfSelectedKeyFrame));
   }

   private void snapControlTransformToCurrentHand()
   {
      RigidBodyTransform currentHandTransform = new RigidBodyTransform(fullRobotModel.getHand(controlSide).getBodyFixedFrame().getTransformToWorldFrame());
      controlTransform.set(currentHandTransform);
   }

   private void updateVisualizedKeyFrames()
   {
      List<Node> objectsToPutReference = new ArrayList<Node>();

      for (RobotSide robotSide : RobotSide.values)
      {
         List<RigidBodyTransform> keyFramePoses = sideDependentKeyFramePoses.get(robotSide);
         for (int i = 0; i < keyFramePoses.size(); i++)
         {
            RigidBodyTransform objectToVisualize = keyFramePoses.get(i);
            if (i == indexOfSelectedKeyFrame && robotSide == controlSide)
               objectToVisualize.set(controlTransform);
            double lengthOfFrame = lengthOfkeyFrameReferenceFrame;

            Tuple3DBasics translation = new Point3D(objectToVisualize.getTranslationVector());
            Quaternion orientation = new Quaternion(objectToVisualize.getRotationMatrix());

            JavaFXCoordinateSystem controlCoordinateSystem = new JavaFXCoordinateSystem(lengthOfFrame);
            Affine controlTransform = JavaFXTools.createAffineFromQuaternionAndTuple(new Quaternion(orientation), translation);
            controlCoordinateSystem.getTransforms().add(controlTransform);
            objectsToPutReference.add(controlCoordinateSystem);
         }
      }

      objectsToVisualizeReference.set(objectsToPutReference);

      List<Node> objectsToVisualize = objectsToVisualizeReference.getAndSet(null);
      ObservableList<Node> children = rootNode.getChildren();

      if (objectsToVisualize != null)
      {
         children.addAll(objectsToVisualize);
      }
   }

   private void updateControlHand()
   {
      if (controlSide != null)
      {
         JavaFXRobotHandVisualizer handViz = sideDependentHandVizs.get(controlSide);

         handViz.updateTransform(controlTransform);
         rootNode.getChildren().add(handViz.getRootNode());
      }
   }

   private int getLeastCommonMultiple(int numberOne, int numberTwo)
   {
      int reminder = 1;
      int multiple = numberOne * numberTwo;
      while (reminder != 0)
      {
         reminder = numberTwo % numberOne;
         numberTwo = numberOne;
         numberOne = reminder;
      }
      int greatestCommonDivisor = numberTwo;
      int leastCommonMultiple = multiple / greatestCommonDivisor;
      return leastCommonMultiple;
   }

   public void start()
   {
      motionPreviewVisualizer.start();
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
