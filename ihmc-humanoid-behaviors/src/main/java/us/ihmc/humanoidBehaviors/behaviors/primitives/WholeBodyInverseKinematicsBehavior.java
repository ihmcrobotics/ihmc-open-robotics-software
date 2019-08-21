package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;

public class WholeBodyInverseKinematicsBehavior extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble solutionQualityThreshold;
   private final YoDouble currentSolutionQuality;
   private final YoBoolean isPaused;
   private final YoBoolean isStopped;
   private final YoBoolean isDone;
   private final YoBoolean hasSolverFailed;
   private final YoBoolean hasSentMessageToController;

   private final SideDependentList<SelectionMatrix6D> handSelectionMatrices = new SideDependentList<>(new SelectionMatrix6D(), new SelectionMatrix6D());
   private final SelectionMatrix6D chestSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D pelvisSelectionMatrix = new SelectionMatrix6D();
   private final SideDependentList<YoFramePoint3D> yoDesiredHandPositions = new SideDependentList<>();
   private final SideDependentList<YoFrameQuaternion> yoDesiredHandOrientations = new SideDependentList<>();
   private final YoFrameQuaternion yoDesiredChestOrientation;
   private final YoFrameQuaternion yoDesiredPelvisOrientation;
   private final YoFramePoint3D yoDesiredPelvisPosition;
   private final YoDouble trajectoryTime;

   private final KinematicsToolboxOutputConverter outputConverter;
   private final FullHumanoidRobotModel fullRobotModel;
   private KinematicsToolboxRigidBodyMessage chestMessage;
   private KinematicsToolboxRigidBodyMessage pelvisMessage;
   private SideDependentList<KinematicsToolboxRigidBodyMessage> handMessages = new SideDependentList<>();

   private final ConcurrentListeningQueue<KinematicsToolboxOutputStatus> kinematicsToolboxOutputQueue = new ConcurrentListeningQueue<>(40);

   private final YoDouble yoTime;
   private final YoDouble timeSolutionSentToController;

   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<KinematicsToolboxRigidBodyMessage> kinematicsToolboxRigidBodyPublisher;
   private final IHMCROS2Publisher<WholeBodyTrajectoryMessage> wholeBodyTrajectoryPublisher;

   public WholeBodyInverseKinematicsBehavior(String robotName, FullHumanoidRobotModelFactory fullRobotModelFactory, YoDouble yoTime,
                                             Ros2Node ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      this(robotName, null, fullRobotModelFactory, yoTime, ros2Node, fullRobotModel);
   }

   public WholeBodyInverseKinematicsBehavior(String robotName, String namePrefix, FullHumanoidRobotModelFactory fullRobotModelFactory, YoDouble yoTime,
                                             Ros2Node ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      super(robotName, namePrefix, ros2Node);
      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;

      solutionQualityThreshold = new YoDouble(behaviorName + "SolutionQualityThreshold", registry);
      solutionQualityThreshold.set(0.005);
      isPaused = new YoBoolean(behaviorName + "IsPaused", registry);
      isStopped = new YoBoolean(behaviorName + "IsStopped", registry);
      isDone = new YoBoolean(behaviorName + "IsDone", registry);
      hasSolverFailed = new YoBoolean(behaviorName + "HasSolverFailed", registry);
      hasSentMessageToController = new YoBoolean(behaviorName + "HasSentMessageToController", registry);

      currentSolutionQuality = new YoDouble(behaviorName + "CurrentSolutionQuality", registry);
      trajectoryTime = new YoDouble(behaviorName + "TrajectoryTime", registry);
      timeSolutionSentToController = new YoDouble(behaviorName + "TimeSolutionSentToController", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         YoFramePoint3D desiredHandPosition = new YoFramePoint3D(behaviorName + "Desired" + side + "Hand", worldFrame, registry);
         yoDesiredHandPositions.put(robotSide, desiredHandPosition);
         YoFrameQuaternion desiredHandOrientation = new YoFrameQuaternion(behaviorName + "Desired" + side + "Hand", worldFrame, registry);
         yoDesiredHandOrientations.put(robotSide, desiredHandOrientation);
      }

      yoDesiredChestOrientation = new YoFrameQuaternion(behaviorName + "DesiredChest", worldFrame, registry);
      yoDesiredPelvisOrientation = new YoFrameQuaternion(behaviorName + "DesiredPelvis", worldFrame, registry);
      yoDesiredPelvisPosition = new YoFramePoint3D(behaviorName + "DesiredPelvis", worldFrame, registry);

      pelvisSelectionMatrix.setToAngularSelectionOnly();
      chestSelectionMatrix.setToAngularSelectionOnly();

      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      createSubscriber(KinematicsToolboxOutputStatus.class, kinematicsToolboxPubGenerator, kinematicsToolboxOutputQueue::put);
      toolboxStatePublisher = createPublisher(ToolboxStateMessage.class, kinematicsToolboxSubGenerator);
      kinematicsToolboxRigidBodyPublisher = createPublisher(KinematicsToolboxRigidBodyMessage.class, kinematicsToolboxSubGenerator);
      wholeBodyTrajectoryPublisher = createPublisherForController(WholeBodyTrajectoryMessage.class);

      clear();
   }

   public void clear()
   {
      currentSolutionQuality.set(Double.POSITIVE_INFINITY);

      yoDesiredChestOrientation.setToNaN();
      yoDesiredPelvisOrientation.setToNaN();
      yoDesiredPelvisPosition.setToNaN();

      for (RobotSide robotSide : RobotSide.values)
      {
         yoDesiredHandPositions.get(robotSide).setToNaN();
         yoDesiredHandOrientations.get(robotSide).setToNaN();
      }
   }

   /** Change the threshold at which a solution is considered to be good enough */
   public void setSolutionQualityThreshold(double newThreshold)
   {
      solutionQualityThreshold.set(newThreshold);
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime.set(trajectoryTime);
   }

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();

   public void setDesiredHandPose(RobotSide robotSide, FramePose3D desiredHandPose)
   {
      desiredHandPose.get(desiredPosition, desiredOrientation);
      setDesiredHandPose(robotSide, desiredPosition, desiredOrientation);
   }

   public void setDesiredHandPose(RobotSide robotSide, FramePoint3D desiredHandPosition, FrameQuaternion desiredHandOrientation)
   {
      yoDesiredHandPositions.get(robotSide).setMatchingFrame(desiredHandPosition);
      yoDesiredHandOrientations.get(robotSide).setMatchingFrame(desiredHandOrientation);
   }

   public void setHandLinearControlOnly(RobotSide robotSide)
   {
      handSelectionMatrices.get(robotSide).setToLinearSelectionOnly();
   }

   public void setHandLinearControlAndYawPitchOnly(RobotSide robotSide)
   {
      handSelectionMatrices.get(robotSide).resetSelection();
      handSelectionMatrices.get(robotSide).selectAngularX(false);
   }

   public void holdCurrentChestOrientation()
   {
      FrameQuaternion currentChestOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      yoDesiredChestOrientation.setMatchingFrame(currentChestOrientation);
      chestSelectionMatrix.setToAngularSelectionOnly();
   }

   public void setDesiredChestOrientation(FrameQuaternion desiredChestOrientation)
   {
      yoDesiredChestOrientation.setMatchingFrame(desiredChestOrientation);
   }

   public void setChestAngularControl(boolean roll, boolean pitch, boolean yaw)
   {
      chestSelectionMatrix.setAngularAxisSelection(roll, pitch, yaw);
      chestSelectionMatrix.clearLinearSelection();
   }

   public void holdCurrentPelvisOrientation()
   {
      FrameQuaternion currentPelvisOrientation = new FrameQuaternion(fullRobotModel.getPelvis().getBodyFixedFrame());
      yoDesiredPelvisOrientation.setMatchingFrame(currentPelvisOrientation);
      pelvisSelectionMatrix.setToAngularSelectionOnly();
   }

   public void setDesiredPelvisOrientation(FrameQuaternion desiredPelvisOrientation)
   {
      yoDesiredPelvisOrientation.setMatchingFrame(desiredPelvisOrientation);
   }

   public void setPelvisAngularControl(boolean roll, boolean pitch, boolean yaw)
   {
      pelvisSelectionMatrix.setAngularAxisSelection(roll, pitch, yaw);
      pelvisSelectionMatrix.clearLinearSelection();
   }

   public void holdCurrentPelvisHeight()
   {
      yoDesiredPelvisPosition.setFromReferenceFrame(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      pelvisSelectionMatrix.clearLinearSelection();
      pelvisSelectionMatrix.selectLinearZ(true);
      pelvisSelectionMatrix.setSelectionFrame(worldFrame);
   }

   public void setDesiredPelvisHeight(FramePoint3D pointContainingDesiredHeight)
   {
      yoDesiredPelvisPosition.setMatchingFrame(pointContainingDesiredHeight);
      pelvisSelectionMatrix.clearLinearSelection();
      pelvisSelectionMatrix.selectLinearZ(true);
      pelvisSelectionMatrix.setSelectionFrame(worldFrame);
   }

   public void setDesiredPelvisHeight(double desiredHeightInWorld)
   {
      yoDesiredPelvisPosition.setZ(desiredHeightInWorld);
      pelvisSelectionMatrix.clearLinearSelection();
      pelvisSelectionMatrix.selectLinearZ(true);
      pelvisSelectionMatrix.setSelectionFrame(worldFrame);
   }

   public double getSolutionQuality()
   {
      return currentSolutionQuality.getDoubleValue();
   }

   @Override
   public void onBehaviorEntered()
   {

      System.out.println("init whole body behavior");
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSentMessageToController.set(false);
      hasSolverFailed.set(false);
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFramePoint3D yoDesiredHandPosition = yoDesiredHandPositions.get(robotSide);
         YoFrameQuaternion yoDesiredHandOrientation = yoDesiredHandOrientations.get(robotSide);

         if (yoDesiredHandPosition.containsNaN() || yoDesiredHandOrientation.containsNaN())
         {
            handMessages.put(robotSide, null);
         }
         else
         {
            Point3D desiredHandPosition = new Point3D(yoDesiredHandPosition);
            Quaternion desiredHandOrientation = new Quaternion(yoDesiredHandOrientation);
            RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
            ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
            KinematicsToolboxRigidBodyMessage handMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(hand, handControlFrame, desiredHandPosition,
                                                                                                                 desiredHandOrientation);
            handMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            handMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(20.0));
            handMessages.put(robotSide, handMessage);
         }
      }

      if (yoDesiredChestOrientation.containsNaN())
      {
         chestMessage = null;
      }
      else
      {
         Quaternion desiredChestOrientation = new Quaternion(yoDesiredChestOrientation);
         RigidBodyBasics chest = fullRobotModel.getChest();
         chestMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(chest, desiredChestOrientation);
         chestMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0.02));
         chestMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0.02));
      }

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      if (yoDesiredPelvisOrientation.containsNaN() && yoDesiredPelvisPosition.containsNaN())
         pelvisMessage = null;
      else
         pelvisMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(pelvis);

      if (!yoDesiredPelvisOrientation.containsNaN())
      {
         pelvisMessage.getDesiredOrientationInWorld().set(yoDesiredPelvisOrientation);
         pelvisMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0.02));
         pelvisMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0.02));
      }

      if (!yoDesiredPelvisPosition.containsNaN())
      {
         Point3D desiredPelvisPosition = new Point3D();
         desiredPelvisPosition.set(yoDesiredPelvisPosition);
         pelvisMessage.getDesiredPositionInWorld().set(desiredPelvisPosition);
         pelvisMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0.02));
         pelvisMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0.02));
      }
   }

   @Override
   public void doControl()
   {
      if (!hasSentMessageToController.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            KinematicsToolboxRigidBodyMessage handMessage = handMessages.get(robotSide);
            if (handMessage != null)
            {
               SelectionMatrix6D selectionMatrix6D = handSelectionMatrices.get(robotSide);
               handMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart()));
               handMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getLinearPart()));
               kinematicsToolboxRigidBodyPublisher.publish(handMessage);
            }
         }

         if (chestMessage != null)
         {
            chestMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(chestSelectionMatrix.getAngularPart()));
            chestMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(chestSelectionMatrix.getLinearPart()));
            kinematicsToolboxRigidBodyPublisher.publish(chestMessage);
         }

         if (pelvisMessage != null)
         {
            pelvisMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(pelvisSelectionMatrix.getAngularPart()));
            pelvisMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(pelvisSelectionMatrix.getLinearPart()));
            kinematicsToolboxRigidBodyPublisher.publish(pelvisMessage);
         }
      }

      if (kinematicsToolboxOutputQueue.isNewPacketAvailable() && !hasSentMessageToController.getBooleanValue())
      {
         KinematicsToolboxOutputStatus newestSolution = kinematicsToolboxOutputQueue.poll();

         double deltaSolutionQuality = currentSolutionQuality.getDoubleValue() - newestSolution.getSolutionQuality();
         boolean isSolutionStable = deltaSolutionQuality > 0.0 && deltaSolutionQuality < 1.0e-6;
         boolean isSolutionGoodEnough = newestSolution.getSolutionQuality() < solutionQualityThreshold.getDoubleValue();
         boolean sendSolutionToController = isSolutionStable && isSolutionGoodEnough;
         if (!isPaused())
         {
            if (isSolutionStable && !isSolutionGoodEnough)
            {
               hasSolverFailed.set(true);
            }
            else if (sendSolutionToController)
            {
               outputConverter.setTrajectoryTime(trajectoryTime.getDoubleValue());
               WholeBodyTrajectoryMessage message = new WholeBodyTrajectoryMessage();
               message.setDestination(PacketDestination.CONTROLLER.ordinal());
               outputConverter.updateFullRobotModel(newestSolution);
               outputConverter.setMessageToCreate(message);
               outputConverter.computeHandTrajectoryMessages();
               outputConverter.computeChestTrajectoryMessage();
               outputConverter.computePelvisTrajectoryMessage();
               wholeBodyTrajectoryPublisher.publish(message);
               hasSentMessageToController.set(true);
               deactivateKinematicsToolboxModule();
               timeSolutionSentToController.set(yoTime.getDoubleValue());
            }
         }
         currentSolutionQuality.set(newestSolution.getSolutionQuality());
      }
      else if (hasSentMessageToController.getBooleanValue())
      {
         if (yoTime.getDoubleValue() - timeSolutionSentToController.getDoubleValue() > trajectoryTime.getDoubleValue())
         {
            isDone.set(true);
         }
      }
   }

   public boolean hasSolverFailed()
   {
      return hasSolverFailed.getBooleanValue();
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue() || hasSolverFailed.getBooleanValue();
   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isStopped.set(false);
      hasSentMessageToController.set(false);
      chestMessage = null;
      pelvisMessage = null;
      pelvisSelectionMatrix.setToAngularSelectionOnly();
      chestSelectionMatrix.setToAngularSelectionOnly();

      for (RobotSide robotSide : RobotSide.values)
      {
         handMessages.put(robotSide, null);
      }

      deactivateKinematicsToolboxModule();
   }

   private void deactivateKinematicsToolboxModule()
   {
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

}
