package us.ihmc.humanoidBehaviors.behaviors.primitives;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WholeBodyInverseKinematicsBehavior extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable solutionQualityThreshold;
   private final DoubleYoVariable currentSolutionQuality;
   private final BooleanYoVariable isPaused;
   private final BooleanYoVariable isStopped;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable hasSolverFailed;
   private final BooleanYoVariable hasSentMessageToController;

   private final SideDependentList<DenseMatrix64F> handSelectionMatrices = new SideDependentList<>(CommonOps.identity(6), CommonOps.identity(6));
   private final SideDependentList<YoFramePoint> yoDesiredHandPositions = new SideDependentList<>();
   private final SideDependentList<YoFrameQuaternion> yoDesiredHandOrientations = new SideDependentList<>();
   private final DoubleYoVariable trajectoryTime;

   private final KinematicsToolboxOutputConverter outputConverter;
   private final FullHumanoidRobotModel fullRobotModel;
   private ChestTrajectoryMessage chestTrajectoryMessage;
   private PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage;
   private SideDependentList<HandTrajectoryMessage> handTrajectoryMessage = new SideDependentList<>();

   private final ConcurrentListeningQueue<KinematicsToolboxOutputStatus> kinematicsToolboxOutputQueue = new ConcurrentListeningQueue<>(40);
   private KinematicsToolboxOutputStatus solutionSentToController = null;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable timeSolutionSentToController;

   public WholeBodyInverseKinematicsBehavior(FullHumanoidRobotModelFactory fullRobotModelFactory, DoubleYoVariable yoTime,
                                             CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel)
   {
      this(null, fullRobotModelFactory, yoTime, outgoingCommunicationBridge, fullRobotModel);
   }

   public WholeBodyInverseKinematicsBehavior(String namePrefix, FullHumanoidRobotModelFactory fullRobotModelFactory, DoubleYoVariable yoTime,
                                             CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel)
   {
      super(namePrefix, outgoingCommunicationBridge);
      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;

      solutionQualityThreshold = new DoubleYoVariable(behaviorName + "SolutionQualityThreshold", registry);
      solutionQualityThreshold.set(0.005);
      isPaused = new BooleanYoVariable(behaviorName + "IsPaused", registry);
      isStopped = new BooleanYoVariable(behaviorName + "IsStopped", registry);
      isDone = new BooleanYoVariable(behaviorName + "IsDone", registry);
      hasSolverFailed = new BooleanYoVariable(behaviorName + "HasSolverFailed", registry);
      hasSentMessageToController = new BooleanYoVariable(behaviorName + "HasSentMessageToController", registry);

      currentSolutionQuality = new DoubleYoVariable(behaviorName + "CurrentSolutionQuality", registry);
      trajectoryTime = new DoubleYoVariable(behaviorName + "TrajectoryTime", registry);
      timeSolutionSentToController = new DoubleYoVariable(behaviorName + "TimeSolutionSentToController", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         YoFramePoint desiredHandPosition = new YoFramePoint(behaviorName + "Desired" + side + "Hand", worldFrame, registry);
         yoDesiredHandPositions.put(robotSide, desiredHandPosition);
         YoFrameQuaternion desiredHandOrientation = new YoFrameQuaternion(behaviorName + "Desired" + side + "Hand", worldFrame, registry);
         yoDesiredHandOrientations.put(robotSide, desiredHandOrientation);
      }

      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      attachNetworkListeningQueue(kinematicsToolboxOutputQueue, KinematicsToolboxOutputStatus.class);

      clear();
   }

   public void clear()
   {
      currentSolutionQuality.set(Double.POSITIVE_INFINITY);

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

   private final FramePoint desiredPosition = new FramePoint();
   private final FrameOrientation desiredOrientation = new FrameOrientation();

   public void setDesiredHandPose(RobotSide robotSide, FramePose desiredHandPose)
   {
      desiredHandPose.getPoseIncludingFrame(desiredPosition, desiredOrientation);
      setDesiredHandPose(robotSide, desiredPosition, desiredOrientation);
   }

   public void setDesiredHandPose(RobotSide robotSide, FramePoint desiredHandPosition, FrameOrientation desiredHandOrientation)
   {
      yoDesiredHandPositions.get(robotSide).setAndMatchFrame(desiredHandPosition);
      yoDesiredHandOrientations.get(robotSide).setAndMatchFrame(desiredHandOrientation);
   }

   public void setHandLinearControlOnly(RobotSide robotSide)
   {
      boolean[] controlledPositionAxes = new boolean[] {true, true, true};
      boolean[] controlledOrientationAxes = new boolean[] {false, false, false};
      setHandControlledAxes(robotSide, controlledPositionAxes, controlledOrientationAxes);
   }

   public void setHandLinearControlAndYawPitchOnly(RobotSide robotSide)
   {
      boolean[] controlledPositionAxes = new boolean[] {true, true, true};
      boolean[] controlledOrientationAxes = new boolean[] {false, true, true};
      setHandControlledAxes(robotSide, controlledPositionAxes, controlledOrientationAxes);
   }

   public void setHandControlledAxes(RobotSide robotSide, boolean[] controlledPositionAxes, boolean[] controlledOrientationAxes)
   {
      DenseMatrix64F selectionMatrix = handSelectionMatrices.get(robotSide);
      selectionMatrix.reshape(6, 6);
      selectionMatrix.zero();

      for (int i = 0; i < 3; i++)
      {
         selectionMatrix.set(i, i, controlledOrientationAxes[i]);
         selectionMatrix.set(i + 3, i, controlledPositionAxes[i]);
      }
   }

   @Override
   public void initialize()
   {

      System.out.println("init whole body behavior");
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSentMessageToController.set(false);
      hasSolverFailed.set(false);
      solutionSentToController = null;
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(message);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFramePoint yoDesiredHandPosition = yoDesiredHandPositions.get(robotSide);
         YoFrameQuaternion yoDesiredHandOrientation = yoDesiredHandOrientations.get(robotSide);

         if (yoDesiredHandPosition.containsNaN() || yoDesiredHandOrientation.containsNaN())
         {
            handTrajectoryMessage.put(robotSide, null);
         }
         else
         {
            Point3d desiredHandPosition = new Point3d();
            Quat4d desiredHandOrientation = new Quat4d();
            yoDesiredHandPosition.get(desiredHandPosition);
            yoDesiredHandOrientation.get(desiredHandOrientation);
            HandTrajectoryMessage temporaryHandTrajectoryMessage = new HandTrajectoryMessage(robotSide, 0.0, desiredHandPosition, desiredHandOrientation);
            handTrajectoryMessage.put(robotSide, temporaryHandTrajectoryMessage);
         }
      }

      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      Quat4d desiredChestOrientation = new Quat4d();
      FrameOrientation desiredChestFrame = new FrameOrientation(chestFrame);
      desiredChestFrame.changeFrame(worldFrame);
      desiredChestFrame.getQuaternion(desiredChestOrientation);
      chestTrajectoryMessage = new ChestTrajectoryMessage(0.0, desiredChestOrientation);

      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      Quat4d desiredPelvisOrientation = new Quat4d();
      FrameOrientation desiredPelvisFrame = new FrameOrientation(pelvisFrame);
      desiredPelvisFrame.changeFrame(worldFrame);
      desiredPelvisFrame.getQuaternion(desiredPelvisOrientation);
      pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(0.0, desiredPelvisOrientation);
   }

   @Override
   public void doControl()
   {
      if (!hasSentMessageToController.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if(handTrajectoryMessage.get(robotSide) != null)
            {
               handTrajectoryMessage.get(robotSide).setSelectionMatrix(handSelectionMatrices.get(robotSide));
               handTrajectoryMessage.get(robotSide).setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
               sendPacket(handTrajectoryMessage.get(robotSide));
            }
         }

         if (chestTrajectoryMessage != null)
         {
            chestTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
            sendPacket(chestTrajectoryMessage);
         }

         if (pelvisOrientationTrajectoryMessage != null)
         {
            pelvisOrientationTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
            sendPacket(pelvisOrientationTrajectoryMessage);
         }
      }
      if (kinematicsToolboxOutputQueue.isNewPacketAvailable() && !hasSentMessageToController.getBooleanValue())
      {
         KinematicsToolboxOutputStatus newestSolution = kinematicsToolboxOutputQueue.poll();

         boolean isSolutionStable = currentSolutionQuality.getDoubleValue() - newestSolution.getSolutionQuality() < 1.0e-6;
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
               solutionSentToController = newestSolution;
               outputConverter.setTrajectoryTime(trajectoryTime.getDoubleValue());
               WholeBodyTrajectoryMessage message = new WholeBodyTrajectoryMessage();
               message.setDestination(PacketDestination.CONTROLLER);
               outputConverter.updateFullRobotModel(newestSolution);
               outputConverter.setMessageToCreate(message);
               outputConverter.computeHandTrajectoryMessages();
               outputConverter.computeChestTrajectoryMessage();
               outputConverter.computePelvisTrajectoryMessage();
               sendPacketToController(message);
               hasSentMessageToController.set(true);
               deactivateKinematicsToolboxModule();
               timeSolutionSentToController.set(yoTime.getDoubleValue());
            }
         }
         currentSolutionQuality.set(newestSolution.getSolutionQuality());

         newestSolution.setDestination(PacketDestination.UI);
         sendPacket(newestSolution);
      }
      else if (hasSentMessageToController.getBooleanValue())
      {
         if (solutionSentToController != null && !isDone.getBooleanValue()) // To visualize the solution sent to the controller
            sendPacket(solutionSentToController);

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
   public void doPostBehaviorCleanup()
   {
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSolverFailed.set(false);
      hasSentMessageToController.set(false);
      solutionSentToController = null;
      chestTrajectoryMessage = null;
      pelvisOrientationTrajectoryMessage = null;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryMessage.put(robotSide, null);
      }
      
      deactivateKinematicsToolboxModule();
   }

   private void deactivateKinematicsToolboxModule()
   {
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.SLEEP);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(message);
   }
}
