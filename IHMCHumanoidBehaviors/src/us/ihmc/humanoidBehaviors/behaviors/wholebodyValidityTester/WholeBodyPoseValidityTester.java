package us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.TrackingWeightsMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WholeBodyPoseValidityTester extends AbstractBehavior
{   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullHumanoidRobotModel fullRobotModel;
   
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame chestFrame;
   
//   private final DoubleYoVariable yoTime;
//   private final DoubleYoVariable solutionQualityThreshold;
//   private final BooleanYoVariable isPaused;
//   private final BooleanYoVariable isStopped;
//   private final BooleanYoVariable isDone;
//   private final BooleanYoVariable hasSolverFailed;
//   private final BooleanYoVariable hasSentMessageToController;   
//   private final DoubleYoVariable currentSolutionQuality;
//   private final DoubleYoVariable trajectoryTime;
//   private final DoubleYoVariable timeSolutionSentToController;
   
   private final KinematicsToolboxOutputConverter outputConverter;
   private final ConcurrentListeningQueue<KinematicsToolboxOutputStatus> kinematicsToolboxOutputQueue = new ConcurrentListeningQueue<>(40);
      
//   private final SideDependentList<YoFramePoint> yoDesiredHandPositions = new SideDependentList<>();
//   private final SideDependentList<YoFrameQuaternion> yoDesiredHandOrientations = new SideDependentList<>();
//   private final YoFrameQuaternion yoDesiredChestOrientation;
//   private final YoFrameQuaternion yoDesiredPelvisOrientation;
//   private final DoubleYoVariable yoDesiredPelvisHeight;
   
//   private ChestTrajectoryMessage chestTrajectoryMessage;
//   private PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage;
//   private PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage;
//   private SideDependentList<HandTrajectoryMessage> handTrajectoryMessage = new SideDependentList<>();
//   private TrackingWeightsMessage trackingWeightsMessage;
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage; // -------
   
   public WholeBodyPoseValidityTester(FullHumanoidRobotModelFactory fullRobotModelFactory, DoubleYoVariable yoTime,
                                      CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel)
   {
      super(null, outgoingCommunicationBridge);
      
//      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();

//      solutionQualityThreshold = new DoubleYoVariable(behaviorName + "SolutionQualityThreshold", registry);
//      solutionQualityThreshold.set(0.005);
//      isPaused = new BooleanYoVariable(behaviorName + "IsPaused", registry);
//      isStopped = new BooleanYoVariable(behaviorName + "IsStopped", registry);
//      isDone = new BooleanYoVariable(behaviorName + "IsDone", registry);
//      hasSolverFailed = new BooleanYoVariable(behaviorName + "HasSolverFailed", registry);
//      hasSentMessageToController = new BooleanYoVariable(behaviorName + "HasSentMessageToController", registry);
//
//      currentSolutionQuality = new DoubleYoVariable(behaviorName + "CurrentSolutionQuality", registry);
//      trajectoryTime = new DoubleYoVariable(behaviorName + "TrajectoryTime", registry);
//      timeSolutionSentToController = new DoubleYoVariable(behaviorName + "TimeSolutionSentToController", registry);


      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      attachNetworkListeningQueue(kinematicsToolboxOutputQueue, KinematicsToolboxOutputStatus.class);

   }
   
   
   public void setWholeBodyPose(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
      this.wholebodyTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(this.wholebodyTrajectoryMessage);
   }
   
   public KinematicsToolboxOutputStatus getOutputStatus()
   {
      KinematicsToolboxOutputStatus newestSolution = kinematicsToolboxOutputQueue.poll();
      
      outputConverter.updateFullRobotModel(newestSolution);
      
      return newestSolution;
   }
   
   public FullHumanoidRobotModel getFullHumanoidRobotModel()
   {
      KinematicsToolboxOutputStatus newestSolution = kinematicsToolboxOutputQueue.poll();
      
      outputConverter.updateFullRobotModel(newestSolution);
      
      return outputConverter.getFullRobotModel();
   }

   @Override
   public void doControl()
   {
      
   }

   @Override
   public void onBehaviorEntered()
   {
      System.out.println("Whole Body Pose Validity Tester Entered");
      isPaused.set(false);
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(message);
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      
      wholebodyTrajectoryMessage = null;
      
      deactivateKinematicsToolboxModule();
      
   }

   private void deactivateKinematicsToolboxModule()
   {
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.SLEEP);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(message);
   }
   
   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }
}
