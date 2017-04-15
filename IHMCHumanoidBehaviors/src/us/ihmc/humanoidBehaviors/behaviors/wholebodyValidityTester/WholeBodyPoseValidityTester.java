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
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
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
   
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable solutionQualityThreshold;
   private final BooleanYoVariable isPaused;
   private final BooleanYoVariable isStopped;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable hasSolverFailed;
   private final BooleanYoVariable hasSentMessageToController;   
   private final DoubleYoVariable currentSolutionQuality;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable timeSolutionSentToController;
   
   private final KinematicsToolboxOutputConverter outputConverter;
   private final ConcurrentListeningQueue<KinematicsToolboxOutputStatus> kinematicsToolboxOutputQueue = new ConcurrentListeningQueue<>(40);
   private KinematicsToolboxOutputStatus solutionSentToController = null;
   
   private final SideDependentList<YoFramePoint> yoDesiredHandPositions = new SideDependentList<>();
   private final SideDependentList<YoFrameQuaternion> yoDesiredHandOrientations = new SideDependentList<>();
   private final YoFrameQuaternion yoDesiredChestOrientation;
   private final YoFrameQuaternion yoDesiredPelvisOrientation;
   private final DoubleYoVariable yoDesiredPelvisHeight;
   
//   private ChestTrajectoryMessage chestTrajectoryMessage;
//   private PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage;
//   private PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage;
//   private SideDependentList<HandTrajectoryMessage> handTrajectoryMessage = new SideDependentList<>();
//   private TrackingWeightsMessage trackingWeightsMessage;
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage; // -------
   
   private boolean isSentToToolbox = false;
   private boolean getMessage = false;
   private boolean forceOut = false;
   
   protected RobotCollisionModel robotCollisionModel;

   public boolean isValid = true;
   protected boolean collisionFree = true;
   protected boolean jointlimitFree = true;
   
   
   public WholeBodyPoseValidityTester(FullHumanoidRobotModelFactory fullRobotModelFactory, DoubleYoVariable yoTime,
                                      CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel)
   {
      super(null, outgoingCommunicationBridge);
      
      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();

      solutionQualityThreshold = new DoubleYoVariable(behaviorName + "SolutionQualityThreshold", registry);
      solutionQualityThreshold.set(0.01);
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
      
      yoDesiredChestOrientation = new YoFrameQuaternion(behaviorName + "DesiredChest", worldFrame, registry);
      yoDesiredPelvisOrientation = new YoFrameQuaternion(behaviorName + "DesiredPelvis", worldFrame, registry);
      yoDesiredPelvisHeight = new DoubleYoVariable(behaviorName + "DesiredPelvisHeight", registry);
      
      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      attachNetworkListeningQueue(kinematicsToolboxOutputQueue, KinematicsToolboxOutputStatus.class);
      clear();
   }
   
   public void clear()
   {
      currentSolutionQuality.set(Double.POSITIVE_INFINITY);

   }
   
   
   
   public void setWholeBodyPose(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {  
      this.wholebodyTrajectoryMessage = wholebodyTrajectoryMessage;
      this.wholebodyTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);

      if(wholebodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT) == null)
      {
         PrintTools.info("R Hand");         
      }
      if(wholebodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.LEFT) == null)
      {
         PrintTools.info("L Hand");  
      }
      if(wholebodyTrajectoryMessage.getChestTrajectoryMessage() == null)
      {
         PrintTools.info("Hold Chest");   
         
         
         FrameOrientation currentChestOrientation = new FrameOrientation(fullRobotModel.getChest().getBodyFixedFrame());
         
         
         yoDesiredChestOrientation.setAndMatchFrame(currentChestOrientation);
         
         Quaternion desiredChestOrientation = new Quaternion();
         yoDesiredChestOrientation.get(desiredChestOrientation);
         
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(0.0, desiredChestOrientation, worldFrame, pelvisZUpFrame);
         this.wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
//         chestTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
//         sendPacket(chestTrajectoryMessage);
      }
      
      if(wholebodyTrajectoryMessage.getPelvisTrajectoryMessage() == null)         
      {
         PrintTools.info("Hold Pelvis");         
         
         
         FrameOrientation currentPelvisOrientation = new FrameOrientation(fullRobotModel.getPelvis().getBodyFixedFrame());
         yoDesiredPelvisOrientation.setAndMatchFrame(currentPelvisOrientation);
         YoFrameQuaternion yoDesiredPelvisQuaternion = yoDesiredPelvisOrientation;
         Quaternion desiredPelvisOrientation = new Quaternion();
         yoDesiredPelvisQuaternion.get(desiredPelvisOrientation);
         PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(0.0, desiredPelvisOrientation);
         
         FramePoint currentPelvisPosition = new FramePoint(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
         currentPelvisPosition.changeFrame(worldFrame);
         yoDesiredPelvisHeight.set(currentPelvisPosition.getZ());
         PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(0.0, yoDesiredPelvisHeight.getDoubleValue());
         
         pelvisOrientationTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
         sendPacket(pelvisOrientationTrajectoryMessage);
         pelvisHeightTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
         sendPacket(pelvisHeightTrajectoryMessage);
         
      }
      
      isSentToToolbox = false;      
      isSolved = false;
      getMessage = true;
      
      
   }
   
   public KinematicsToolboxOutputStatus getKinematicsToolboxOutputStatus()
   {
      KinematicsToolboxOutputStatus newestSolution = kinematicsToolboxOutputQueue.poll();
            
      return newestSolution;
   }
   
   public FullHumanoidRobotModel getFullHumanoidRobotModel()
   {            
      return outputConverter.getFullRobotModel();
   }

   private int cnt = 0;
   
   private boolean isReceived = false;
   private boolean isSolved = false;
   
   private boolean isGoodSolutionOld = false;
   @Override
   public void doControl()
   {
      if (kinematicsToolboxOutputQueue.isNewPacketAvailable())
      {
         if(getMessage == true)
         {         
            if(isSentToToolbox == false)
            {
               PrintTools.info("Sending...");
               sendPacket(this.wholebodyTrajectoryMessage);
               isSentToToolbox = true;
            }
            
            KinematicsToolboxOutputStatus newestSolution = kinematicsToolboxOutputQueue.poll();
         
            
            double deltaSolutionQuality = currentSolutionQuality.getDoubleValue() - newestSolution.getSolutionQuality();
            boolean isSolutionStable = Math.abs(deltaSolutionQuality) < 0.002;
            boolean isSolutionGoodEnough = newestSolution.getSolutionQuality() < solutionQualityThreshold.getDoubleValue();
            boolean isGoodSolutionCur = isSolutionStable && isSolutionGoodEnough;
            if(isGoodSolutionCur == false && isGoodSolutionOld == true)
            {
               isReceived = true;
            }
            
            if(isReceived == true)
            {
               if(isGoodSolutionCur == true)
               {
                  isSolved = true;
               }
            }
            
            
            outputConverter.updateFullRobotModel(newestSolution);
            
            
            
            isGoodSolutionOld = isGoodSolutionCur;
            currentSolutionQuality.set(newestSolution.getSolutionQuality());
            
            cnt++;
//            PrintTools.info(""+cnt+" SQ "+ newestSolution.getSolutionQuality()+" isSolutionStable "+isSolutionStable+" isSolutionGoodEnough "+isSolutionGoodEnough
//                            +" isReceived "+isReceived
//                            +" isSolved "+isSolved);

            if(isSolved == true || forceOut == true)
            {
               PrintTools.info(""+cnt+" SQ "+ newestSolution.getSolutionQuality());
               PrintTools.info("Solution "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM03()
                               +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM13()
                               +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM23());
               isReceived = false;               
               getMessage = false;
               forceOut = false;
            }
         }
         else
         {
            
         }   
      }
   }
   
   public void forceOut()
   {
      PrintTools.info("forceout ");      
      forceOut = true;
   }

   @Override
   public void onBehaviorEntered()
   {
      PrintTools.info("Whole Body Pose Validity Tester Entered");
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSentMessageToController.set(false);
      hasSolverFailed.set(false);
      solutionSentToController = null;
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(message);
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub
      PrintTools.info("Aborted");
      
   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub
      PrintTools.info("Paused");
   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub
      PrintTools.info("Resumes");
   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSolverFailed.set(false);
      hasSentMessageToController.set(false);
      solutionSentToController = null;
      
      wholebodyTrajectoryMessage = null;
      
      PrintTools.info("Exited");
      
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
      return isSolved;
   }
   
   
   
   
   
   private void getResult()
   {
      // collision tester
      robotCollisionModel = new RobotCollisionModel(getFullHumanoidRobotModel());
      
      robotCollisionModel.update();
      collisionFree = robotCollisionModel.getCollisionResult();

      // joint limit tester
      /*
       * joint limit should be added. 170401
       */
      jointlimitFree = true;
   }

   public boolean isValid()
   {
      getResult();
      
      if (collisionFree == false || jointlimitFree == false)
      {
         isValid = false;
      }
      else
      {
         isValid = true;
      }
      
      return isValid;
   }

   public RobotCollisionModel getRobotCollisionModel()
   {
      return robotCollisionModel;
   }
}
