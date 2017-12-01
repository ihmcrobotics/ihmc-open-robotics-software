package us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxConfigurationMessage;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public abstract class WholeBodyPoseValidityTester extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   protected final HumanoidReferenceFrames referenceFrames;
   protected ReferenceFrame midFeetFrame;
   
   private static boolean DEBUG = false;
   
   private final YoDouble solutionQualityThreshold;
   private final YoDouble solutionStableThreshold;
   private final YoDouble jointLimitThreshold;
   private final YoDouble currentSolutionQuality;
   private final YoBoolean isPaused;
   private final YoBoolean isStopped;
   private final YoBoolean isDone;
   private final YoBoolean hasSolverFailed;
   private final YoBoolean hasSentMessageToController;

   private final SideDependentList<SelectionMatrix6D> handSelectionMatrices = new SideDependentList<>(new SelectionMatrix6D(), new SelectionMatrix6D());
   private final SelectionMatrix6D chestSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D pelvisSelectionMatrix = new SelectionMatrix6D();
   private final SideDependentList<YoFramePoint> yoDesiredHandPositions = new SideDependentList<>();
   private final SideDependentList<YoFrameQuaternion> yoDesiredHandOrientations = new SideDependentList<>();
   private final YoFrameQuaternion yoDesiredChestOrientation;
   private final YoFrameQuaternion yoDesiredPelvisOrientation;
   private final YoFramePoint yoDesiredPelvisPosition;

   private final KinematicsToolboxOutputConverter outputConverter;
   protected final FullHumanoidRobotModel fullRobotModel;
   private KinematicsToolboxRigidBodyMessage chestMessage;
   private KinematicsToolboxRigidBodyMessage pelvisMessage;
   private SideDependentList<KinematicsToolboxRigidBodyMessage> handMessages = new SideDependentList<>();   

   private final ConcurrentListeningQueue<KinematicsToolboxOutputStatus> kinematicsToolboxOutputQueue = new ConcurrentListeningQueue<>(15);   
   
   private int numberOfCntForTimeExpire = 50;
   private int cnt = 0;
   private int numberOfCntForJointLimitExpire = 20;
   
   private boolean isSendingPacket = false;   
   private boolean isReceived = false;   
   private boolean isSolved = false;   
   private boolean isJointLimit = false;
   
   private KinematicsToolboxOutputStatus newestSolution;   
   
   protected FullHumanoidRobotModel ikFullRobotModel;
   protected RobotCollisionModel robotCollisionModel;
   
   public static int numberOfTest = 0;   
   
   public WholeBodyPoseValidityTester(FullHumanoidRobotModelFactory fullRobotModelFactory, CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super(null, outgoingCommunicationBridge);
      
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      
      referenceFrames.updateFrames();
      midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
      
      solutionQualityThreshold = new YoDouble(behaviorName + "SolutionQualityThreshold", registry);
      solutionQualityThreshold.set(0.05);
      solutionStableThreshold = new YoDouble(behaviorName + "solutionStableThreshold", registry);
      solutionStableThreshold.set(0.005);
      jointLimitThreshold = new YoDouble(behaviorName + "jointLimitThreshold", registry);
      jointLimitThreshold.set(Math.PI/180 * 2.5);
      isPaused = new YoBoolean(behaviorName + "IsPaused", registry);
      isStopped = new YoBoolean(behaviorName + "IsStopped", registry);
      isDone = new YoBoolean(behaviorName + "IsDone", registry);
      hasSolverFailed = new YoBoolean(behaviorName + "HasSolverFailed", registry);
      hasSentMessageToController = new YoBoolean(behaviorName + "HasSentMessageToController", registry);

      currentSolutionQuality = new YoDouble(behaviorName + "CurrentSolutionQuality", registry);

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
      yoDesiredPelvisPosition = new YoFramePoint(behaviorName + "DesiredPelvis", worldFrame, registry);

      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      attachNetworkListeningQueue(kinematicsToolboxOutputQueue, KinematicsToolboxOutputStatus.class);

      clear();
      
      this.ikFullRobotModel = getFullHumanoidRobotModel();
      this.robotCollisionModel = new RobotCollisionModel(this.ikFullRobotModel); 
   }
   
   public FullHumanoidRobotModel getFullHumanoidRobotModel()
   {            
      return outputConverter.getFullRobotModel();
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
   
   public double getSolutionQuality()
   {
      return currentSolutionQuality.getDoubleValue();
   }
   
   public boolean hasSolverFailed()
   {
      return hasSolverFailed.getBooleanValue();
   }
   
   private void deactivateKinematicsToolboxModule()
   {
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.SLEEP);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);      
      sendPacket(message);
   }
    
   // Hand ---------------------------------------------------
   public void setDesiredHandPose(RobotSide robotSide, FramePose desiredHandPose)
   {
      FramePoint3D desiredPosition = new FramePoint3D();
      FrameQuaternion desiredOrientation = new FrameQuaternion();
      
      desiredHandPose.getPoseIncludingFrame(desiredPosition, desiredOrientation);
      setDesiredHandPose(robotSide, desiredPosition, desiredOrientation);
   }

   public void setDesiredHandPose(RobotSide robotSide, FramePoint3D desiredHandPosition, FrameQuaternion desiredHandOrientation)
   {
      yoDesiredHandPositions.get(robotSide).setAndMatchFrame(desiredHandPosition);
      yoDesiredHandOrientations.get(robotSide).setAndMatchFrame(desiredHandOrientation);
   }
   
   public void setHandLinearControlOnly(RobotSide robotSide)
   {
      handSelectionMatrices.get(robotSide).setToLinearSelectionOnly();
   }

   // Chest ---------------------------------------------------
   public void holdCurrentChestOrientation()
   {
      FrameQuaternion currentChestOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      //FrameOrientation currentChestOrientation = new FrameOrientation(referenceFrames.getChestFrame());
      yoDesiredChestOrientation.setAndMatchFrame(currentChestOrientation);
      chestSelectionMatrix.setToAngularSelectionOnly();      
   }

   public void setDesiredChestOrientation(FrameQuaternion desiredChestOrientation)
   {
      yoDesiredChestOrientation.setAndMatchFrame(desiredChestOrientation);
   }

   public void setChestAngularControl(boolean roll, boolean pitch, boolean yaw)
   {
      chestSelectionMatrix.setAngularAxisSelection(roll, pitch, yaw);
      chestSelectionMatrix.clearLinearSelection();      
   }
   
   public void holdCurrentPelvisOrientation()
   {      
      FrameQuaternion currentPelvisOrientation = new FrameQuaternion(fullRobotModel.getPelvis().getBodyFixedFrame());
      //FrameOrientation currentPelvisOrientation = new FrameOrientation(referenceFrames.getPelvisFrame());      
      yoDesiredPelvisOrientation.setAndMatchFrame(currentPelvisOrientation);
      pelvisSelectionMatrix.setToAngularSelectionOnly();      
   }

   public void setDesiredPelvisOrientation(FrameQuaternion desiredPelvisOrientation)
   {
      yoDesiredPelvisOrientation.setAndMatchFrame(desiredPelvisOrientation);
   }

   public void setPelvisAngularControl(boolean roll, boolean pitch, boolean yaw)
   {
      pelvisSelectionMatrix.setAngularAxisSelection(roll, pitch, yaw);
      pelvisSelectionMatrix.clearLinearSelection();
   }

   // Pelvis ---------------------------------------------------
   public void holdCurrentPelvisHeight()
   {      
      yoDesiredPelvisPosition.setFromReferenceFrame(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      //yoDesiredPelvisPosition.setFromReferenceFrame(referenceFrames.getPelvisFrame());
      pelvisSelectionMatrix.clearLinearSelection();
      pelvisSelectionMatrix.selectLinearZ(true);
      pelvisSelectionMatrix.setSelectionFrame(worldFrame);
   }

   public void setDesiredPelvisHeight(FramePoint3D pointContainingDesiredHeight)
   {      
      yoDesiredPelvisPosition.setAndMatchFrame(pointContainingDesiredHeight);
      pelvisSelectionMatrix.clearLinearSelection();
      pelvisSelectionMatrix.selectLinearZ(true);
      pelvisSelectionMatrix.setSelectionFrame(worldFrame);
   }

   public void setDesiredPelvisHeight(double desiredHeightInWorld)
   {      
      yoDesiredPelvisPosition.setX(1.0);
      yoDesiredPelvisPosition.setY(1.0);
      yoDesiredPelvisPosition.setZ(desiredHeightInWorld);
      pelvisSelectionMatrix.clearLinearSelection();
      pelvisSelectionMatrix.selectLinearZ(true);
      pelvisSelectionMatrix.setSelectionFrame(worldFrame);
   }
   

      
   public void setUpHasBeenDone()
   {  
      putHandMessage();
      putChestMessage();
      putPelvisMessage();
      
      isSolved = false;
      isJointLimit = false;
      isSendingPacket = true;
   }
   
   private void putHandMessage()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFramePoint yoDesiredHandPosition = yoDesiredHandPositions.get(robotSide);
         YoFrameQuaternion yoDesiredHandOrientation = yoDesiredHandOrientations.get(robotSide);

         if (yoDesiredHandPosition.containsNaN() || yoDesiredHandOrientation.containsNaN())
         {
            handMessages.put(robotSide, null);
         }
         else
         {
            Point3D desiredHandPosition = new Point3D();
            Quaternion desiredHandOrientation = new Quaternion();
            yoDesiredHandPosition.get(desiredHandPosition);
            yoDesiredHandOrientation.get(desiredHandOrientation);
            RigidBody hand = fullRobotModel.getHand(robotSide);
            ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
            KinematicsToolboxRigidBodyMessage handMessage = new KinematicsToolboxRigidBodyMessage(hand, handControlFrame, desiredHandPosition,
                                                                                                  desiredHandOrientation);            
            handMessage.setWeight(5.0);
            handMessages.put(robotSide, handMessage);
         }
      }
   }
   
   private void putChestMessage()
   {
      if (yoDesiredChestOrientation.containsNaN())
      {
         chestMessage = null;
      }
      else
      {
         Quaternion desiredChestOrientation = new Quaternion();
         yoDesiredChestOrientation.get(desiredChestOrientation);
         RigidBody chest = fullRobotModel.getChest();
         chestMessage = new KinematicsToolboxRigidBodyMessage(chest, desiredChestOrientation);
         chestMessage.setWeight(10.00);
      }
   }
   
   private void putPelvisMessage()
   {
      RigidBody pelvis = fullRobotModel.getPelvis();

      if (yoDesiredPelvisOrientation.containsNaN() && yoDesiredPelvisPosition.containsNaN())
         pelvisMessage = null;
      else
         pelvisMessage = new KinematicsToolboxRigidBodyMessage(pelvis);

      if (!yoDesiredPelvisOrientation.containsNaN())
      {
         Quaternion desiredPelvisOrientation = new Quaternion();
         yoDesiredPelvisOrientation.get(desiredPelvisOrientation);
         pelvisMessage.setDesiredOrientation(desiredPelvisOrientation);
         pelvisMessage.setWeight(0.02);
      }

      if (!yoDesiredPelvisPosition.containsNaN())
      {
         Point3D desiredPelvisPosition = new Point3D();
         yoDesiredPelvisPosition.get(desiredPelvisPosition);
         pelvisMessage.setDesiredPosition(desiredPelvisPosition);
         pelvisMessage.setWeight(10.0);
      }
      Point3D desiredPelvisPosition = new Point3D();
      yoDesiredPelvisPosition.get(desiredPelvisPosition);
   }
   
   
   
   @Override
   public void doControl()
   {  
      if (kinematicsToolboxOutputQueue.isNewPacketAvailable())
      {
         if(isSendingPacket == true)
         {               
            for (RobotSide robotSide : RobotSide.values)
            {
               if (handMessages.get(robotSide) != null)
               {
                  handMessages.get(robotSide).setSelectionMatrix(handSelectionMatrices.get(robotSide));
                  handMessages.get(robotSide).setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
                  sendPacket(handMessages.get(robotSide));                  
               }
            }
            FramePose desiredPoseToPack = new FramePose();
            handMessages.get(RobotSide.RIGHT).getDesiredPose(desiredPoseToPack);

            if (chestMessage != null)
            {
               chestMessage.setSelectionMatrix(chestSelectionMatrix);
               chestMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
               sendPacket(chestMessage);
            }

            if (pelvisMessage != null)
            {
               pelvisMessage.setSelectionMatrix(pelvisSelectionMatrix);
               pelvisMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
               sendPacket(pelvisMessage);
            }
            
            newestSolution = kinematicsToolboxOutputQueue.poll();
            
            double deltaSolutionQuality = currentSolutionQuality.getDoubleValue() - newestSolution.getSolutionQuality();
            boolean isSolutionStable = (Math.abs(deltaSolutionQuality) < solutionStableThreshold.getDoubleValue());
            boolean isSolutionGoodEnough = newestSolution.getSolutionQuality() < solutionQualityThreshold.getDoubleValue();
            boolean isGoodSolutionCur = isSolutionStable && isSolutionGoodEnough;
            
            if(isReceived == false)
            {
               isReceived = deltaSolutionQuality < -0.1;               
            }
            
            // break condition 1. solved
            if(isReceived == true)
            {
               if(isGoodSolutionCur == true)
               {
                  isSolved = true;
               }
            }
                        
            // break condition 2. joint limit            
            isJointLimit = isJointLimit();
            
            outputConverter.updateFullRobotModel(newestSolution);
            
            currentSolutionQuality.set(newestSolution.getSolutionQuality());
            
            cnt++;
            if(false)
               PrintTools.info(""+cnt+" SQ "+ newestSolution.getSolutionQuality() + " dSQ " + deltaSolutionQuality
                               +" isReceived "+isReceived +" isGoodSolutionCur "+isGoodSolutionCur +" isSolved "+isSolved);
            
            if(isSolved == true || isJointLimit == true || cnt == numberOfCntForTimeExpire)
            {               
               cntOfSeries = 0;
               if(isSolved == true)
               {
                  if(DEBUG)
                     PrintTools.info("isSolved isSolved isSolved");
                  isGoodIKSolution = true;
               }
                  
               if(isJointLimit == true)
               {
                  if(DEBUG)
                     PrintTools.info("isJointLimit "+isJointLimit+" "+getFullHumanoidRobotModel().getOneDoFJoints()[indexOfLimit].getName());
                  isGoodIKSolution = false;
               }
               
               if(cnt == numberOfCntForTimeExpire)
               {
                  if(DEBUG)
                     PrintTools.info("Time is over ");
                  isGoodIKSolution = false;
               }
                  
               
               isReceived = false;               
               isSendingPacket = false;
                               
               setIsDone(true);
            }
         }
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      numberOfTest++;
      
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSentMessageToController.set(false);
      hasSolverFailed.set(false);
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(message);
            
      KinematicsToolboxConfigurationMessage privilegedMessage = new KinematicsToolboxConfigurationMessage();
      fullRobotModel.updateFrames();
      OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      long[] jointNameBasedHashCodes = new long[oneDoFJoints.length];
      float[] privilegedJointAngles = new float[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointNameBasedHashCodes[i] = oneDoFJoints[i].getNameBasedHashCode();
         privilegedJointAngles[i] = (float) oneDoFJoints[i].getQ();
//         PrintTools.info(""+oneDoFJoints[i].getName()+" "+oneDoFJoints[i].getQ());
      }

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      Point3D privilegedRootJointPosition = new Point3D();
      rootJoint.getTranslation(privilegedRootJointPosition);
      Quaternion privilegedRootJointOrientation = new Quaternion();
      rootJoint.getRotation(privilegedRootJointOrientation);
      
      MovingReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      
      FramePoint3D pelvisPosition = new FramePoint3D();
      pelvisPosition.setToZero(pelvisFrame);
      pelvisPosition.changeFrame(referenceFrames.getMidFootZUpGroundFrame());
//      System.out.println(pelvisPosition);
      
//      PrintTools.info(""+ privilegedRootJointPosition.getX()+" "+privilegedRootJointPosition.getY()+" "+privilegedRootJointPosition.getZ());

      privilegedMessage.setPrivilegedRobotConfiguration(privilegedRootJointPosition, privilegedRootJointOrientation, jointNameBasedHashCodes, privilegedJointAngles);
      privilegedMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      
      sendPacket(privilegedMessage);

//      message = new ToolboxStateMessage(ToolboxState.REINITIALIZE);
//      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
//      sendPacket(message);
   }
      
   static int cntOfSeries = 0;
   int indexOfLimit = 0;
   private boolean isJointLimit()
   {      
      boolean isLimit = false;
      int numberOfJoints = getFullHumanoidRobotModel().getOneDoFJoints().length;
      
      int rightArmStart = 12;
      int rightArmEnd = 18;
            
      for(int i=0;i<numberOfJoints;i++)   
      {              
         if(i<3 || (i>=rightArmStart && i<=rightArmEnd))
         {
            OneDoFJoint aJoint = getFullHumanoidRobotModel().getOneDoFJoints()[i];
            double aJointValue = newestSolution.getJointAngles()[i];
            double upperValue = aJoint.getJointLimitUpper();
            double lowerValue = aJoint.getJointLimitLower();               
            if(Math.abs(lowerValue - aJointValue) < jointLimitThreshold.getDoubleValue() || Math.abs(upperValue - aJointValue) < jointLimitThreshold.getDoubleValue())
            {
               indexOfLimit = i;
               isLimit = true;
               break;
            }   
         }
      }      
      
      
      if(isLimit == true)
         cntOfSeries++;
      else
         cntOfSeries = 0;
      
      
      if(cntOfSeries > numberOfCntForJointLimitExpire)
      {         
         return true;
      }
         
      else
         return false;
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

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSolverFailed.set(false);
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

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }
   
   public void setIsDone(boolean value)
   {
      cnt = 0;
      isDone.set(value);
   }
   
   
   

   protected boolean isValid = true;
   protected boolean isCollisionFree = true;
   protected boolean isGoodIKSolution = true;
   
   private void getCollisionResult()
   {
      robotCollisionModel = new RobotCollisionModel(getFullHumanoidRobotModel());
      addEnvironmentCollisionModel();
      
      robotCollisionModel.update();
      isCollisionFree = robotCollisionModel.getCollisionResult();
   }
   
   public double getScroe()
   {
      double score = 0;
      
      int numberOfJoints = getFullHumanoidRobotModel().getOneDoFJoints().length;
      
      int rightArmStart = 12;
      int rightArmEnd = 18;
            
      for(int i=0;i<numberOfJoints;i++)   
      {              
         if(i<3 || (i>=rightArmStart && i<=rightArmEnd))
         {
            OneDoFJoint aJoint = getFullHumanoidRobotModel().getOneDoFJoints()[i];
            double aJointValue = newestSolution.getJointAngles()[i];
            double upperValue = aJoint.getJointLimitUpper();
            double lowerValue = aJoint.getJointLimitLower();               
            
            double diffUpper = upperValue - aJointValue;
            double diffLower = aJointValue - lowerValue;
            
            if(diffUpper > diffLower)
               score = diffLower;
            else
               score = diffUpper;
            
            if(DEBUG)
               PrintTools.info(""+score +" "+ aJointValue+" "+ upperValue +" "+ lowerValue);
         }
      } 
      if(DEBUG)
         PrintTools.info("");      
      
      return score;
   }

   public boolean isValid()
   {
      getCollisionResult();
            
      if (isCollisionFree == false || isGoodIKSolution == false)
      {
         if(false)
         {
            if(isCollisionFree == false)
               PrintTools.warn("col");
            if(isGoodIKSolution == false)
               PrintTools.warn("ik");   
         }
         
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
   
   
   public abstract void addEnvironmentCollisionModel();

}
