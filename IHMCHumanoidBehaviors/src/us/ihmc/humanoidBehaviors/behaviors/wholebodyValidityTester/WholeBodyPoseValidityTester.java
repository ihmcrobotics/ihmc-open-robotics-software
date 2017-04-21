package us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTNode1DTimeDomain;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
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
import us.ihmc.tools.thread.ThreadTools;

public abstract class WholeBodyPoseValidityTester extends AbstractBehavior
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
   private final DenseMatrix64F chestSelectionMatrix = initializeAngularSelctionMatrix();
   private final DenseMatrix64F pelvisSelectionMatrix = initializeAngularSelctionMatrix();
   private final SideDependentList<YoFramePoint> yoDesiredHandPositions = new SideDependentList<>();
   private final SideDependentList<YoFrameQuaternion> yoDesiredHandOrientations = new SideDependentList<>();
   private final YoFrameQuaternion yoDesiredChestOrientation;
   private final YoFrameQuaternion yoDesiredPelvisOrientation;
   private final DoubleYoVariable yoDesiredPelvisHeight;

   private final KinematicsToolboxOutputConverter outputConverter;
   private final FullHumanoidRobotModel fullRobotModel;
   private ChestTrajectoryMessage chestTrajectoryMessage;
   private PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage;
   private PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage;
   private SideDependentList<HandTrajectoryMessage> handTrajectoryMessage = new SideDependentList<>();   

   private final ConcurrentListeningQueue<KinematicsToolboxOutputStatus> kinematicsToolboxOutputQueue = new ConcurrentListeningQueue<>(10);   
   
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame chestFrame;

   
   private static boolean Debug = false;
   
   protected FullHumanoidRobotModel ikFullRobotModel;
   protected RobotCollisionModel robotCollisionModel;
   
   public WholeBodyPoseValidityTester(FullHumanoidRobotModelFactory fullRobotModelFactory, CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel)
   {
      super(null, outgoingCommunicationBridge);
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
      
      this.ikFullRobotModel = getFullHumanoidRobotModel();

      this.robotCollisionModel = new RobotCollisionModel(this.ikFullRobotModel); 
   }
   
   public FullHumanoidRobotModel getFullHumanoidRobotModel()
   {            
      return outputConverter.getFullRobotModel();
   }
   
   private DenseMatrix64F initializeAngularSelctionMatrix()
   {
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 6);
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
      return selectionMatrix;
   }

   public void clear()
   {
      currentSolutionQuality.set(Double.POSITIVE_INFINITY);

      yoDesiredChestOrientation.setToNaN();
      yoDesiredPelvisOrientation.setToNaN();
      yoDesiredPelvisHeight.setToNaN();

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
   
   // message set ****************************************************************
   public void setDesiredHandPose(RobotSide desiredRobotSide, FramePose desiredHandPose)
   {
      FramePoint desiredPosition = new FramePoint();
      FrameOrientation desiredOrientation = new FrameOrientation();
      
      desiredHandPose.getPoseIncludingFrame(desiredPosition, desiredOrientation);
      
      yoDesiredHandPositions.get(desiredRobotSide).setAndMatchFrame(desiredPosition);
      yoDesiredHandOrientations.get(desiredRobotSide).setAndMatchFrame(desiredOrientation);
      
      setDesiredHandPose();
   }
   
   private void setDesiredHandPose()
   {
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
            Point3D desiredHandPosition = new Point3D();
            Quaternion desiredHandOrientation = new Quaternion();
            yoDesiredHandPosition.get(desiredHandPosition);
            yoDesiredHandOrientation.get(desiredHandOrientation);
            HandTrajectoryMessage temporaryHandTrajectoryMessage = new HandTrajectoryMessage(robotSide, 0.0, desiredHandPosition, desiredHandOrientation, worldFrame, chestFrame);
            handTrajectoryMessage.put(robotSide, temporaryHandTrajectoryMessage);
         }
      } 
   }
   
   public void setDesiredChestOrientation(FrameOrientation desired)
   {
      yoDesiredChestOrientation.setAndMatchFrame(desired);
      setDesiredChestOrientation();
   }
   
   private void setDesiredChestOrientation()
   {
      if (yoDesiredChestOrientation.containsNaN())
      {
         chestTrajectoryMessage = null;
      }
      else
      {
         YoFrameQuaternion yoDesiredChestQuaternion = yoDesiredChestOrientation;
         Quaternion desiredChestOrientation = new Quaternion();
         yoDesiredChestQuaternion.get(desiredChestOrientation);
         if(Debug)
            PrintTools.info("Chest "+ desiredChestOrientation.getAngle()+" "+desiredChestOrientation.getX() +" "+ desiredChestOrientation.getY()+" "+desiredChestOrientation.getZ());
         chestTrajectoryMessage = new ChestTrajectoryMessage(0.0, desiredChestOrientation, worldFrame, pelvisZUpFrame);
      }
   }

   public void setDesiredPelvisHeight(double desired)
   {
      yoDesiredPelvisHeight.set(desired); 
      setDesiredPelvisHeight();
   }
   
   private void setDesiredPelvisHeight()
   {
      if (yoDesiredPelvisHeight.isNaN())
      {
         pelvisHeightTrajectoryMessage = null;
      }
      else
      {
         if(Debug)
            PrintTools.info("Pelvis "+ yoDesiredPelvisHeight.getDoubleValue());
         pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(0.0, yoDesiredPelvisHeight.getDoubleValue());
      }
   }
   
   public void setDesiredPelvisOrientation(FrameOrientation desired)
   {
      yoDesiredPelvisOrientation.setAndMatchFrame(desired);      
      setDesiredPelvisOrientation();
   }
   
   private void setDesiredPelvisOrientation()
   {
      if (yoDesiredPelvisOrientation.containsNaN())
      {
         pelvisOrientationTrajectoryMessage = null;
      }
      else
      {
         YoFrameQuaternion yoDesiredPelvisQuaternion = yoDesiredPelvisOrientation;
         Quaternion desiredPelvisOrientation = new Quaternion();
         yoDesiredPelvisQuaternion.get(desiredPelvisOrientation);
         if(Debug)
            PrintTools.info("Pelvis "+ desiredPelvisOrientation.getAngle()+" "+desiredPelvisOrientation.getX() +" "+ desiredPelvisOrientation.getY()+" "+desiredPelvisOrientation.getZ());
         pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(0.0, desiredPelvisOrientation);
      }
   }
   
   public void holdCurrentChestOrientation()
   {
      FrameOrientation currentChestOrientation = new FrameOrientation(fullRobotModel.getChest().getBodyFixedFrame());
      yoDesiredChestOrientation.setAndMatchFrame(currentChestOrientation);
      setDesiredChestOrientation();
   }
   
   public void holdCurrentPelvisHeight()
   {
      FramePoint currentPelvisPosition = new FramePoint(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      currentPelvisPosition.changeFrame(worldFrame);
      yoDesiredPelvisHeight.set(currentPelvisPosition.getZ());
      setDesiredPelvisHeight();
   }
   
   public void holdCurrentPelvisOrientation()
   {
      FrameOrientation currentPelvisOrientation = new FrameOrientation(fullRobotModel.getPelvis().getBodyFixedFrame());
      yoDesiredPelvisOrientation.setAndMatchFrame(currentPelvisOrientation);
      setDesiredPelvisOrientation();
   }
      
   private int cnt = 0;
   
   private boolean isSendingPacket = false;
   private boolean forceOut = false;
   
   private boolean isReceived = false;
   private boolean isSolved = false;
   
   //private boolean wasStable = false;
   
   public void setUpHasBeenDone()
   {  
      isSolved = false;
      isSendingPacket = true;
      
      ToolboxStateMessage message = new ToolboxStateMessage(ToolboxState.REINITIALIZE);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      sendPacket(message);
   }
   
   public boolean getIKResult()
   {
      PrintTools.info("Try to get ik result");
      setUpHasBeenDone();
      ThreadTools.sleep(3000);
      for(int i=0;i<100;i++)
      {
         //PrintTools.info("SQ "+ i +" "+currentSolutionQuality);
         ThreadTools.sleep(5);
         if(isSolved == true)
         {
            //PrintTools.info("SQ "+ currentSolutionQuality);
            if(Debug)
               PrintTools.info("Solution Get "+i);
            return true;
         }
      }
      if(Debug)
         PrintTools.info("No solution ");
      
      PrintTools.info("Solution "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM03()
                      +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM13()
                      +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM23());
      
      forceOut();
      ThreadTools.sleep(10);
      
      return false;
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
               if (handTrajectoryMessage.get(robotSide) != null)
               {
                  handTrajectoryMessage.get(robotSide).setSelectionMatrix(handSelectionMatrices.get(robotSide));
                  handTrajectoryMessage.get(robotSide).setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
                  sendPacket(handTrajectoryMessage.get(robotSide));
               }
            }

            if (chestTrajectoryMessage != null)
            {
               chestTrajectoryMessage.setSelectionMatrix(chestSelectionMatrix);
               chestTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
               sendPacket(chestTrajectoryMessage);
            }

            if (pelvisOrientationTrajectoryMessage != null)
            {
               pelvisOrientationTrajectoryMessage.setSelectionMatrix(pelvisSelectionMatrix);
               pelvisOrientationTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
               sendPacket(pelvisOrientationTrajectoryMessage);
            }

            if (pelvisHeightTrajectoryMessage != null)
            {
               pelvisHeightTrajectoryMessage.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);
               sendPacket(pelvisHeightTrajectoryMessage);
            }
           
            
            KinematicsToolboxOutputStatus newestSolution = kinematicsToolboxOutputQueue.poll();
            
            double deltaSolutionQuality = currentSolutionQuality.getDoubleValue() - newestSolution.getSolutionQuality();
            boolean isSolutionStable = Math.abs(deltaSolutionQuality) < 0.001;
            boolean isSolutionGoodEnough = newestSolution.getSolutionQuality() < solutionQualityThreshold.getDoubleValue();
            boolean isGoodSolutionCur = isSolutionStable && isSolutionGoodEnough;
            
            if(isReceived == false)
            {
               isReceived = deltaSolutionQuality < -0.005;               
            }
            
            
            if(isReceived == true)
            {
               if(isGoodSolutionCur == true)
               {
                  isSolved = true;
               }
            }
            
            
            outputConverter.updateFullRobotModel(newestSolution);
            
            
            
            currentSolutionQuality.set(newestSolution.getSolutionQuality());
            
            cnt++;
            if(false)
               PrintTools.info(""+cnt+" SQ "+ newestSolution.getSolutionQuality() + " dSQ " + deltaSolutionQuality);
            if(true)
               PrintTools.info(""+cnt+" isSolutionStable "+isSolutionStable+" isSolutionGoodEnough "+isSolutionGoodEnough
                               +" isReceived "+isReceived
                               +" isGoodSolutionCur "+isGoodSolutionCur                            
                               +" isSolved "+isSolved);

//            PrintTools.info("Solution "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM03()
//                            +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM13()
//                            +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM23());
            
            if(isSolved == true || forceOut == true)
            {
               if(true)
                  PrintTools.info(""+cnt+" SQ "+ newestSolution.getSolutionQuality());
               if(true)
                  PrintTools.info("Solution "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM03()
                               +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM13()
                               +" "+outputConverter.getFullRobotModel().getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame().getM23());
               isReceived = false;               
               isSendingPacket = false;
               forceOut = false;
               
            }
         }
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      PrintTools.info("Validity Tester start! ");      
      isPaused.set(false);
      isStopped.set(false);
      isDone.set(false);
      hasSentMessageToController.set(false);
      hasSolverFailed.set(false);
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
            Point3D desiredHandPosition = new Point3D();
            Quaternion desiredHandOrientation = new Quaternion();
            yoDesiredHandPosition.get(desiredHandPosition);
            yoDesiredHandOrientation.get(desiredHandOrientation);
            HandTrajectoryMessage temporaryHandTrajectoryMessage = new HandTrajectoryMessage(robotSide, 0.0, desiredHandPosition, desiredHandOrientation, worldFrame, chestFrame);
            handTrajectoryMessage.put(robotSide, temporaryHandTrajectoryMessage);
         }
      }

      if (yoDesiredChestOrientation.containsNaN())
      {
         chestTrajectoryMessage = null;
      }
      else
      {
         YoFrameQuaternion yoDesiredChestQuaternion = yoDesiredChestOrientation;
         Quaternion desiredChestOrientation = new Quaternion();
         yoDesiredChestQuaternion.get(desiredChestOrientation);
         chestTrajectoryMessage = new ChestTrajectoryMessage(0.0, desiredChestOrientation, worldFrame, pelvisZUpFrame);
      }

      if (yoDesiredPelvisOrientation.containsNaN())
      {
         pelvisOrientationTrajectoryMessage = null;
      }
      else
      {
         YoFrameQuaternion yoDesiredPelvisQuaternion = yoDesiredPelvisOrientation;
         Quaternion desiredPelvisOrientation = new Quaternion();
         yoDesiredPelvisQuaternion.get(desiredPelvisOrientation);
         pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(0.0, desiredPelvisOrientation);
      }

      if (yoDesiredPelvisHeight.isNaN())
      {
         pelvisHeightTrajectoryMessage = null;
      }
      else
      {
         pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(0.0, yoDesiredPelvisHeight.getDoubleValue());
      }
      
   }
   
   public void forceOut()
   {
      if(Debug)
         PrintTools.info("forceout ");      
      forceOut = true;
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
      chestTrajectoryMessage = null;
      pelvisOrientationTrajectoryMessage = null;
      pelvisHeightTrajectoryMessage = null;

      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryMessage.put(robotSide, null);
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
      isDone.set(value);
   }
   
   
   

   public boolean isValid = true;
   protected boolean isCollisionFree = true;
   protected boolean isGoodIKSolution = true;
   
   private void getCollisionResult()
   {
      // collision tester
      robotCollisionModel = new RobotCollisionModel(getFullHumanoidRobotModel());
      addEnvironmentCollisionModel();
      
      robotCollisionModel.update();
      isCollisionFree = robotCollisionModel.getCollisionResult();

//      // joint limit tester
//      /*
//       * joint limit should be added. 170401
//       */
//      isGoodIKSolution = true;
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
