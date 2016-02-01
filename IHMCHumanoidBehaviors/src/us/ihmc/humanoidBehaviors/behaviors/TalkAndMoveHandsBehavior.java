package us.ihmc.humanoidBehaviors.behaviors;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;

import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.taskExecutor.PipeLine;

public class TalkAndMoveHandsBehavior extends BehaviorInterface implements VariableChangedListener
{
   private enum HandGestures {NONE, FIST_RAGE, HEIGHT_INDICATION, NUMBER_ONE, KNIFE_HAND, ME, NUMBERING_HAND_GESTURE};
   private final EnumYoVariable<HandGestures> currentHandGesture = new EnumYoVariable<TalkAndMoveHandsBehavior.HandGestures>("currentHandGesture", registry, HandGestures.class);
   private EnumYoVariable<RobotSide> selectedRobotSide = new EnumYoVariable<RobotSide>("selectedRobotSide", registry, RobotSide.class);
   private final DoubleYoVariable yoTime;
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   private final HandPoseBehavior handPoseBehavior;
   private final HandPoseListBehavior handPoseListBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final SideDependentList<FramePose> handTargets = new SideDependentList<FramePose>();

   private final HumanoidReferenceFrames referenceFrames;
   private final ReferenceFrame chestFrame;
   private final SideDependentList<ReferenceFrame> handFrames = new SideDependentList<ReferenceFrame>();
   //   private final ConcurrentListeningQueue<VideoPacket> inputListeningQueue = new ConcurrentListeningQueue<VideoPacket>();

   private final Point3d leftHandFistRagePosition = new Point3d(0.3, -0.15, 0.0);
   private final Point3d rightHandFistRagePosition = new Point3d(0.3, 0.15, 0.0);
   private final SideDependentList<Point3d> fistHandPosition = new SideDependentList<Point3d>(leftHandFistRagePosition, rightHandFistRagePosition);
   private IntegerYoVariable numberOfFingers = new IntegerYoVariable("numberOfFingers", registry);

   public TalkAndMoveHandsBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames,
         DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;
      this.referenceFrames = referenceFrames;
      chestFrame = referenceFrames.getChestFrame();

      for (RobotSide side : RobotSide.values)
      {
         handFrames.put(side, referenceFrames.getHandFrame(side));
         handTargets.put(side, new FramePose(ReferenceFrame.getWorldFrame()));
      }
      
      currentHandGesture.addVariableChangedListener(this);

      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      handPoseListBehavior = new HandPoseListBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      //		this.attachNetworkProcessorListeningQueue(inputListeningQueue, VideoPacket.class);
   }

   @Override
   public void doControl()
   {
//      if (!isPaused())
      {
//         if (!pipeLine.isDone())
         {
            pipeLine.doControl();
         }
      }
   }

   private void performBackHandSlap()
   {

   }

   private void performFistRage(RobotSide side)
   {
      FramePose handPose = handTargets.get(side);
      handPose.setToZero(chestFrame);
      handPose.setPosition(fistHandPosition.get(side));
      handPose.setOrientation(0.0, Math.PI / 2.0, 0.0);
      handPose.changeFrame(ReferenceFrame.getWorldFrame());
      
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side, 1.0, handPose, Frame.CHEST, handPoseBehavior, yoTime));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.CRUSH, fingerStateBehavior, yoTime));
   }

   private void performFistPump(RobotSide side)
   {
      //      handPoseListBehavior.setInput(new HandPoseListPacket(side, referenceFrame, dataType, positions, orientations, toHomePosition, trajectoryTime, jointAngles));
      //      pipeLine.submitTaskForPallelPipesStage(handPoseListBehavior, handPoseBehavior);
   }

   //flat palm
   private void performHeightIndication(RobotSide side, double heightFromChestFrame)
   {
      FramePose handPose = handTargets.get(side);
      handPose.setToZero(chestFrame);

      handPose.setPosition(0.1, side.negateIfLeftSide(0.25), 0.0);
      handPose.setOrientation(side.negateIfLeftSide(Math.PI / 2.0), 0.0, 0.0);
      handPose.changeFrame(ReferenceFrame.getWorldFrame());
      
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side, 1.0, handPose, Frame.CHEST, handPoseBehavior, yoTime));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.HALF_CLOSE, fingerStateBehavior, yoTime));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.CLOSE_THUMB, fingerStateBehavior, yoTime));

   }

   //being finger tips together then apart 
   private void performTapFingersTogethorLikeMrBurns()
   {

   }

   //wag the pointer finger
   private void performWagFinger(RobotSide side)
   {

   }

   //like your drunk and going to hug someone, they say ehh, and you howl come onnnnnn
   private void performGrandGesture()
   {

   }

   //point one finger up
   private void performNumberOne(RobotSide side)
   {
      performNumberingHandGesture(side, 1);
   }

   //cut through the air, need several fingers
   private void performKnifeHand(RobotSide side)
   {
      FramePose handPose = handTargets.get(side);
      handPose.setToZero(chestFrame);

      handPose.setPosition(0.25, side.negateIfLeftSide(0.25), 0.0);
      handPose.changeFrame(ReferenceFrame.getWorldFrame());
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side, 1.0, handPose, Frame.CHEST, handPoseBehavior, yoTime));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.PINCH_GRIP, fingerStateBehavior, yoTime));
   }

   //hands in towards chest
   private void performMe()
   {
      System.out.println("ME!");
      for(RobotSide side : RobotSide.values)
      {
         FramePose chestPose = new FramePose(chestFrame);
         FramePose handPose = handTargets.get(side);
         handPose.setToZero(chestFrame);

         handPose.setPosition(0.25, side.negateIfLeftSide(0.25), 0.0);
         AxisAngle4d pointAtChestRotation = new AxisAngle4d();
         handPose.getAxisAngleRotationToOtherPose(chestPose, pointAtChestRotation);
         handPose.setOrientation(pointAtChestRotation);
         handPose.changeFrame(ReferenceFrame.getWorldFrame());
         pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side, 1.0, handPose, Frame.CHEST, handPoseBehavior, yoTime));
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.PINCH_GRIP, fingerStateBehavior, yoTime));
         System.out.println("running in parallel!");
      }
   }

   //palms up and hand infront of the other, like holding the earth at your chest and the moon far away, will use random yaw 
   private void performThisAndThat(double distanceFromYourChest, double distanceApartFromEachother)
   {

   }

   //startWithHandsApartAndCloseGap
   private void performComeTogethor(double distanceApartAtStart, double distanceApartAtEnd, double time)
   {

   }

   private void performNumberingHandGesture(RobotSide side, int numberOfFingers)
   {
      FramePose handPose = handTargets.get(side);
      handPose.setToZero(chestFrame);
      handPose.setPosition(fistHandPosition.get(side));
      handPose.setOrientation(0.0, Math.PI / 2.0, 0.0);
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side, 1.0, handPose, Frame.CHEST, handPoseBehavior, yoTime));

      switch(numberOfFingers)
      {
      case 3:
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.OPEN, fingerStateBehavior, yoTime));
         break;
         
      case 2: 
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.OPEN_INDEX, fingerStateBehavior, yoTime));
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.OPEN_MIDDLE, fingerStateBehavior, yoTime));
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.CLOSE_THUMB, fingerStateBehavior, yoTime));
         break;
      
      case 1: 
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.OPEN_INDEX, fingerStateBehavior, yoTime));
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.CRUSH_MIDDLE, fingerStateBehavior, yoTime));
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.CLOSE_THUMB, fingerStateBehavior, yoTime));

      default:
         pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.CLOSE, fingerStateBehavior, yoTime));
      }
   }

   // hands are at a 45 degree angle with the palms facing up, 
   private void performNothingToHide()
   {

   }

   private void performScaleGesture(RobotSide side, double distanceBetweenFingers)
   {

   }

   private void performOpenArms()
   {

   }

   private void constricHandGestureToJazzHandsSpectrum()
   {

   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }

   @Override
   public void stop()
   {
      defaultStop();
   }

   @Override
   public void enableActions()
   {

   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public void resume()
   {
      defaultResume();
   }

   @Override
   public boolean isDone()
   {
      return defaultIsDone();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return false;
   }

   @Override
   public void initialize()
   {
      defaultPostBehaviorCleanup();
   }

   @Override
   public void variableChanged(YoVariable<?> v)
   {
      pipeLine.clearAll();
      RobotSide robotSide = selectedRobotSide.getEnumValue();
      switch(currentHandGesture.getEnumValue())
      {
      case FIST_RAGE:
         performFistRage(robotSide);
         break;
      case HEIGHT_INDICATION:
         performHeightIndication(robotSide, -0.1);
         break;
      case KNIFE_HAND:
         performKnifeHand(robotSide);
         break;
      case ME:
         performMe();
         break;
      case NUMBER_ONE:
         performNumberOne(robotSide);
         break;
      case NUMBERING_HAND_GESTURE:
         performNumberingHandGesture(robotSide, numberOfFingers.getIntegerValue());
         break;
      default:
         break;
      }
   }
}
