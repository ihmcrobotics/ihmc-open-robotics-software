package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.ChestOrientationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisPoseTask;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;

public class DropDebrisBehavior extends BehaviorInterface
{
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   private final PelvisPoseBehavior pelvisPoseBehavior;
   private final ChestOrientationBehavior chestOrientationBehavior;

   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame midFeetZUpFrame;

   private double trajectoryTime = 2.5;

   private final WholeBodyControllerParameters wholeBodyControllerParameters;
   private final DoubleYoVariable yoTime;

   public DropDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames,
         WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      this.yoTime = yoTime;

      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      pelvisPoseBehavior = new PelvisPoseBehavior(outgoingCommunicationBridge, yoTime);
      chestOrientationBehavior = new ChestOrientationBehavior(outgoingCommunicationBridge, yoTime);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      isDone = new BooleanYoVariable("isDone", registry);
   }

   private void setPoses(RobotSide side)
   {
      FramePose turnHandPoseToCarryDebris = new FramePose(midFeetZUpFrame);
      FramePose armOutPose = new FramePose(midFeetZUpFrame);
      FramePose turnHandPoseToDropDebris = new FramePose(midFeetZUpFrame);
      FramePose armInFrontOfRobotPose = new FramePose(midFeetZUpFrame);
      FramePose armCloseToHomePose = new FramePose(midFeetZUpFrame);

      Point3d tempPosition = new Point3d();
      Quat4d tempOrientation = new Quat4d();

      // turn the hand to put the debris horizontally
      tempPosition.set(0.55, side.negateIfRightSide(0.1), 1.2);
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(tempOrientation, 0.0, 0.0, side.negateIfRightSide((Math.toRadians(90.0))));
      submitSingleTaskStageHandPose(turnHandPoseToCarryDebris, side, tempPosition, tempOrientation);

      // put the arm at the outside
      tempPosition.set(0.2, side.negateIfRightSide(0.8), 1.2);
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(tempOrientation, side.negateIfRightSide(Math.toRadians(80.0)), 0.0,
            side.negateIfRightSide(Math.toRadians(90.0)));
      submitSingleTaskStageHandPose(armOutPose, side, tempPosition, tempOrientation);

      // drop debris by opening hand and turning the hand
      tempPosition.set(0.2, side.negateIfRightSide(0.8), 1.2);
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(tempOrientation, side.negateIfRightSide(Math.toRadians(80.0)), 0.0, 0.0);
      submitReleaseDebrisParallellTasks(turnHandPoseToDropDebris, side, tempPosition, tempOrientation);

      //put the arm back in front of the robot
      tempPosition.set(0.55, side.negateIfRightSide(0.1), 1.2);
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(tempOrientation, 0.0, 0.0, 0.0);
      submitSingleTaskStageHandPose(armInFrontOfRobotPose, side, tempPosition, tempOrientation);

      //put the robot in its default position (arm, pelvis and chest)
      submitGoToDefaultPositionTasks(side);

   }

   private void submitGoToDefaultPositionTasks(RobotSide side)
   {
      double[] desiredArmJointAngles = wholeBodyControllerParameters.getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, side);
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime));

      desiredArmJointAngles = wholeBodyControllerParameters.getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.COMPACT_HOME,
            side.getOppositeSide());
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side.getOppositeSide(), desiredArmJointAngles, yoTime, handPoseBehavior,
            trajectoryTime));

      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side, FingerState.RESET, fingerStateBehavior, yoTime));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(side.getOppositeSide(), FingerState.RESET, fingerStateBehavior, yoTime));

      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior, new PelvisPoseTask(PacketControllerTools.createGoToHomePelvisPosePacket(trajectoryTime),
            yoTime, pelvisPoseBehavior));
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior,
            new ChestOrientationTask(PacketControllerTools.createGoToHomeChestOrientationPacket(trajectoryTime), yoTime, chestOrientationBehavior));
   }

   private void submitReleaseDebrisParallellTasks(FramePose handPoseToPack, RobotSide side, Point3d tempPosition, Quat4d tempOrientation)
   {
      handPoseToPack.setPose(tempPosition, tempOrientation);
      handPoseToPack.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, (new FingerStateTask(side, FingerState.OPEN, fingerStateBehavior, yoTime)));
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(side, trajectoryTime, handPoseToPack, Frame.WORLD, handPoseBehavior, yoTime));

   }

   private void submitSingleTaskStageHandPose(FramePose handPoseToPack, RobotSide side, Point3d tempPosition, Quat4d tempOrientation)
   {
      handPoseToPack.setPose(tempPosition, tempOrientation);
      handPoseToPack.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new HandPoseTask(side, trajectoryTime, handPoseToPack, Frame.WORLD, handPoseBehavior, yoTime));

   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   public void setInputs(RobotSide side)
   {
      setPoses(side);
      haveInputsBeenSet.set(true);
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      fingerStateBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
      fingerStateBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      handPoseBehavior.enableActions();
   }
   
   @Override
   public void pause()
   {
      handPoseBehavior.pause();
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return (pipeLine.isDone() && hasInputBeenSet());
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      haveInputsBeenSet.set(false);
      isDone.set(false);
   }

   @Override
   public void initialize()
   {
      haveInputsBeenSet.set(false);
      isDone.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
