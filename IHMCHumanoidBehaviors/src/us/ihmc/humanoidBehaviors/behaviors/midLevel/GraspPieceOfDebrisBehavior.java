package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.ChestOrientationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WholeBodyInverseKinematicTask;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspPieceOfDebrisBehavior extends BehaviorInterface
{
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final HandPoseBehavior handPoseBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final ChestOrientationBehavior chestOrientationBehavior;
   private final PelvisPoseBehavior pelvisPoseBehavior;

   private final BooleanYoVariable useWholeBodyIK;
   private final BooleanYoVariable haveInputsBeenSet;
   private final DoubleYoVariable offsetToThePointOfGrabbing;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private RobotSide robotSide;
   private double trajectoryTime = 2.5;

   private final FullRobotModel fullRobotModel;

   private final double WRIST_OFFSET = 0.14;

   private DoubleYoVariable yoTime;

   private Quat4d rotationToBePerformedInWorldFrame = new Quat4d();

   private final ReferenceFrame midFeetZUpFrame;

   public GraspPieceOfDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         ReferenceFrame midFeetZUpFrame, WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime, Boolean useWholeBodyIK)
   {
      super(outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.yoTime = yoTime;
      this.midFeetZUpFrame = midFeetZUpFrame;

      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      wholeBodyIKBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters, fullRobotModel, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      chestOrientationBehavior = new ChestOrientationBehavior(outgoingCommunicationBridge, yoTime);
      pelvisPoseBehavior = new PelvisPoseBehavior(outgoingCommunicationBridge, yoTime);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet" + behaviorName, registry);
      this.useWholeBodyIK = new BooleanYoVariable("useWholeBodyIK" + behaviorName, registry);

      this.useWholeBodyIK.set(useWholeBodyIK);

      offsetToThePointOfGrabbing = new DoubleYoVariable("offsetToThePointOfGrabbing", registry);
      offsetToThePointOfGrabbing.set(0.3);
   }

   public void setGraspPose(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector, RobotSide robotSide)
   {
      if (graspVector.length() == 0.0)
      {
         throw new RuntimeException("graspVector has not been set!");
      }
      this.robotSide = robotSide;
      setTasks(debrisTransform, graspPosition, graspVector);
      haveInputsBeenSet.set(true);
   }

   private void setTasks(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector)
   {
      FramePose desiredGrabPose = new FramePose();
      FramePose midGrabPose = new FramePose();

      computeDesiredGraspOrientation(debrisTransform, graspVector, fullRobotModel.getHandControlFrame(robotSide), rotationToBePerformedInWorldFrame);

      computeDesiredHandPosesWithOffsetAlongGraspVector(midGrabPose, rotationToBePerformedInWorldFrame, graspPosition, graspVector,
            offsetToThePointOfGrabbing.getDoubleValue());
      computeDesiredHandPosesWithOffsetAlongGraspVector(desiredGrabPose, rotationToBePerformedInWorldFrame, graspPosition, graspVector, WRIST_OFFSET);

      if (useWholeBodyIK.getBooleanValue())
         pipeLine.submitSingleTaskStage(new WholeBodyInverseKinematicTask(robotSide, yoTime, wholeBodyIKBehavior, midGrabPose, trajectoryTime));
      else
         pipeLine.submitSingleTaskStage(new HandPoseTask(robotSide, yoTime, handPoseBehavior, Frame.WORLD, midGrabPose, trajectoryTime));

      pipeLine.submitSingleTaskStage(new FingerStateTask(robotSide, FingerState.OPEN, fingerStateBehavior));

      if (useWholeBodyIK.getBooleanValue())
         pipeLine.submitSingleTaskStage(new WholeBodyInverseKinematicTask(robotSide, yoTime, wholeBodyIKBehavior, desiredGrabPose, trajectoryTime));
      else
         pipeLine.submitSingleTaskStage(new HandPoseTask(robotSide, yoTime, handPoseBehavior, Frame.WORLD, desiredGrabPose, trajectoryTime));

      pipeLine.submitSingleTaskStage(new FingerStateTask(robotSide, FingerState.CLOSE, fingerStateBehavior));

      FramePose prepareToDropPose = new FramePose(midFeetZUpFrame);
      prepareToDropPose.setPosition(0.55, robotSide.negateIfRightSide(0.1), 1.2);
      prepareToDropPose.changeFrame(worldFrame);
      prepareToDropPose.setOrientation(rotationToBePerformedInWorldFrame);

      if (useWholeBodyIK.getBooleanValue())
      {
         pipeLine.submitSingleTaskStage(new WholeBodyInverseKinematicTask(robotSide, yoTime, wholeBodyIKBehavior, prepareToDropPose, trajectoryTime));

         pipeLine.requestNewStage();

         pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior, new PelvisPoseTask(PacketControllerTools.createGoToHomePelvisPosePacket(trajectoryTime),
               yoTime, pelvisPoseBehavior));
         pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior,
               new ChestOrientationTask(PacketControllerTools.createGoToHomeChestOrientationPacket(trajectoryTime), yoTime, chestOrientationBehavior));
      }
   }

   private void computeDesiredHandPosesWithOffsetAlongGraspVector(FramePose desiredPoseToPack, Quat4d rotationToBePerformedInWorldFrame, Point3d graspPosition,
         Vector3d graspVector, double wristOffset)
   {
      Vector3d translation = new Vector3d(graspPosition);
      Vector3d tempGraspVector = new Vector3d(graspVector);

      desiredPoseToPack.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      desiredPoseToPack.changeFrame(worldFrame);

      desiredPoseToPack.setOrientation(rotationToBePerformedInWorldFrame);

      tempGraspVector.normalize();
      tempGraspVector.scale(wristOffset);
      translation.add(tempGraspVector);

      desiredPoseToPack.setPosition(translation);
   }

   private void computeDesiredGraspOrientation(RigidBodyTransform debrisTransform, Vector3d graspVector, ReferenceFrame handFrame,
         Quat4d desiredGraspOrientationToPack)
   {
      PoseReferenceFrame handFrameBeforeRotation = new PoseReferenceFrame("handFrameBeforeRotation", worldFrame);
      handFrameBeforeRotation.setPoseAndUpdate(debrisTransform);

      FramePose handPoseSolution1 = new FramePose(handFrameBeforeRotation);
      handPoseSolution1.changeFrame(handFrame);

      FramePose handPoseSolution2 = new FramePose(handFrameBeforeRotation);
      handPoseSolution2.setOrientation(0.0, 0.0, Math.PI);
      handPoseSolution2.changeFrame(handFrame);

      double rollOfSolution1 = handPoseSolution1.getRoll();
      double rollOfSolution2 = handPoseSolution2.getRoll();

      FramePose handPose = new FramePose(handFrameBeforeRotation);
      if (Math.abs(rollOfSolution1) <= Math.abs(rollOfSolution2))
      {
         handPose.setPoseIncludingFrame(handPoseSolution1);
      }
      else
      {
         handPose.setPoseIncludingFrame(handPoseSolution2);
      }

      handPose.changeFrame(worldFrame);
      handPose.getOrientation(desiredGraspOrientationToPack);
   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      wholeBodyIKBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
      wholeBodyIKBehavior.consumeObjectFromController(object);
   }

   public RobotSide getSideToUse()
   {
      return robotSide;
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
   public void finalize()
   {
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      haveInputsBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
