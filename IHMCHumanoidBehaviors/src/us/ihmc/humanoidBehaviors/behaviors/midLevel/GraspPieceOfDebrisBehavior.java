package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyPacketBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.ChestOrientationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WholeBodyInverseKinematicTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WholeBodyPacketTask;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspPieceOfDebrisBehavior extends BehaviorInterface
{
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final HandPoseBehavior handPoseBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   private final WholeBodyPacketBehavior wholeBodyPacketBehavior;

   private final ChestOrientationBehavior chestOrientationBehavior;
   private final PelvisPoseBehavior pelvisPoseBehavior;

   private final BooleanYoVariable haveInputsBeenSet;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame midFeetZUpFrame;

   private RobotSide robotSide;
   private final double trajectoryTime = 2.5;
   private final double wristOffset = 0.14;
   private final double offsetToThePointOfGrabbing = 0.05;

   private final SDFFullRobotModel actualFullRobotModel;

   private final DoubleYoVariable yoTime;

   private Quat4d rotationToBePerformedInWorldFrame = new Quat4d();

   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final SDFFullRobotModel graspingDebrisRobot;
   private final SDFFullRobotModel approachingDebrisRobot;

   public GraspPieceOfDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         ReferenceFrame midFeetZUpFrame, WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.actualFullRobotModel = fullRobotModel;
      this.yoTime = yoTime;
      this.midFeetZUpFrame = midFeetZUpFrame;

      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      wholeBodyIKBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters, fullRobotModel, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      wholeBodyPacketBehavior = new WholeBodyPacketBehavior(outgoingCommunicationBridge, yoTime, wholeBodyControllerParameters);
      chestOrientationBehavior = new ChestOrientationBehavior(outgoingCommunicationBridge, yoTime);
      pelvisPoseBehavior = new PelvisPoseBehavior(outgoingCommunicationBridge, yoTime);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet" + behaviorName, registry);

      wholeBodyIKSolver = wholeBodyControllerParameters.createWholeBodyIkSolver();
      graspingDebrisRobot = wholeBodyControllerParameters.createFullRobotModel();
      approachingDebrisRobot = wholeBodyControllerParameters.createFullRobotModel();
   }

   public void setGraspPose(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector, RobotSide robotSide)
   {
      if (graspVector.length() == 0.0)
      {
         throw new RuntimeException("graspVector has not been set!");
      }
      this.robotSide = robotSide;
      wholeBodyIKBehavior.setPositionAndOrientationErrorTolerance(0.05, 0.3);
      setTasks(debrisTransform, graspPosition, graspVector);
      haveInputsBeenSet.set(true);
   }

   private void setTasks(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector)
   {
      computeDesiredGraspOrientation(debrisTransform, graspVector, actualFullRobotModel.getHandControlFrame(robotSide), rotationToBePerformedInWorldFrame);
      FramePose graspPose = new FramePose();
      FramePose approachPose = new FramePose();

      computeDesiredGraspPose(graspPose, rotationToBePerformedInWorldFrame, graspPosition);
      computeWholeBodyIK(actualFullRobotModel, graspingDebrisRobot, 6, graspPose, 0.005, 0.1, ControlledDoF.DOF_3P2R);

      computeDesiredApproachPose(approachPose, graspingDebrisRobot);
      computeWholeBodyIK(graspingDebrisRobot, approachingDebrisRobot, 0, approachPose, 0.02, 0.1, ControlledDoF.DOF_3P3R);
      pipeLine.submitSingleTaskStage(new WholeBodyPacketTask(approachingDebrisRobot, yoTime, wholeBodyPacketBehavior, trajectoryTime));

      pipeLine.submitSingleTaskStage(new FingerStateTask(robotSide, FingerState.OPEN, fingerStateBehavior, yoTime));

      pipeLine.submitSingleTaskStage(new WholeBodyPacketTask(graspingDebrisRobot, yoTime, wholeBodyPacketBehavior, trajectoryTime));

      pipeLine.submitSingleTaskStage(new FingerStateTask(robotSide, FingerState.CLOSE, fingerStateBehavior, yoTime));

      FramePose prepareToDropPose = new FramePose(midFeetZUpFrame);
      prepareToDropPose.setPosition(0.55, robotSide.negateIfRightSide(0.2), 1.2);
      prepareToDropPose.changeFrame(worldFrame);
      prepareToDropPose.setOrientation(rotationToBePerformedInWorldFrame);

      pipeLine.submitSingleTaskStage(new WholeBodyInverseKinematicTask(robotSide, yoTime, wholeBodyIKBehavior, prepareToDropPose, trajectoryTime, 0,
            ControlledDoF.DOF_3P2R));

      pipeLine.requestNewStage();

      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior, new PelvisPoseTask(PacketControllerTools.createGoToHomePelvisPosePacket(trajectoryTime),
            yoTime, pelvisPoseBehavior));
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior,
            new ChestOrientationTask(PacketControllerTools.createGoToHomeChestOrientationPacket(trajectoryTime), yoTime, chestOrientationBehavior));
   }

   private void computeDesiredApproachPose(FramePose approachPoseToPack, SDFFullRobotModel graspingDebrisRobot)
   {
      approachPoseToPack.setToZero(graspingDebrisRobot.getHandControlFrame(robotSide));
      approachPoseToPack.setX(-offsetToThePointOfGrabbing - wristOffset);
      approachPoseToPack.changeFrame(worldFrame);
   }

   private void computeDesiredGraspPose(FramePose desiredPoseToPack, Quat4d rotationToBePerformedInWorldFrame, Point3d graspPosition)
   {
      FramePose graspPoseWithoutOffset = new FramePose(worldFrame);
      graspPoseWithoutOffset.setOrientation(rotationToBePerformedInWorldFrame);
      graspPoseWithoutOffset.setPosition(graspPosition);

      PoseReferenceFrame graspWithoutOffsetReferenceFrame = new PoseReferenceFrame("graspWithoutOffsetReferenceFrame", graspPoseWithoutOffset);
      desiredPoseToPack.setToZero(graspWithoutOffsetReferenceFrame);
      desiredPoseToPack.translate(-wristOffset, 0.0, 0.0);
      desiredPoseToPack.changeFrame(worldFrame);
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
         handPose.setPoseIncludingFrame(handPoseSolution1);
      else
         handPose.setPoseIncludingFrame(handPoseSolution2);

      handPose.changeFrame(worldFrame);
      handPose.getOrientation(desiredGraspOrientationToPack);
   }

   public void computeWholeBodyIK(SDFFullRobotModel actualRobotModel, SDFFullRobotModel desiredRobotModelToPack, int numberOfReseeds,
         FramePose endEffectorPose, double positionTolerance, double orientationTolerance, ControlledDoF controlledDofs)
   {
      wholeBodyIKSolver.setNumberOfControlledDoF(robotSide, controlledDofs);
      wholeBodyIKSolver.setNumberOfControlledDoF(robotSide.getOppositeSide(), ControlledDoF.DOF_NONE);
      wholeBodyIKSolver.maxNumberOfAutomaticReseeds = numberOfReseeds;
      wholeBodyIKSolver.setGripperAttachmentTarget(actualRobotModel, robotSide, endEffectorPose);
      setPositionAndOrientationErrorTolerance(positionTolerance, orientationTolerance);

      try
      {
         ComputeResult computeResult = wholeBodyIKSolver.compute(actualRobotModel, desiredRobotModelToPack, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   private void setPositionAndOrientationErrorTolerance(double positionErrorTolerance, double orientationErrorTolerance)
   {
      wholeBodyIKSolver.taskEndEffectorPose.get(RobotSide.RIGHT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      wholeBodyIKSolver.taskEndEffectorPose.get(RobotSide.LEFT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
   }

   @Override
   public void doControl()
   {
      if (pipeLine.getCurrentStage() instanceof WholeBodyInverseKinematicTask)
      {
         if (!wholeBodyIKBehavior.hasSolutionBeenFound())
         {
            pipeLine.clearAll();
         }
      }
      pipeLine.doControl();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      wholeBodyIKBehavior.consumeObjectFromNetworkProcessor(object);
      wholeBodyPacketBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
      wholeBodyIKBehavior.consumeObjectFromController(object);
      wholeBodyPacketBehavior.consumeObjectFromController(object);
   }

   public RobotSide getSideToUse()
   {
      return robotSide;
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
      wholeBodyIKBehavior.stop();
      fingerStateBehavior.stop();
      chestOrientationBehavior.stop();
      pelvisPoseBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      handPoseBehavior.enableActions();
      wholeBodyIKBehavior.enableActions();
      fingerStateBehavior.enableActions();
      chestOrientationBehavior.enableActions();
      pelvisPoseBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      handPoseBehavior.pause();
      wholeBodyIKBehavior.pause();
      fingerStateBehavior.pause();
      chestOrientationBehavior.pause();
      pelvisPoseBehavior.pause();
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
      wholeBodyIKBehavior.resume();
      fingerStateBehavior.resume();
      chestOrientationBehavior.resume();
      pelvisPoseBehavior.resume();
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
      wholeBodyIKSolver.setVerbosityLevel(0);
      wholeBodyIKSolver.getHierarchicalSolver().collisionAvoidance.setEnabled(true);

      haveInputsBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
