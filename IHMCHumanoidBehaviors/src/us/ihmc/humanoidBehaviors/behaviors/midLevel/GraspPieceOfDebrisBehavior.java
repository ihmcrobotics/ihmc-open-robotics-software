package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
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
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double trajectoryTime = 2.5;
   private static final double offsetToThePointOfGrabbing = 0.05;
   
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final ArrayList<BehaviorInterface> childBehaviors = new ArrayList<BehaviorInterface>();
   private final HandPoseBehavior handPoseBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   private final WholeBodyPacketBehavior wholeBodyPacketBehavior;
   private final ChestOrientationBehavior chestOrientationBehavior;
   private final PelvisPoseBehavior pelvisPoseBehavior;

   private final BooleanYoVariable haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet" + behaviorName, registry);
   private final BooleanYoVariable hasTasksBeenSetUp = new BooleanYoVariable("hasTasksBeenSetUp" + behaviorName, registry);

   private final ReferenceFrame midFeetZUpFrame;
   private final SDFFullHumanoidRobotModel actualFullRobotModel;
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final SDFFullHumanoidRobotModel graspingDebrisRobot;
   private final SDFFullRobotModel approachingDebrisRobot;
   private final DoubleYoVariable yoTime;

   private RigidBodyTransform debrisTransform;
   private Point3d graspPosition;
   private Vector3d graspVector;
   private RobotSide robotSide;
   private Quat4d rotationToBePerformedInWorldFrame = new Quat4d();

   public GraspPieceOfDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullHumanoidRobotModel fullRobotModel,
         ReferenceFrame midFeetZUpFrame, WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.actualFullRobotModel = fullRobotModel;
      this.yoTime = yoTime;
      this.midFeetZUpFrame = midFeetZUpFrame;

      // set up child behaviors
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(handPoseBehavior);
      
      wholeBodyIKBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters, fullRobotModel, yoTime);
      childBehaviors.add(wholeBodyIKBehavior);
      
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(fingerStateBehavior);
      
      wholeBodyPacketBehavior = new WholeBodyPacketBehavior(outgoingCommunicationBridge, yoTime, wholeBodyControllerParameters);
      childBehaviors.add(wholeBodyPacketBehavior);
      
      chestOrientationBehavior = new ChestOrientationBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(chestOrientationBehavior);
      
      pelvisPoseBehavior = new PelvisPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(pelvisPoseBehavior);
      
      for (BehaviorInterface behavior : childBehaviors)
      {
         registry.addChild(behavior.getYoVariableRegistry());
      }

      wholeBodyIKSolver = wholeBodyControllerParameters.createWholeBodyIkSolver();
      graspingDebrisRobot = wholeBodyControllerParameters.createFullRobotModel();
      approachingDebrisRobot = wholeBodyControllerParameters.createFullRobotModel();
   }

   public void setInput(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector, RobotSide robotSide)
   {
      if (graspVector.length() == 0.0)
      {
         throw new RuntimeException("graspVector has not been set!");
      }
      wholeBodyIKBehavior.setPositionAndOrientationErrorTolerance(0.05, 0.3);
      this.debrisTransform = debrisTransform;
      this.graspPosition = graspPosition;
      this.graspVector = graspVector;
      this.robotSide = robotSide;
      
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
            ControlledDoF.DOF_3P2R, false));

      pipeLine.requestNewStage();

      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior, new PelvisPoseTask(PacketControllerTools.createGoToHomePelvisPosePacket(trajectoryTime),
            yoTime, pelvisPoseBehavior));
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior,
            new ChestOrientationTask(PacketControllerTools.createGoToHomeChestOrientationPacket(trajectoryTime), yoTime, chestOrientationBehavior));
   }

   private void computeDesiredApproachPose(FramePose approachPoseToPack, SDFFullHumanoidRobotModel graspingDebrisRobot)
   {
      approachPoseToPack.setToZero(graspingDebrisRobot.getHandControlFrame(robotSide));
      approachPoseToPack.setX(-offsetToThePointOfGrabbing);
      approachPoseToPack.changeFrame(worldFrame);
   }

   private void computeDesiredGraspPose(FramePose desiredPoseToPack, Quat4d rotationToBePerformedInWorldFrame, Point3d graspPosition)
   {
      FramePose graspPoseWithoutOffset = new FramePose(worldFrame);
      graspPoseWithoutOffset.setOrientation(rotationToBePerformedInWorldFrame);
      graspPoseWithoutOffset.setPosition(graspPosition);

      PoseReferenceFrame graspWithoutOffsetReferenceFrame = new PoseReferenceFrame("graspWithoutOffsetReferenceFrame", graspPoseWithoutOffset);
      desiredPoseToPack.setToZero(graspWithoutOffsetReferenceFrame);
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

   public void computeWholeBodyIK(SDFFullHumanoidRobotModel actualRobotModel, SDFFullRobotModel desiredRobotModelToPack, int numberOfReseeds,
         FramePose endEffectorPose, double positionTolerance, double orientationTolerance, ControlledDoF controlledDofs)
   {
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(robotSide, controlledDofs);
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(robotSide.getOppositeSide(), ControlledDoF.DOF_NONE);
      wholeBodyIKSolver.getConfiguration().setMaxNumberOfAutomaticReseeds( numberOfReseeds );
      wholeBodyIKSolver.setGripperPalmTarget( robotSide, endEffectorPose);
      setPositionAndOrientationErrorTolerance(positionTolerance, orientationTolerance);

      try
      {
         ComputeResult computeResult = wholeBodyIKSolver.compute(actualRobotModel, desiredRobotModelToPack, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
         System.out.println(computeResult);
      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   private void setPositionAndOrientationErrorTolerance(double positionErrorTolerance, double orientationErrorTolerance)
   {
      wholeBodyIKSolver.taskEndEffectorPosition.get(RobotSide.RIGHT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      wholeBodyIKSolver.taskEndEffectorRotation.get(RobotSide.RIGHT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      
      wholeBodyIKSolver.taskEndEffectorPosition.get(RobotSide.LEFT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      wholeBodyIKSolver.taskEndEffectorRotation.get(RobotSide.LEFT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
   }

   @Override
   public void doControl()
   {
      if (isPaused())
      {
         return;
      }
      
      if(hasInputBeenSet() && !hasTasksBeenSetUp.getBooleanValue())
      {
         setTasks(debrisTransform, graspPosition, graspVector);
         hasTasksBeenSetUp.set(true);
      }
      else
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
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.consumeObjectFromNetworkProcessor(object);
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.consumeObjectFromController(object);
      }
   }

   public RobotSide getSideToUse()
   {
      return robotSide;
   }

   @Override
   public void stop()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.stop();
      }
   }

   @Override
   public void enableActions()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.enableActions();
      }
   }

   @Override
   public void pause()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.pause();
      }
   }

   @Override
   public void resume()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.resume();
      }
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone() && hasInputBeenSet();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      haveInputsBeenSet.set(false);
      hasTasksBeenSetUp.set(false);
   }

   @Override
   public void initialize()
   {
      wholeBodyIKSolver.setVerbosityLevel(0);
      wholeBodyIKSolver.getHierarchicalSolver().collisionAvoidance.setEnabled(true);

      haveInputsBeenSet.set(false);
      hasTasksBeenSetUp.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
