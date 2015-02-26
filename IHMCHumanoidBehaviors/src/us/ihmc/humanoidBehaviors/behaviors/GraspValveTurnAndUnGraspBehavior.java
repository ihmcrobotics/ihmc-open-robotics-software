package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspValveTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.RotateHandAboutAxisTask;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveTurnAndUnGraspBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private final FullRobotModel fullRobotModel;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final ArrayList<BehaviorInterface> childBehaviors;

   private final GraspValveBehavior graspValveBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyInverseKinematicBehavior;
   private final RotateHandAboutAxisBehavior rotateGraspedValveBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;
   private final BooleanYoVariable hasInputBeenSet;

   private final double maxObservedWristForce = 0.0;
   private final double maxObservedCapturePointError = 0.0;

   public GraspValveTurnAndUnGraspBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, DoubleYoVariable yoTime, WholeBodyControllerParameters wholeBodyControllerParameters,
         BooleanYoVariable tippingDetectedBoolean, boolean useWholeBodyInverseKinematics)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;

      childBehaviors = new ArrayList<BehaviorInterface>();

      graspValveBehavior = new GraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, wholeBodyControllerParameters, yoTime);
      childBehaviors.add(graspValveBehavior);
      rotateGraspedValveBehavior = new RotateHandAboutAxisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, wholeBodyControllerParameters,
            yoTime, useWholeBodyInverseKinematics);
      childBehaviors.add(rotateGraspedValveBehavior);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(fingerStateBehavior);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(handPoseBehavior);
      wholeBodyInverseKinematicBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters, fullRobotModel,
            yoTime);

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);
   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet())
      {
         pipeLine.doControl();
      }
   }

   public void setInput(RigidBodyTransform valveTransformToWorld, ValveGraspLocation valveGraspLocation, double graspApproachConeAngle,
         Axis valvePinJointAxisInValveFrame, double valveRadius, double turnValveAngle)
   {
      RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
      double valveRotationRateRadPerSec = Math.PI / 2.0;
      double trajectoryTimeMoveHandAwayFromValve = 2.0;

      GraspValveTask graspValveTask = new GraspValveTask(graspValveBehavior, valveTransformToWorld, valveGraspLocation, graspApproachConeAngle,
            valvePinJointAxisInValveFrame, valveRadius, yoTime);

      RotateHandAboutAxisTask rotateGraspedValveTask = new RotateHandAboutAxisTask(robotSideOfHandToUse, yoTime, rotateGraspedValveBehavior,
            valveTransformToWorld, valvePinJointAxisInValveFrame, turnValveAngle, valveRotationRateRadPerSec);

      FingerStateTask openHandTask = new FingerStateTask(robotSideOfHandToUse, FingerState.OPEN, fingerStateBehavior, yoTime);

      FrameVector graspApproachDirection = getGraspApproachDirectionInWorld(valveTransformToWorld, valveGraspLocation, graspApproachConeAngle);

      HandPoseTask moveHandAwayFromValveTask = new HandPoseTask(robotSideOfHandToUse, graspApproachDirection.getVectorCopy(), -0.3, fullRobotModel, yoTime,
            handPoseBehavior, trajectoryTimeMoveHandAwayFromValve);

      pipeLine.submitSingleTaskStage(graspValveTask);
      pipeLine.submitSingleTaskStage(rotateGraspedValveTask);
      pipeLine.submitSingleTaskStage(openHandTask);
      pipeLine.submitSingleTaskStage(moveHandAwayFromValveTask);

      hasInputBeenSet.set(true);
   }

   private FrameVector getGraspApproachDirectionInWorld(RigidBodyTransform valveTransformToWorld, ValveGraspLocation valveGraspLocation,
         double graspApproachConeAngle)
   {
      TransformReferenceFrame valveFrame = new TransformReferenceFrame("valve", ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      FrameVector graspApproachDirection = new FrameVector(valveFrame, Math.cos(graspApproachConeAngle), 0.0, Math.sin(graspApproachConeAngle));

      double graspClockwiseOffsetFromTwelveOClock = -valveGraspLocation.ordinal() * Math.toRadians(90.0);
      RigidBodyTransform rotateClockwise = new RigidBodyTransform();
      rotateClockwise.rotX(graspClockwiseOffsetFromTwelveOClock);
      graspApproachDirection.applyTransform(rotateClockwise);

      graspApproachDirection.changeFrame(ReferenceFrame.getWorldFrame());
      return graspApproachDirection;
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.consumeObjectFromNetworkProcessor(object);
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.consumeObjectFromController(object);
      }
   }

   @Override
   public void stop()
   {
      pipeLine.getCurrentStage().stop();
   }

   @Override
   public void enableActions()
   {
      SysoutTool.println("max wrist force : " + maxObservedWristForce);
      SysoutTool.println("max capture point error : " + maxObservedCapturePointError);
   }

   @Override
   public void pause()
   {
      pipeLine.getCurrentStage().pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      pipeLine.getCurrentStage().resume();
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void finalize()
   {
      hasInputBeenSet.set(false);

      handPoseBehavior.finalize();
      graspValveBehavior.finalize();
      fingerStateBehavior.finalize();
   }

   @Override
   public void initialize()
   {
      hasInputBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
