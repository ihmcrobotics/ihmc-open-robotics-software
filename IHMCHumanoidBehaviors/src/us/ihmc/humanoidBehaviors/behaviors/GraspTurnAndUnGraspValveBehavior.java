package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveTurnDirection;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior.ValveGraspMethod;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspValveTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseRelativeToCurrentTask;
import us.ihmc.humanoidBehaviors.taskExecutor.RotateHandAboutAxisTask;
import us.ihmc.robotics.Axis;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.FullHumanoidRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspTurnAndUnGraspValveBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private final FullHumanoidRobotModel fullRobotModel;

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

   public GraspTurnAndUnGraspValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, DoubleYoVariable yoTime, WholeBodyControllerParameters wholeBodyControllerParameters,
         BooleanYoVariable tippingDetectedBoolean, boolean useWholeBodyInverseKinematics)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;

      childBehaviors = new ArrayList<BehaviorInterface>();

      graspValveBehavior = new GraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, wholeBodyControllerParameters, yoTime);
      childBehaviors.add(graspValveBehavior);
      rotateGraspedValveBehavior = new RotateHandAboutAxisBehavior("", outgoingCommunicationBridge, fullRobotModel, yoTime);
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

   public void setInput(RigidBodyTransform valveTransformToWorld, ValveGraspLocation graspLocation, ValveGraspMethod graspMethod, double graspApproachConeAngle,
	         Axis valvePinJointAxisInValveFrame, double valveRadius, double turnValveAngle, double valveRotationRateRadPerSec, boolean stopHandIfGraspCollision, boolean stopHandIfTurnCollision)
   {
      RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
      double trajectoryTimeMoveHandAwayFromValve = 2.0;

      ValveTurnDirection valveTurnDirection;
      if (turnValveAngle > 0.0)
      {
         valveTurnDirection = ValveTurnDirection.CLOCKWISE;
      }
      else
      {
         valveTurnDirection = ValveTurnDirection.COUNTERCLOCKWISE;
      }
         
      GraspValveTask graspValveTask = new GraspValveTask(graspValveBehavior, valveTransformToWorld, graspLocation, graspMethod, valveTurnDirection, graspApproachConeAngle,
              valvePinJointAxisInValveFrame, valveRadius, stopHandIfGraspCollision, yoTime);

      RotateHandAboutAxisTask rotateGraspedValveTask = new RotateHandAboutAxisTask(robotSideOfHandToUse, yoTime, rotateGraspedValveBehavior,
              valveTransformToWorld, valvePinJointAxisInValveFrame, false, turnValveAngle, valveRotationRateRadPerSec, stopHandIfTurnCollision);

      FingerStateTask openHandTask = new FingerStateTask(robotSideOfHandToUse, FingerState.OPEN, fingerStateBehavior, yoTime);

      HandPoseRelativeToCurrentTask moveHandAwayFromValveTask = new HandPoseRelativeToCurrentTask(robotSideOfHandToUse, -0.3, fullRobotModel, yoTime,
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
      PrintTools.debug(this, "max wrist force : " + maxObservedWristForce);
      PrintTools.debug(this, "max capture point error : " + maxObservedCapturePointError);
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
   public void doPostBehaviorCleanup()
   {
      hasInputBeenSet.set(false);

      handPoseBehavior.doPostBehaviorCleanup();
      graspValveBehavior.doPostBehaviorCleanup();
      fingerStateBehavior.doPostBehaviorCleanup();
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
