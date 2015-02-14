package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspValveTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.RotateHandAboutAxisTask;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
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
   private final RotateHandAboutAxisBehavior rotateGraspedValveBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;
   private final BooleanYoVariable hasInputBeenSet;

   private final double maxObservedWristForce = 0.0;
   private final double maxObservedCapturePointError = 0.0;

   public GraspValveTurnAndUnGraspBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, DoubleYoVariable yoTime, BooleanYoVariable tippingDetectedBoolean)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;

      childBehaviors = new ArrayList<BehaviorInterface>();

      graspValveBehavior = new GraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      childBehaviors.add(graspValveBehavior);
      rotateGraspedValveBehavior = new RotateHandAboutAxisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime);
      childBehaviors.add(rotateGraspedValveBehavior);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(fingerStateBehavior);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(handPoseBehavior);

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

      //      if (!currentBehavior.equals(walkToLocationBehavior))
      //      {
      //         pauseIfCapturePointErrorIsTooLarge();
      //      }
   }

   private void pauseIfCapturePointErrorIsTooLarge()
   {
      if (tippingDetected.getBooleanValue() && !isPaused.getBooleanValue())
      {
         this.pause();
         if (DEBUG)
            SysoutTool.println("TurnValveBehavior: Tipping detected! Pausing behavior.");
      }
   }

   public void setInput(ValveType valveType, RigidBodyTransform valveTransformToWorld, Vector3d graspApproachDirectionInValveFrame,
         Axis valvePinJointAxisInValveFrame, boolean graspValveRim, double turnValveAngle)
   {
      pipeLine.submitSingleTaskStage(new GraspValveTask(graspValveBehavior, valveType, valveTransformToWorld, graspApproachDirectionInValveFrame,
            graspValveRim, yoTime));

      double trajectoryTime = 2.0;
      RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
      pipeLine.submitSingleTaskStage(new RotateHandAboutAxisTask(robotSideOfHandToUse, yoTime, rotateGraspedValveBehavior, valveTransformToWorld,
            valvePinJointAxisInValveFrame, turnValveAngle, trajectoryTime));

      pipeLine.submitSingleTaskStage(new FingerStateTask(robotSideOfHandToUse, FingerState.OPEN, fingerStateBehavior, yoTime));

      RigidBodyTransform valveTransformToWorldRotationOnly = new RigidBodyTransform(valveTransformToWorld);
      valveTransformToWorldRotationOnly.setTranslation(new Vector3d());
      Vector3d graspApproachDirectionInWorld = new Vector3d(graspApproachDirectionInValveFrame);
      valveTransformToWorldRotationOnly.transform(graspApproachDirectionInWorld);

      pipeLine.submitSingleTaskStage(new HandPoseTask(robotSideOfHandToUse, graspApproachDirectionInWorld, -0.3, fullRobotModel, yoTime, handPoseBehavior,
            trajectoryTime));

      hasInputBeenSet.set(true);
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
