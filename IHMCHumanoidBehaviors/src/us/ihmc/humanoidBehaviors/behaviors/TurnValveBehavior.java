package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspValveTask;
import us.ihmc.humanoidBehaviors.taskExecutor.RotateHandAboutAxisTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ScriptTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class TurnValveBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private static final boolean graspValveRim = true;
   private static final ValveType valveType = ValveType.BIG_VALVE;
   private static final Vector3d valvePinJointAxisInValveFrame = new Vector3d(1, 0, 0);
   public static final RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
   public static final double howFarToStandToTheRightOfValve = robotSideOfHandToUse.negateIfRightSide(0.13); //0.13
   public static final double howFarToStandBackFromValve = 0.64; //0.64

   public static final Vector3d graspApproachDirectionInValveFrame = new Vector3d(valvePinJointAxisInValveFrame);
   public static final double turnValveAngle = Math.toRadians(90.0);

   private final FullRobotModel fullRobotModel;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final WalkingControllerParameters walkingControllerParameters;

   private final ArrayList<BehaviorInterface> childBehaviors;

   private final WalkToLocationBehavior walkToLocationBehavior;
   private final ComHeightBehavior comHeightBehavior;
   private final GraspValveBehavior graspValveBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final RotateHandAboutAxisBehavior rotateGraspedValveBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   private final ScriptBehavior scriptBehavior;

   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;
   private final BooleanYoVariable hasInputBeenSet;

   private final double maxObservedWristForce = 0.0;
   private final double maxObservedCapturePointError = 0.0;

   private final double minDistanceForWalkingBetweenTurns = 0.1;
   private final double minYawDeltaForWalkingBetweenTurns = Math.toRadians(5.0);

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, BooleanYoVariable tippingDetectedBoolean,
         WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;
      this.walkingControllerParameters = walkingControllerParameters;

      childBehaviors = new ArrayList<BehaviorInterface>();

      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      childBehaviors.add(walkToLocationBehavior);
      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(comHeightBehavior);
      rotateGraspedValveBehavior = new RotateHandAboutAxisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime);
      childBehaviors.add(rotateGraspedValveBehavior);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(handPoseBehavior);
      graspValveBehavior = new GraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);// removed arguement so it would compile
      childBehaviors.add(graspValveBehavior);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(fingerStateBehavior);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport);
      childBehaviors.add(scriptBehavior);

      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<ScriptBehaviorInputPacket>();

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

      walkToLocationBehavior.setDistanceThreshold(minDistanceForWalkingBetweenTurns);
      walkToLocationBehavior.setYawAngleThreshold(minYawDeltaForWalkingBetweenTurns);

      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
   }

   @Override
   public void doControl()
   {
      if (scriptBehaviorInputPacketListener.isNewPacketAvailable() && !hasInputBeenSet.getBooleanValue())
      {
         setInput(scriptBehaviorInputPacketListener.getNewestPacket());
      }

      pipeLine.doControl();

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

   private final Vector3d valveOffsetFromWorld = new Vector3d();

   public void setInput(ScriptBehaviorInputPacket scriptBehaviorInputPacket)
   {
      String scriptName = scriptBehaviorInputPacket.getScriptName();
      RigidBodyTransform valveTransformToWorld = scriptBehaviorInputPacket.getReferenceTransform();
      valveTransformToWorld.get(valveOffsetFromWorld);

      pipeLine.submitSingleTaskStage(createWalkToValveTask(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength()));

      if (scriptName == null)
      {
         SysoutTool.println("Not using script behavior.", DEBUG);

         double comHeightDesired = valveOffsetFromWorld.getZ() - 1.15;
         pipeLine.submitSingleTaskStage(new CoMHeightTask(comHeightDesired, yoTime, comHeightBehavior, 1.0));
         pipeLine.submitSingleTaskStage(new GraspValveTask(graspValveBehavior, valveType, valveTransformToWorld, graspApproachDirectionInValveFrame,
               graspValveRim, yoTime));

         double trajectoryTime = 2.0;

         pipeLine.submitSingleTaskStage(new RotateHandAboutAxisTask(robotSideOfHandToUse, yoTime, rotateGraspedValveBehavior, valveTransformToWorld, Axis.X,
               turnValveAngle, trajectoryTime));
         pipeLine.submitSingleTaskStage(new FingerStateTask(robotSideOfHandToUse, FingerState.OPEN, fingerStateBehavior, yoTime));

         //         pipeLine.submitSingleTaskStage(new HandPoseTask(robotSideOfHandToUse, graspApproachDirectionInValveFrame, -0.3, fullRobotModel, yoTime, handPoseBehavior, trajectoryTime));  //FIXME: For some reason this messes up the preceeding graspValveTask
         //         
         //         pipeLine.submitSingleTaskStage(new GraspValveTask(graspValveBehavior, valveType, valveTransformToWorld, graspApproachDirectionInValveFrame,
         //               graspValveRim, yoTime));
         //            pipeLine.submitSingleTaskStage(new HandPoseTask(createHandPosePacket(trajectoryTime, graspPoseInWorld), handPoseBehavior, yoTime));
         //            
         //            pipeLine.submitSingleTaskStage(new GraspValveTask(graspValveBehavior, valveType, valveTransformToWorld, graspApproachDirectionInValveFrame,
         //                  graspValveRim));
         //            graspValveBehavior.getFinalGraspPose(graspPoseInWorld);
         //            
         //            FramePose desiredHandPoseInWorldAfterThirdTurn = copyCurrentPoseAndRotateAboutValveXaxis(graspPoseInWorld, turnValveAngle);
         //            pipeLine.submitSingleTaskStage(new HandPoseTask(createHandPosePacket(trajectoryTime, desiredHandPoseInWorldAfterThirdTurn), handPoseBehavior,
         //                  yoTime));

      }
      else
      {
         SysoutTool.println("New Script Behavior Input Packet Received.  Script File : " + scriptBehaviorInputPacket.getScriptName(), DEBUG);

         pipeLine.submitSingleTaskStage(new ScriptTask(scriptBehavior, scriptBehaviorInputPacket, yoTime));
      }

      hasInputBeenSet.set(true);
   }

   private WalkToLocationTask createWalkToValveTask(RigidBodyTransform valveTransformToWorld, double stepLength)
   {
      FramePose2d valvePose2d = new FramePose2d();
      valvePose2d.setPose(valveTransformToWorld);
      double valveYaw = valvePose2d.getYaw();

      FramePose2d manipulationMidFeetPose = computeDesiredMidFeetPoseForManipulation(valvePose2d);
      WalkToLocationTask ret = new WalkToLocationTask(manipulationMidFeetPose, walkToLocationBehavior, valveYaw, stepLength, yoTime);

      return ret;
   }

   private FramePose2d computeDesiredMidFeetPoseForManipulation(FramePose2d valvePose2d)
   {
      FramePose2d ret = new FramePose2d(valvePose2d);

      RigidBodyTransform yawTransform = new RigidBodyTransform();
      TransformTools.rotate(yawTransform, valvePose2d.getYaw(), Axis.Z);

      Vector3d desiredRobotPosFromValve = new Vector3d(-howFarToStandBackFromValve, -howFarToStandToTheRightOfValve, 0.0);
      Vector3d desiredRobotPosFromValveInWorld = TransformTools.getTransformedVector(desiredRobotPosFromValve, yawTransform);

      ret.setX(ret.getX() + desiredRobotPosFromValveInWorld.x);
      ret.setY(ret.getY() + desiredRobotPosFromValveInWorld.y);

      return ret;
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
      //      SysoutTool.println("Current Child Behavior: " + currentBehavior.getName());

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
      walkToLocationBehavior.finalize();
      comHeightBehavior.finalize();
      graspValveBehavior.finalize();
      fingerStateBehavior.finalize();
      scriptBehavior.finalize();
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
