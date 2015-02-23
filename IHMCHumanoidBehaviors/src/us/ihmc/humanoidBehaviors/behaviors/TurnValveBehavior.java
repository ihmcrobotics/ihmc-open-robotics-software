package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.TurnValvePacket;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspValveTurnAndUnGraspTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ScriptTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class TurnValveBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private static final boolean graspValveRim = true;
   public static final RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
   public static final double howFarToStandToTheRightOfValve = robotSideOfHandToUse.negateIfRightSide(0.13); //0.13
   public static final double howFarToStandBackFromValve = 0.64; //0.64
   
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final WalkingControllerParameters walkingControllerParameters;

   private final ArrayList<BehaviorInterface> childBehaviors;
   private final HandPoseBehavior moveHandToHomeBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final GraspValveTurnAndUnGraspBehavior graspValveTurnAndUnGraspBehavior;
   private final ScriptBehavior scriptBehavior;

   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
   private final ConcurrentListeningQueue<TurnValvePacket> turnValvePacketListener;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;
   private final BooleanYoVariable hasInputBeenSet;

   private final double maxObservedWristForce = 0.0;
   private final double maxObservedCapturePointError = 0.0;

   private final double minDistanceForWalkingBetweenTurns = 0.1;
   private final double minYawDeltaForWalkingBetweenTurns = Math.toRadians(5.0);

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, BooleanYoVariable tippingDetectedBoolean,
         WholeBodyControllerParameters wholeBodyControllerParameters, WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);
      this.walkingControllerParameters = walkingControllerParameters;
      
      childBehaviors = new ArrayList<BehaviorInterface>();
      moveHandToHomeBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(moveHandToHomeBehavior);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      childBehaviors.add(walkToLocationBehavior);
      graspValveTurnAndUnGraspBehavior = new GraspValveTurnAndUnGraspBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime,
            wholeBodyControllerParameters, tippingDetectedBoolean);
      childBehaviors.add(graspValveTurnAndUnGraspBehavior);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport);
      childBehaviors.add(scriptBehavior);

      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<ScriptBehaviorInputPacket>();
      turnValvePacketListener = new ConcurrentListeningQueue<TurnValvePacket>();

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

      walkToLocationBehavior.setDistanceThreshold(minDistanceForWalkingBetweenTurns);
      walkToLocationBehavior.setYawAngleThreshold(minYawDeltaForWalkingBetweenTurns);

      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
      super.attachNetworkProcessorListeningQueue(turnValvePacketListener, TurnValvePacket.class);
   }

   @Override
   public void doControl()
   {
      if (scriptBehaviorInputPacketListener.isNewPacketAvailable() && !hasInputBeenSet.getBooleanValue())
      {
         setInput(scriptBehaviorInputPacketListener.getNewestPacket());
      }

      if (turnValvePacketListener.isNewPacketAvailable() && !hasInputBeenSet.getBooleanValue())
      {
         setInput(turnValvePacketListener.getNewestPacket());
      }

      pipeLine.doControl();

      if (walkToLocationBehavior.isDone())
      {
         pauseIfCapturePointErrorIsTooLarge();
      }
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

   public void setInput(TurnValvePacket turnValvePacket)
   {
      setInput(turnValvePacket.getValveTransformToWorld(), turnValvePacket.getGraspApproachDirectionInValveFrame(), turnValvePacket.getValveRadius(),
            turnValvePacket.getTurnValveAngle(), Math.toRadians(90.0));
   }

   public void setInput(RigidBodyTransform valveTransformToWorld, Vector3d graspApproachDirectionInValveFrame, double valveRadius,
         double totalAngleToRotateValve, double angleToRotatePerGraspUngraspCycle)
   {
      SysoutTool.println("Not using script behavior.", DEBUG);

      HandPoseTask moveHandToHomeTask = new HandPoseTask(robotSideOfHandToUse, PacketControllerTools.createGoToHomeHandPosePacket(robotSideOfHandToUse, 1.0),
            moveHandToHomeBehavior, yoTime);

      WalkToLocationTask walkToValveTask = createWalkToValveTask(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength());

      GraspValveTurnAndUnGraspTask graspValveTurnAndUnGraspTask = new GraspValveTurnAndUnGraspTask(graspValveTurnAndUnGraspBehavior, valveTransformToWorld,
            graspApproachDirectionInValveFrame, Axis.X, valveRadius, graspValveRim, angleToRotatePerGraspUngraspCycle, yoTime);

      int numberOfGraspUngraspCycles = (int) Math.ceil(totalAngleToRotateValve / angleToRotatePerGraspUngraspCycle);

      pipeLine.submitSingleTaskStage(moveHandToHomeTask);
      pipeLine.submitSingleTaskStage(walkToValveTask);
      for (int i = 0; i < numberOfGraspUngraspCycles; i++)
      {
         pipeLine.submitSingleTaskStage(graspValveTurnAndUnGraspTask);
      }
      pipeLine.submitSingleTaskStage(moveHandToHomeTask);

      hasInputBeenSet.set(true);
   }

   public void setInput(ScriptBehaviorInputPacket scriptBehaviorInputPacket)
   {
      RigidBodyTransform valveTransformToWorld = scriptBehaviorInputPacket.getReferenceTransform();

      SysoutTool.println("New Script Behavior Input Packet Received.  Script File : " + scriptBehaviorInputPacket.getScriptName(), DEBUG);

      pipeLine.submitSingleTaskStage(new HandPoseTask(robotSideOfHandToUse, PacketControllerTools.createGoToHomeHandPosePacket(robotSideOfHandToUse, 1.0),
            moveHandToHomeBehavior, yoTime));
      pipeLine.submitSingleTaskStage(createWalkToValveTask(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength()));
      pipeLine.submitSingleTaskStage(new ScriptTask(scriptBehavior, scriptBehaviorInputPacket, yoTime));

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
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.finalize();
      }

      hasInputBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.initialize();
      }
      hasInputBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
