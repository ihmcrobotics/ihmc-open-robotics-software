package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.TurnValvePacket;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior.WalkingOrientation;
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
import us.ihmc.utilities.math.geometry.Pose2dReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class TurnValveBehavior extends BehaviorInterface
{
   public enum ValveGraspLocation
   {
      TWELVE_O_CLOCK, THREE_O_CLOCK, SIX_O_CLOCK, NINE_O_CLOCK
   }

   private static final boolean DEBUG = false;

   public static final RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
   public static final ValveGraspLocation DEFAULT_GRASP_LOCATION = ValveGraspLocation.NINE_O_CLOCK;
   public static final double DEFAULT_ANGLE_TO_ROTATE_PER_GRASP_CYCLE = Math.toRadians(90.0);
   public static final double howFarToStandToTheRightOfValve = 2.0 * robotSideOfHandToUse.negateIfRightSide(0.13); //0.13
   public static final double howFarToStandBackFromValve = 0.64; //0.64

   private final SDFFullRobotModel fullRobotModel;
   private final ReferenceFrames referenceFrames;

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
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.walkingControllerParameters = walkingControllerParameters;

      childBehaviors = new ArrayList<BehaviorInterface>();
      moveHandToHomeBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(moveHandToHomeBehavior);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      childBehaviors.add(walkToLocationBehavior);
      graspValveTurnAndUnGraspBehavior = new GraspValveTurnAndUnGraspBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime,
            wholeBodyControllerParameters, tippingDetectedBoolean);
      childBehaviors.add(graspValveTurnAndUnGraspBehavior);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport, walkingControllerParameters);
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
      setInput(turnValvePacket.getValveTransformToWorld(), DEFAULT_GRASP_LOCATION, turnValvePacket.getGraspApproachConeAngle(), Axis.X,
            turnValvePacket.getValveRadius(), turnValvePacket.getTurnValveAngle(),
            DEFAULT_ANGLE_TO_ROTATE_PER_GRASP_CYCLE * Math.signum(turnValvePacket.getTurnValveAngle()));
   }

   public void setInput(RigidBodyTransform valveTransformToWorld, ValveGraspLocation valveGraspLocation, double graspApproachConeAngle,
         Axis valvePinJointAxisInValveFrame, double valveRadius, double totalAngleToRotateValve, double angleToRotatePerGraspUngraspCycle)
   {
      SysoutTool.println("Not using script behavior.", DEBUG);

      HandPoseTask moveHandToHomeTask = new HandPoseTask(robotSideOfHandToUse, PacketControllerTools.createGoToHomeHandPosePacket(robotSideOfHandToUse, 1.0),
            moveHandToHomeBehavior, yoTime);

      WalkToLocationTask walkToValveTask = createWalkToValveTask(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength());

      angleToRotatePerGraspUngraspCycle = Math.abs(angleToRotatePerGraspUngraspCycle) * Math.signum(totalAngleToRotateValve);
      GraspValveTurnAndUnGraspTask graspValveTurnAndUnGraspTask = new GraspValveTurnAndUnGraspTask(graspValveTurnAndUnGraspBehavior, valveTransformToWorld,
            valveGraspLocation, graspApproachConeAngle, valvePinJointAxisInValveFrame, valveRadius, angleToRotatePerGraspUngraspCycle, yoTime);

      pipeLine.submitSingleTaskStage(moveHandToHomeTask);
      pipeLine.submitSingleTaskStage(walkToValveTask);

      int numberOfGraspUngraspCycles = (int) Math.ceil(totalAngleToRotateValve / angleToRotatePerGraspUngraspCycle);
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

      HandPoseTask moveHandToHomeTask = new HandPoseTask(robotSideOfHandToUse, PacketControllerTools.createGoToHomeHandPosePacket(robotSideOfHandToUse, 1.0),
            moveHandToHomeBehavior, yoTime);
      WalkToLocationTask walkToValveTask = createWalkToValveTask(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength());
      ScriptTask turnValveTask = new ScriptTask(scriptBehavior, scriptBehaviorInputPacket, yoTime);

      pipeLine.submitSingleTaskStage(moveHandToHomeTask);
      pipeLine.submitSingleTaskStage(walkToValveTask);
      pipeLine.submitSingleTaskStage(turnValveTask);

      hasInputBeenSet.set(true);
   }

   private WalkToLocationTask createWalkToValveTask(RigidBodyTransform valveTransformToWorld, double stepLength)
   {
      fullRobotModel.updateFrames();

      FramePose2d valvePose2d = new FramePose2d();
      valvePose2d.setPose(valveTransformToWorld);
      
      Pose2dReferenceFrame valveZUpFrame = new Pose2dReferenceFrame("valveZUp", valvePose2d);
      
      FramePose2d targetMidFeetZUpFramePose = new FramePose2d(valveZUpFrame);
      targetMidFeetZUpFramePose.setX(-howFarToStandBackFromValve);
      targetMidFeetZUpFramePose.setY(-howFarToStandToTheRightOfValve);
      targetMidFeetZUpFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      double maxDistanceToWalkNotAlignedWithWalkingPath = 4.0 * walkingControllerParameters.getMaxStepLength();
      boolean targetIsNearby = getWalkingDistanceToValve(targetMidFeetZUpFramePose) < maxDistanceToWalkNotAlignedWithWalkingPath;
      WalkingOrientation walkingOrientation;
      if (targetIsNearby)
      {
         walkingOrientation = WalkingOrientation.START_TARGET_ORIENTATION_MEAN;
      }
      else
      {
         walkingOrientation = WalkingOrientation.ALIGNED_WITH_PATH;
      }

      double sleepTimeBeforeNextTask = 0.0;
      WalkToLocationTask ret = new WalkToLocationTask(targetMidFeetZUpFramePose, walkToLocationBehavior, walkingOrientation, stepLength, yoTime, sleepTimeBeforeNextTask);

      return ret;
   }
   
   private final FramePose2d initialMidFeetPose = new FramePose2d();

   private FramePose2d getInitialRobotMidFeetZupPose()
   {
      fullRobotModel.updateFrames();
      initialMidFeetPose.setPose(referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame());
      
      return initialMidFeetPose;
   }
   
   private double getWalkingDistanceToValve(FramePose2d targetMidFeetFramePose)
   {
      return getInitialRobotMidFeetZupPose().getPositionDistance(targetMidFeetFramePose);
   }
   
   private boolean isWalkingTargetBehindRobot(FramePose2d targetMidFeetFramePose)
   {
      targetMidFeetFramePose.changeFrame(referenceFrames.getMidFeetZUpFrame());
      double targetMidFeetXPositionInCurrentMidFeetFrame = targetMidFeetFramePose.getX();
      boolean ret = targetMidFeetXPositionInCurrentMidFeetFrame < 0.0;
      
      return ret;
   }
   
   private boolean isRobotAlreadyFacingValve(FramePose2d targetMidFeetFramePose)
   {
      double orientationDistance = targetMidFeetFramePose.getOrientationDistance(getInitialRobotMidFeetZupPose());
      return orientationDistance < Math.toRadians(5.0);
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
