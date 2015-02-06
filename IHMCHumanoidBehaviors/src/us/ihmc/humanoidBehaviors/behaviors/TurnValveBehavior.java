package us.ihmc.humanoidBehaviors.behaviors;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspValveTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ScriptTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
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
   public static final double turnValveAngle = Math.toRadians(30.0);

   private static final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final WalkingControllerParameters walkingControllerParameters;

   private final WalkToLocationBehavior walkToLocationBehavior;
   private final ComHeightBehavior comHeightBehavior;
   private final GraspValveBehavior graspValveBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   private final ScriptBehavior scriptBehavior;

   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;
   private final BooleanYoVariable isDone;
   private final BooleanYoVariable hasInputBeenSet;

   private ReferenceFrame valveFrame;

   private double maxObservedWristForce = 0.0;
   private double maxObservedCapturePointError = 0.0;

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, BooleanYoVariable tippingDetectedBoolean,
         WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);

      this.walkingControllerParameters = walkingControllerParameters;

      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      graspValveBehavior = new GraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport);

      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<ScriptBehaviorInputPacket>();

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.isDone = new BooleanYoVariable("isDone", registry);
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

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
         SysoutTool.println("TurnValveBehavior: Tipping detected! Pausing behavior.");

      }
   }

   public void setInput(ScriptBehaviorInputPacket scriptBehaviorInputPacket)
   {
      String scriptName = scriptBehaviorInputPacket.getScriptName();
      RigidBodyTransform valveTransformToWorld = scriptBehaviorInputPacket.getReferenceTransform();
      valveFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("valve", world, valveTransformToWorld);

      pipeLine.submitSingleTaskStage(createWalkToValveTask(valveTransformToWorld, walkingControllerParameters.getMaxStepLength()));

      if (scriptName == null)
      {
         SysoutTool.println("Not using script behavior.", DEBUG);

         pipeLine.submitSingleTaskStage(new CoMHeightTask(0.5 * ComHeightPacket.MIN_COM_HEIGHT, yoTime, comHeightBehavior, 1.0));
         pipeLine.submitSingleTaskStage(new GraspValveTask(graspValveBehavior, valveType, valveTransformToWorld, graspApproachDirectionInValveFrame,
               graspValveRim));

         if (graspValveBehavior.hasInputBeenSet())
         {
            FramePose graspPoseInWorld = new FramePose();
            graspValveBehavior.getFinalGraspPose(graspPoseInWorld);
            SysoutTool.println(" hand grasp pose: " + graspPoseInWorld, DEBUG);

            double trajectoryTime = 2.0;

            FramePose desiredHandPoseInWorldAfterFirstTurn = copyCurrentPoseAndRotateAboutValveXaxis(graspPoseInWorld, turnValveAngle);
            pipeLine.submitSingleTaskStage(new HandPoseTask(createHandPosePacket(trajectoryTime, desiredHandPoseInWorldAfterFirstTurn), handPoseBehavior,
                  yoTime));
            
            FramePose desiredHandPoseInWorldAfterSecondTurn = copyCurrentPoseAndRotateAboutValveXaxis(desiredHandPoseInWorldAfterFirstTurn, turnValveAngle);
            pipeLine.submitSingleTaskStage(new HandPoseTask(createHandPosePacket(trajectoryTime, desiredHandPoseInWorldAfterSecondTurn), handPoseBehavior,
                  yoTime));
            
            pipeLine.submitSingleTaskStage(new FingerStateTask(robotSideOfHandToUse, FingerState.OPEN, fingerStateBehavior));
            
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
            throw new RuntimeException("Can't set hand pose behaviors until Grasp Pose behavior has been set!");
         }

      }
      else
      {
         SysoutTool.println("New Script Behavior Input Packet Received.  Script File : " + scriptBehaviorInputPacket.getScriptName(), DEBUG);

         pipeLine.submitSingleTaskStage(new ScriptTask(scriptBehavior, scriptBehaviorInputPacket));
      }

      hasInputBeenSet.set(true);
   }

   private WalkToLocationTask createWalkToValveTask(RigidBodyTransform valveTransformToWorld, double stepLength)
   {
      FramePose2d valvePose2d = new FramePose2d();
      valvePose2d.setPose(valveTransformToWorld);
      double valveYaw = valvePose2d.getYaw();

      FramePose2d manipulationMidFeetPose = computeDesiredMidFeetPoseForManipulation(valvePose2d);
      WalkToLocationTask ret = new WalkToLocationTask(manipulationMidFeetPose, walkToLocationBehavior, valveYaw, stepLength);

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

   private FramePose copyCurrentPoseAndRotateAboutValveXaxis(FramePose currentPoseInWorld, double turnAngle)
   {
      FramePose currentPoseInValve = new FramePose(currentPoseInWorld);
      currentPoseInValve.changeFrame(valveFrame);

      RigidBodyTransform xRotation = new RigidBodyTransform();
      xRotation.rotX(turnAngle);
      
      RigidBodyTransform desiredTransformToValve = new RigidBodyTransform();
      currentPoseInValve.getPose(desiredTransformToValve);
      desiredTransformToValve.multiply(xRotation);
      
      Point3d desiredPositionInValve = new Point3d();
      currentPoseInValve.getPosition(desiredPositionInValve);
      xRotation.transform(desiredPositionInValve);
      
      FramePose ret = new FramePose(valveFrame, desiredTransformToValve);
      ret.setPosition(desiredPositionInValve);
      ret.changeFrame(world);

      return ret;
   }
   
   private HandPosePacket createHandPosePacket(double trajectoryTime, FramePose desiredHandPoseInWorld)
   {
      Point3d position = new Point3d();
      Quat4d orientation = new Quat4d();

      desiredHandPoseInWorld.getPosition(position);
      desiredHandPoseInWorld.getOrientation(orientation);

      Frame packetFrame;
      ReferenceFrame referenceFrame = desiredHandPoseInWorld.getReferenceFrame();
      if (referenceFrame.isWorldFrame())
      {
         packetFrame = Frame.WORLD;
      }
      else
      {
         throw new RuntimeException("Hand Pose must be defined in World reference frame.");
      }

      HandPosePacket ret = new HandPosePacket(robotSideOfHandToUse, packetFrame, position, orientation, trajectoryTime);
      return ret;
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      walkToLocationBehavior.consumeObjectFromNetworkProcessor(object);
      comHeightBehavior.consumeObjectFromNetworkProcessor(object);
      graspValveBehavior.consumeObjectFromNetworkProcessor(object);
      scriptBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
      walkToLocationBehavior.consumeObjectFromController(object);
      comHeightBehavior.consumeObjectFromController(object);
      graspValveBehavior.consumeObjectFromController(object);
      scriptBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      //      currentBehavior.stop();
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
      //      currentBehavior.pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      //      currentBehavior.resume();
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void finalize()
   {
      //      currentBehavior.finalize();
      isDone.set(false);
      hasInputBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      isDone.set(false);
      hasInputBeenSet.set(false);

      handPoseBehavior.initialize();
      walkToLocationBehavior.initialize();
      comHeightBehavior.initialize();
      graspValveBehavior.initialize();
      scriptBehavior.initialize();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
