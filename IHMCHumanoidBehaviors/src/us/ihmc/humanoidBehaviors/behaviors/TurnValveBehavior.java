package us.ihmc.humanoidBehaviors.behaviors;

//~--- non-JDK imports --------------------------------------------------------

//~--- JDK imports ------------------------------------------------------------
import java.io.InputStream;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
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

   private boolean useScriptBehavior;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final FullRobotModel fullRobotModel;

   private final WalkToLocationBehavior walkToLocationBehavior;
   private final ComHeightBehavior comHeightBehavior;
   private final GraspValveBehavior graspValveBehavior;
   private final HandPoseBehavior handPoseValveTurn1Behavior;
   private final HandPoseBehavior handPoseValveTurn2Behavior;
   private final ScriptBehavior scriptBehavior;

   private InputStream scriptResourceStream;
   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;

   private final ArrayList<BehaviorInterface> behaviorQueue = new ArrayList<>();
   private BehaviorInterface currentBehavior;
   private BooleanYoVariable isDone;
   private BooleanYoVariable hasInputBeenSet;

   private double maxObservedWristForce = 0.0;
   private double maxObservedCapturePointError = 0.0;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, BooleanYoVariable tippingDetectedBoolean,
         WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;

      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      graspValveBehavior = new GraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      handPoseValveTurn1Behavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      handPoseValveTurn2Behavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport);

      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<ScriptBehaviorInputPacket>();

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.isDone = new BooleanYoVariable("isDone", registry);
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);

      //      BehaviorCommunicationBridge behaviorCommunicationBridge = (BehaviorCommunicationBridge) outgoingCommunicationBridge;
      //      behaviorCommunicationBridge.attachGlobalListenerToController(handPoseBehavior.getControllerGlobalPacketConsumer());
      //      behaviorCommunicationBridge.attachGlobalListenerToController(handPoseBehavior2.getControllerGlobalPacketConsumer());
   }

   @Override
   public void doControl()
   {
      if (isDone.getBooleanValue())
      {
         return;
      }
      else
      {
         checkForNewInputs();
         //
         //         taskExecutor.doControl();
         //         
         //         if (taskExecutor.isDone())
         //         {
         //            isDone.set(true);
         //         }

         //      if (!currentBehavior.equals(walkToLocationBehavior))
         //      {
         //      pauseIfCapturePointErrorIsTooLarge();
         //      }

         if (currentBehavior.isDone())
         {
            SysoutTool.println(currentBehavior.getName() + " is done.", DEBUG);
            currentBehavior.finalize();
            if (!behaviorQueue.isEmpty())
            {
               currentBehavior = behaviorQueue.remove(0);
               SysoutTool.println(currentBehavior.getName() + " is starting.", DEBUG);
            }
            else
            {
               isDone.set(true);
            }
         }
         currentBehavior.doControl();
      }
   }

   private void checkForNewInputs()
   {
      if (scriptBehaviorInputPacketListener.isNewPacketAvailable() && !hasInputBeenSet.getBooleanValue())
      {
         ScriptBehaviorInputPacket scriptBehaviorInputPacket = scriptBehaviorInputPacketListener.getNewestPacket();
         setInput(scriptBehaviorInputPacket);
      }
   }

   public void setInput(ScriptBehaviorInputPacket scriptBehaviorInputPacket)
   {
      String scriptName = scriptBehaviorInputPacket.getScriptName();

      if (scriptName == null)
      {
         useScriptBehavior = false;
         SysoutTool.println("Not using script behavior.", DEBUG);

      }
      else
      {
         useScriptBehavior = true;
         SysoutTool.println("New Script Behavior Input Packet Received.  Script File : " + scriptBehaviorInputPacket.getScriptName(), DEBUG);
      }

      RigidBodyTransform valveTransformToWorld = scriptBehaviorInputPacket.getReferenceTransform();

      setWalkToLocationBehaviorInput(valveTransformToWorld);

      if (useScriptBehavior)
      {
         scriptBehavior.importScriptInputPacket(scriptBehaviorInputPacket);
      }
      else
      {
         comHeightBehavior.setInput(new ComHeightPacket(0.5 * ComHeightPacket.MIN_COM_HEIGHT, 1.0));

         FramePose graspPose = setGraspValveBehaviorInput(valveTransformToWorld);

         double trajectoryTime = 2.0;
         double turnValveAngle = Math.toRadians(30.0);
         setHandPoseBehaviorInputs(valveTransformToWorld, graspPose, trajectoryTime, turnValveAngle);
      }

      hasInputBeenSet.set(true);

      for (BehaviorInterface behavior : behaviorQueue)
      {
         if (!behavior.hasInputBeenSet())
         {
            hasInputBeenSet.set(true);
         }
      }
   }

   private void setWalkToLocationBehaviorInput(RigidBodyTransform valveTransformToWorld)
   {
      FramePose valvePose = new FramePose();
      valvePose.setPoseIncludingFrame(world, valveTransformToWorld);
      SysoutTool.println("Turn Valve Location Updated:" + valvePose, DEBUG);

      FramePose2d manipulationMidFeetPose = computeDesiredMidFeetPoseForManipulation(valvePose);
      walkToLocationBehavior.setTarget(manipulationMidFeetPose);
      SysoutTool.println("Target Walk to Location Updated:" + manipulationMidFeetPose, DEBUG);
   }

   private FramePose2d computeDesiredMidFeetPoseForManipulation(FramePose valvePose)
   {
      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(world, valvePose.getX(), valvePose.getY(), valvePose.getYaw());

      RigidBodyTransform yawTransform = new RigidBodyTransform();
      TransformTools.rotate(yawTransform, valvePose.getYaw(), Axis.Z);

      double valveYaw = valvePose.getYaw();
      SysoutTool.println("Valve Pose Yaw : " + valveYaw);

      Vector3d desiredRobotPosFromValve = new Vector3d(-howFarToStandBackFromValve, -howFarToStandToTheRightOfValve, 0.0);
      Vector3d desiredRobotPosFromValveInWorld = TransformTools.getTransformedVector(desiredRobotPosFromValve, yawTransform);

      SysoutTool.println("ValveOffset in World Frame:" + desiredRobotPosFromValveInWorld, DEBUG);

      ret.setX(ret.getX() + desiredRobotPosFromValveInWorld.x);
      ret.setY(ret.getY() + desiredRobotPosFromValveInWorld.y);

      return ret;
   }

   private FramePose setGraspValveBehaviorInput(RigidBodyTransform valveTransformToWorld)
   {
      Vector3d graspApproachDirectionInValveFrame = new Vector3d(valvePinJointAxisInValveFrame);
      graspValveBehavior.setGraspPose(valveType, valveTransformToWorld, graspApproachDirectionInValveFrame, graspValveRim);

      FramePose graspPose = new FramePose();
      graspValveBehavior.getFinalGraspPose(graspPose);
      SysoutTool.println(" hand grasp pose: " + graspPose, DEBUG);
      return graspPose;
   }

   private void setHandPoseBehaviorInputs(RigidBodyTransform valveTransformToWorld, FramePose graspPose, double trajectoryTime, double turnValveAngle)
   {
      FramePose desiredHandPoseAfterFirstTurn = createHandPosePacketToTurnValve(valveType, graspPose, valveTransformToWorld, turnValveAngle, trajectoryTime);
      HandPosePacket firstTurnValveHandPosePacket = createHandPosePacket(trajectoryTime, desiredHandPoseAfterFirstTurn);
      handPoseValveTurn1Behavior.setInput(firstTurnValveHandPosePacket);

      FramePose desiredHandPoseAfterSecondTurn = createHandPosePacketToTurnValve(valveType, desiredHandPoseAfterFirstTurn, valveTransformToWorld,
            turnValveAngle, trajectoryTime);
      HandPosePacket secondTurnValveHandPosePacket = createHandPosePacket(trajectoryTime, desiredHandPoseAfterSecondTurn);
      handPoseValveTurn2Behavior.setInput(secondTurnValveHandPosePacket);
   }

   private void pauseIfCapturePointErrorIsTooLarge()
   {
      if (tippingDetected.getBooleanValue() && !isPaused.getBooleanValue())
      {
         this.pause();
         SysoutTool.println("TurnValveBehavior: Tipping detected! Pausing behavior.");

      }
   }

   private FramePose createHandPosePacketToTurnValve(ValveType valveType, FramePose handPoseBeforeTurn, RigidBodyTransform valveTransformToWorld,
         double turnAngle, double trajectoryTime)
   {
      SysoutTool.println("Hand Pose before valve turn in world frame: " + handPoseBeforeTurn);

      FramePose handPoseInValveFrame = new FramePose(handPoseBeforeTurn);
      ReferenceFrame valveFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("valve", world, valveTransformToWorld);
      handPoseInValveFrame.changeFrame(valveFrame);
      SysoutTool.println("Hand Pose before valve turn in valve frame: " + handPoseInValveFrame);

      RigidBodyTransform handTransformToValve = new RigidBodyTransform();
      handPoseInValveFrame.getPose(handTransformToValve);
      SysoutTool.println("Hand transform to valve before valve turn: " + handTransformToValve);

      RigidBodyTransform xRotation = new RigidBodyTransform();
      xRotation.rotX(turnAngle);

      handTransformToValve.multiply(xRotation);
      SysoutTool.println("Hand transform to valve after valve turn: " + handTransformToValve);

      Point3d handInValve = new Point3d();
      handPoseInValveFrame.getPosition(handInValve);
      xRotation.transform(handInValve);

      FramePose desiredHandPoseAfterTurn = new FramePose(valveFrame, handTransformToValve);
      desiredHandPoseAfterTurn.setPosition(handInValve);
      SysoutTool.println("Hand Pose after valve turn in valve frame: " + desiredHandPoseAfterTurn);

      desiredHandPoseAfterTurn.changeFrame(world);
      SysoutTool.println("Hand Pose after turn in world frame: " + desiredHandPoseAfterTurn + "\n");

      return desiredHandPoseAfterTurn;
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
      currentBehavior.passReceivedNetworkProcessorObjectToChildBehaviors(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      if (object instanceof HandPoseStatus && currentBehavior.equals(handPoseValveTurn1Behavior))
      {
         SysoutTool.println(" Received object from controller: " + object.toString() + ".  Sending to: " + currentBehavior, DEBUG);
         currentBehavior.consumeObjectFromController(object);
         currentBehavior.passReceivedControllerObjectToChildBehaviors(object);
      }
      else if (object instanceof HandPoseStatus && currentBehavior.equals(handPoseValveTurn2Behavior))
      {
         SysoutTool.println(" Received object from controller: " + object.toString() + ".  Sending to: " + currentBehavior, DEBUG);
         currentBehavior.consumeObjectFromController(object);
         currentBehavior.passReceivedControllerObjectToChildBehaviors(object);
      }
      else
      {
         currentBehavior.passReceivedControllerObjectToChildBehaviors(object);
      }
   }

   @Override
   public void stop()
   {
      currentBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      SysoutTool.println("Current Child Behavior: " + currentBehavior.getName());

      SysoutTool.println("Script Resource Stream" + scriptResourceStream);

      SysoutTool.println("max wrist force : " + maxObservedWristForce);
      SysoutTool.println("max capture point error : " + maxObservedCapturePointError);
   }

   @Override
   public void pause()
   {
      currentBehavior.pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      currentBehavior.resume();
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
      currentBehavior.finalize();
      isDone.set(false);
      hasInputBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      isDone.set(false);
      hasInputBeenSet.set(false);

      behaviorQueue.clear();
      behaviorQueue.add(walkToLocationBehavior);
      if (useScriptBehavior)
      {
         behaviorQueue.add(scriptBehavior);
      }
      else
      {
         behaviorQueue.add(comHeightBehavior);
         behaviorQueue.add(graspValveBehavior);
         behaviorQueue.add(handPoseValveTurn1Behavior);
         behaviorQueue.add(handPoseValveTurn2Behavior);
      }

      for (BehaviorInterface behavior : behaviorQueue)
      {
         behavior.initialize();
      }

      currentBehavior = behaviorQueue.remove(0);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
