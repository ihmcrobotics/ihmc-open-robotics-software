package us.ihmc.humanoidBehaviors.behaviors;

//~--- non-JDK imports --------------------------------------------------------

//~--- JDK imports ------------------------------------------------------------
import java.io.InputStream;
import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
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
   private static final boolean DEBUG = true;

   public static final RobotSide handToTurnValveWith = RobotSide.RIGHT;
   public static final double howFarToStandToTheRightOfValve = handToTurnValveWith.negateIfRightSide(0.13);
   public static final double howFarToStandBackFromValve = 0.64;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final FramePose valvePose = new FramePose();

   private final WalkToLocationBehavior walkToLocationBehavior;
   private final GraspValveBehavior graspObjectBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final HandPoseBehavior handPoseBehavior2;

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

      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      graspObjectBehavior = new GraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      handPoseBehavior2 = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);

      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport);

      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.isDone = new BooleanYoVariable("isDone", registry);
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
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
               currentBehavior.initialize();
               SysoutTool.println(currentBehavior.getName() + " is starting.", DEBUG);
            }
            else
            {
               //               handPitchAngle += 10.0 * Math.PI / 180.0;
               //               HandPosePacket handPosePacket = createHandPosePacketToBumpTheValveToVerifyItsPosition(valvePose, handPitchAngle, 2.0);
               //               handPoseBehavior.initialize();
               //               handPoseBehavior.setInput(handPosePacket);
               //
               //               SysoutTool.println("hand pitch angle :" + handPitchAngle);
               //
               //               currentBehavior = handPoseBehavior;

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
      scriptBehavior.importScriptInputPacket(scriptBehaviorInputPacket);
      SysoutTool.println("New Script Behavior Input Packet Received.  Script File : " + scriptBehaviorInputPacket.getScriptName(), DEBUG);

      RigidBodyTransform valveTransformToWorld = scriptBehaviorInputPacket.getReferenceTransform();

      valvePose.setPoseIncludingFrame(world, valveTransformToWorld);
      SysoutTool.println("Turn Valve Location Updated:" + valvePose, DEBUG);

      FramePose2d manipulationMidFeetPose = computeDesiredMidFeetPoseForManipulation(valvePose);
      walkToLocationBehavior.setTarget(manipulationMidFeetPose);
      SysoutTool.println("Target Walk to Location Updated:" + manipulationMidFeetPose, DEBUG);

      Vector3d finalToMidGraspVec = new Vector3d(-1, 0, 0);
      graspObjectBehavior.setGraspPose(valveTransformToWorld, finalToMidGraspVec);

      FramePose graspPose = new FramePose();
      graspObjectBehavior.getFinalGraspPose(graspPose);

      double trajectoryTime = 2.0;
      double turnValveAngle = -10.0 * Math.PI / 180.0;
      HandPosePacket turnValveHandPosePacket = createHandPosePacketToTurnValve(graspPose, turnValveAngle, trajectoryTime);
      handPoseBehavior.setInput(turnValveHandPosePacket);

      HandPosePacket turnValveHandPosePacket2 = createHandPosePacketToTurnValve(graspPose, turnValveAngle * 2, trajectoryTime);
      handPoseBehavior2.setInput(turnValveHandPosePacket2);

      hasInputBeenSet.set(true);

      for (BehaviorInterface behavior : behaviorQueue)
      {
         if (!behavior.hasInputBeenSet())
         {
            hasInputBeenSet.set(false);
         }
      }
   }

   private double handPitchAngle = 0.0;

   private void pauseIfCapturePointErrorIsTooLarge()
   {
      if (tippingDetected.getBooleanValue() && !isPaused.getBooleanValue())
      {
         this.pause();
         SysoutTool.println("TurnValveBehavior: Tipping detected! Pausing behavior.");

      }
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

   private HandPosePacket createHandPosePacketToTurnValve(FramePose handPoseBeforeTurn, double turnAngle, double trajectoryTime)
   {
      AxisAngle4d handAxisAngleBeforeTurn = new AxisAngle4d();
      handPoseBeforeTurn.getOrientation(handAxisAngleBeforeTurn);

      AxisAngle4d handAxisAngleAfterTurn = new AxisAngle4d();
      handAxisAngleAfterTurn.setAngle(handAxisAngleBeforeTurn.angle + turnAngle);

      FramePose handPoseAfterTurn = new FramePose(handPoseBeforeTurn);
      handPoseAfterTurn.setOrientation(handAxisAngleAfterTurn);

      HandPosePacket ret = createHandPosePacket(trajectoryTime, handPoseAfterTurn);

      return ret;
   }

   private HandPosePacket createHandPosePacket(double trajectoryTime, FramePose desiredHandPose)
   {
      Point3d position = new Point3d();
      Quat4d orientation = new Quat4d();

      desiredHandPose.getPosition(position);
      desiredHandPose.getOrientation(orientation);

      HandPosePacket ret = new HandPosePacket(RobotSide.RIGHT, Frame.WORLD, position, orientation, trajectoryTime);
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
      currentBehavior.passReceivedControllerObjectToChildBehaviors(object);
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
      if (scriptBehavior != null)
      {
         scriptBehavior.initialize();
      }
      if (walkToLocationBehavior != null)
      {
         walkToLocationBehavior.initialize();
      }
      if (handPoseBehavior != null)
      {
         handPoseBehavior.initialize();
      }

      behaviorQueue.clear();
      behaviorQueue.add(walkToLocationBehavior);
      behaviorQueue.add(graspObjectBehavior);
      behaviorQueue.add(handPoseBehavior);
      behaviorQueue.add(handPoseBehavior2);

      //      behaviorQueue.add(handPoseBehavior);
      //      behaviorQueue.add(scriptBehavior);
      currentBehavior = behaviorQueue.remove(0);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
