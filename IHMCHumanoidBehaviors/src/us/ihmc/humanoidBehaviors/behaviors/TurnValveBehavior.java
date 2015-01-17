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
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class TurnValveBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = true;
   private final Vector3d valveInteractionOffsetInValveFrame = new Vector3d(-0.13, 0.0, 0.64);
   private Vector3d valveLocation = new Vector3d();
   private Vector3d valveOffsetInWorldFrame = new Vector3d();
   private final ArrayList<BehaviorInterface> behaviorQueue = new ArrayList<>();
   private final ScriptBehavior scriptBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;

   private final HandPoseBehavior handPoseBehavior;

   private Point3d targetWalkLocation;
   private final YoFrameOrientation targetWalkOrientation;
   private InputStream scriptResourceStream;
   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
   private RigidBodyTransform worldToValveTransform;
   private YoFrameOrientation valveOrientation;
   private BehaviorInterface currentBehavior;
   private boolean isDone;

   private DoubleYoVariable rightWristForceFiltered;
   private double maxObservedWristForce = 0.0;

   private Double maxObservedCapturePointError = 0.0;
   private BooleanYoVariable tippingDetected;

   // private final ModifiableValveModel valveModel;

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, BooleanYoVariable tippingDetectedBoolean,
         WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);
      targetWalkLocation = new Point3d();
      valveOrientation = new YoFrameOrientation(behaviorName + "ValveOrientation", ReferenceFrame.getWorldFrame(), registry);
      targetWalkOrientation = new YoFrameOrientation(behaviorName + "WalkToOrientation", ReferenceFrame.getWorldFrame(), registry);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();
      this.tippingDetected = tippingDetectedBoolean;
      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
   }

   @Override
   public void doControl()
   {
      //      if (!currentBehavior.equals(walkToLocationBehavior))
      //      {
      pauseBehaviorIfCapturePointErrorIsTooLarge();
      //      }

      if (scriptBehaviorInputPacketListener.isNewPacketAvailable())
      {
         ScriptBehaviorInputPacket scriptBehaviorInputPacket = scriptBehaviorInputPacketListener.getNewestPacket();
         System.out.println("TurnValveBehavior: New Script Behavior Input Packet Received: " + scriptBehaviorInputPacket);

         worldToValveTransform = scriptBehaviorInputPacket.getReferenceTransform();

         setValveLocationAndOrientation(worldToValveTransform);
         setTargetWalkLocationAndOrientation(valveLocation, valveOrientation);

         HandPosePacket bumpValveHandPosePacket = createHandPosePacketToBumpTheValveToVerifyItsPosition(worldToValveTransform);
         handPoseBehavior.setInput(bumpValveHandPosePacket);
         //       handPoseBehavior.setInput(Frame.WORLD, worldToValveTransformWithYawCorrection, RobotSide.RIGHT, 1.0);

         String scriptName = scriptBehaviorInputPacket.getScriptName();

         scriptResourceStream = this.getClass().getClassLoader().getResourceAsStream(scriptName);

         if (scriptResourceStream == null)
         {
            throw new RuntimeException("TurnValveBehavior: Script Resource Stream is null. Can't load script!  scriptResourceStream : " + scriptResourceStream);
         }
         else
         {
            System.out.println("TurnValveBehavior: Script " + scriptBehaviorInputPacket.getScriptName() + " loaded.");
         }

         scriptBehavior.importChildInputPackets(scriptName, scriptResourceStream, worldToValveTransform);
      }

      if (currentBehavior.isDone())
      {
         System.out.println("TurnValveBehavior: " + currentBehavior.getName() + " is done.");
         currentBehavior.finalize();
         if (!behaviorQueue.isEmpty())
         {
            currentBehavior = behaviorQueue.remove(0);
            currentBehavior.initialize();
            System.out.println("TurnValveBehavior: " + currentBehavior.getName() + " is starting.");
         }
         else
         {
            isDone = true;
         }
      }

      currentBehavior.doControl();
   }

   private HandPosePacket createHandPosePacketToBumpTheValveToVerifyItsPosition(RigidBodyTransform worldToValveTransform)
   {
      RigidBodyTransform worldToValveTransformWithYawCorrection = new RigidBodyTransform();
      worldToValveTransformWithYawCorrection.set(worldToValveTransform);

      Vector3d eulerAngles = new Vector3d();
      worldToValveTransformWithYawCorrection.getEulerXYZ(eulerAngles);
      eulerAngles.setZ(eulerAngles.z + 0.5 * Math.PI);

      Vector3d handBumpPosition = new Vector3d();
      Point3d handBumpPositionPt = new Point3d();
      worldToValveTransform.get(handBumpPosition);
      handBumpPositionPt.set(handBumpPosition);

      Quat4d orientation = new Quat4d();
      worldToValveTransform.get(orientation);
      orientation.set(1, 0, 0, 0);

      HandPosePacket ret = new HandPosePacket(RobotSide.RIGHT, Frame.WORLD, handBumpPositionPt, orientation, 1.0);

      return ret;
   }

   private void pauseBehaviorIfCapturePointErrorIsTooLarge()
   {
      if (tippingDetected.getBooleanValue() && !isPaused.getBooleanValue())
      {
         this.pause();
         if (DEBUG)
         {
            System.out.println("TurnValveBehavior: Tipping detected! Pausing behavior.");
         }
      }
   }

   private void setValveLocationAndOrientation(RigidBodyTransform worldToValveTransform)
   {
      worldToValveTransform.get(valveLocation);
      valveOrientation.set(worldToValveTransform);
   }

   private void setTargetWalkLocationAndOrientation(Vector3d valveLocation, YoFrameOrientation valveOrientation)
   {
      targetWalkLocation.set(valveLocation);
      worldToValveTransform.transform(valveInteractionOffsetInValveFrame, valveOffsetInWorldFrame);
      targetWalkLocation.add(valveOffsetInWorldFrame);
      targetWalkOrientation.setYawPitchRoll(valveOrientation.getYaw().getDoubleValue() + 0.5 * Math.PI, valveOrientation.getPitch().getDoubleValue(),
            valveOrientation.getRoll().getDoubleValue());
      walkToLocationBehavior.setTarget(targetWalkLocation, targetWalkOrientation);

      System.out.println("TurnValveBehavior:  ValveOffset in World Frame:" + valveOffsetInWorldFrame);
      System.out.println("TurnValveBehavior: Turn Valve Location Updated:" + valveLocation);
      System.out.println("TurnValveBehavior: Target Walk to Location Updated:" + targetWalkLocation);
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
      System.out.println("TurnValveBehavior: Current Child Behavior: " + currentBehavior.getName());

      System.out.println("TurnValveBehavior: Script Resource Stream" + scriptResourceStream);

      System.out.println("TurnValveBehavior: max wrist force : " + maxObservedWristForce);
      System.out.println("TurnValveBehavior: max capture point error : " + maxObservedCapturePointError);

      System.out.println("TurnValveBehavior: Valve Location: " + valveLocation);
      System.out.println("TurnValveBehavior: ValveOffset in World Frame: " + valveOffsetInWorldFrame);
      System.out.println("TurnValveBehavior: Target Walk to Location: " + targetWalkLocation);
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
      return isDone;
   }

   @Override
   public void finalize()
   {
      currentBehavior.finalize();
   }

   @Override
   public void initialize()
   {
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
      //      behaviorQueue.add(handPoseBehavior);
      behaviorQueue.add(scriptBehavior);
      currentBehavior = behaviorQueue.remove(0);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return currentBehavior.hasInputBeenSet();
   }
}
