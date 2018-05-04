package us.ihmc.humanoidBehaviors.behaviors.goalLocation;

import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicBoolean;

import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

public class LocateGoalBehavior extends AbstractBehavior
{
   private final GoalDetectorBehaviorService detectorBehaviorService;
   private final AtomicBoolean done = new AtomicBoolean(false);
   private final FramePose3D foundFiducialPose = new FramePose3D();
   private final DecimalFormat decimalFormat = new DecimalFormat("#.##");

   public LocateGoalBehavior(CommunicationBridgeInterface communicationBridge, GoalDetectorBehaviorService detectorBehaviorService)
   {
      super(detectorBehaviorService.getClass().getSimpleName(), communicationBridge);

      this.detectorBehaviorService = detectorBehaviorService;
      addBehaviorService(detectorBehaviorService);
   }

   @Override
   public void doControl()
   {
      if (detectorBehaviorService.getGoalHasBeenLocated())
      {
         detectorBehaviorService.getReportedGoalPoseWorldFrame(foundFiducialPose);
         Point3D position = new Point3D(foundFiducialPose.getPosition());

         double x = position.getX(), y = position.getY(), z = position.getZ();
         double yaw = Math.toDegrees(foundFiducialPose.getYaw()), pitch = Math.toDegrees(foundFiducialPose.getPitch()), roll = Math.toDegrees(foundFiducialPose.getRoll());

         sendTextToSpeechPacket(String.format("Target object located at [%s, %s, %s], rotation (%s, %s, %s)",
                                              decimalFormat.format(x),
                                              decimalFormat.format(y),
                                              decimalFormat.format(z),
                                              decimalFormat.format(yaw),
                                              decimalFormat.format(pitch),
                                              decimalFormat.format(roll)));
         done.set(true);
      }
      else
      {
         sendTextToSpeechPacket("Target object not located");
         done.set(false);
      }
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = MessageTools.createTextToSpeechPacket(message);
      textToSpeechPacket.setBeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   public void getReportedGoalPoseWorldFrame(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(foundFiducialPose);
   }

   @Override
   public boolean isDone()
   {
      return done.get();
   }

   @Override
   public void onBehaviorEntered()
   {
      detectorBehaviorService.initialize();
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }
}
