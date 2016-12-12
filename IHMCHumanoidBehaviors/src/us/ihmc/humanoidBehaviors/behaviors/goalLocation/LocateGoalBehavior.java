package us.ihmc.humanoidBehaviors.behaviors.goalLocation;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.tools.io.printing.PrintTools;

import javax.vecmath.Point3d;
import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicBoolean;

public class LocateGoalBehavior extends AbstractBehavior
{
   private final GoalDetectorBehaviorService detectorBehaviorService;
   private final AtomicBoolean done = new AtomicBoolean(false);
   private final FramePose foundFiducialPose = new FramePose();
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
         Point3d position = new Point3d();
         foundFiducialPose.getPosition(position);

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
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket(message);
      textToSpeechPacket.setbeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   public void getReportedGoalPoseWorldFrame(FramePose framePoseToPack)
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
