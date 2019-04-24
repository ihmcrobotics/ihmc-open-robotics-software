package us.ihmc.humanoidBehaviors.behaviors.goalLocation;

import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;

public class LocateGoalBehavior extends AbstractBehavior
{
   private final GoalDetectorBehaviorService detectorBehaviorService;
   private final AtomicBoolean done = new AtomicBoolean(false);
   private final FramePose3D foundFiducialPose = new FramePose3D();
   private final DecimalFormat decimalFormat = new DecimalFormat("#.##");

   public LocateGoalBehavior(String robotName, Ros2Node ros2Node, GoalDetectorBehaviorService detectorBehaviorService)
   {
      super(robotName, detectorBehaviorService.getClass().getSimpleName(), ros2Node);

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
         double yaw = Math.toDegrees(foundFiducialPose.getYaw()), pitch = Math.toDegrees(foundFiducialPose.getPitch()),
               roll = Math.toDegrees(foundFiducialPose.getRoll());

         //publishTextToSpeech(String.format("Target object located at [%s, %s, %s], rotation (%s, %s, %s)", decimalFormat.format(x), decimalFormat.format(y),
         //                                  decimalFormat.format(z), decimalFormat.format(yaw), decimalFormat.format(pitch), decimalFormat.format(roll)));
         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         foundFiducialPose.get(location, orientation);
         publishUIPositionCheckerPacket(location, orientation);

         
         done.set(true);
      }
      else
      {
         //publishTextToSpeech("Target object not located");
         done.set(false);
      }
   }

   public void getReportedGoalPoseWorldFrame(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(foundFiducialPose);
   }

   @Override
   public boolean isDone(double timeinState)
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
