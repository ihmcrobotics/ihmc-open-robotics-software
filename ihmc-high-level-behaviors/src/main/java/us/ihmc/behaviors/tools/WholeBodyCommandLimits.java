package us.ihmc.behaviors.tools;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Limits on the pelvis height and torso orientation that operators and behaviors may command. A robot's RL
 * whole-body policy is only trained over a range of these commands, and outside it the policy does what it
 * was never taught to do. Teleoperation and the behavior tree both clamp to these, so there is one place to
 * change them, in {@code WholeBodyCommandLimits<RobotName>.json} next to this class on the classpath.
 * <p>
 * Both are in the walking frame, the "Walking" frame of the behavior tree, which is on the ground under the
 * mid-feet and faces where the robot walks. The pelvis height is the pelvis z in it, and the torso angles are
 * the chest yaw, pitch and roll in it. Commands in any other frame are clamped in the walking frame.
 * </p>
 */
public class WholeBodyCommandLimits extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey pelvisMinimumHeight = keys.addDoubleKey("Pelvis minimum height", 0.6);
   public static final DoubleStoredPropertyKey pelvisMaximumHeight = keys.addDoubleKey("Pelvis maximum height", 1.2);
   /** The height a pelvis home command goes to. It has to match the controller's, like the RL height manager's home height. */
   public static final DoubleStoredPropertyKey pelvisHomeHeight = keys.addDoubleKey("Pelvis home height", 0.91);
   public static final DoubleStoredPropertyKey torsoMinimumRoll = keys.addDoubleKey("Torso minimum roll", -0.5);
   public static final DoubleStoredPropertyKey torsoMaximumRoll = keys.addDoubleKey("Torso maximum roll", 0.5);
   public static final DoubleStoredPropertyKey torsoMinimumPitch = keys.addDoubleKey("Torso minimum pitch", -0.5);
   public static final DoubleStoredPropertyKey torsoMaximumPitch = keys.addDoubleKey("Torso maximum pitch", 0.5);
   public static final DoubleStoredPropertyKey torsoMinimumYaw = keys.addDoubleKey("Torso minimum yaw", -1.0);
   public static final DoubleStoredPropertyKey torsoMaximumYaw = keys.addDoubleKey("Torso maximum yaw", 1.0);

   private static final Map<String, WholeBodyCommandLimits> loadedLimits = new ConcurrentHashMap<>();

   /** The limits for the robot, loaded once and shared, for code that only reads them, like behavior tree actions. */
   public static WholeBodyCommandLimits forRobot(String robotName)
   {
      return loadedLimits.computeIfAbsent(robotName, WholeBodyCommandLimits::new);
   }

   /** Loads the limits for the robot, falling back to the defaults when the robot has no file. */
   public WholeBodyCommandLimits(String robotName)
   {
      super(keys, WholeBodyCommandLimits.class, StringUtils.capitalize(robotName));
      load();
   }

   public double getPelvisMinimumHeight()
   {
      return get(pelvisMinimumHeight);
   }

   public double getPelvisMaximumHeight()
   {
      return get(pelvisMaximumHeight);
   }

   public double getPelvisHomeHeight()
   {
      return get(pelvisHomeHeight);
   }

   public double getTorsoMinimumRoll()
   {
      return get(torsoMinimumRoll);
   }

   public double getTorsoMaximumRoll()
   {
      return get(torsoMaximumRoll);
   }

   public double getTorsoMinimumPitch()
   {
      return get(torsoMinimumPitch);
   }

   public double getTorsoMaximumPitch()
   {
      return get(torsoMaximumPitch);
   }

   public double getTorsoMinimumYaw()
   {
      return get(torsoMinimumYaw);
   }

   public double getTorsoMaximumYaw()
   {
      return get(torsoMaximumYaw);
   }

   /** @param height the pelvis height in the walking frame */
   public double clampPelvisHeight(double height)
   {
      return MathTools.clamp(height, getPelvisMinimumHeight(), getPelvisMaximumHeight());
   }

   public double clampTorsoRoll(double roll)
   {
      return MathTools.clamp(roll, getTorsoMinimumRoll(), getTorsoMaximumRoll());
   }

   public double clampTorsoPitch(double pitch)
   {
      return MathTools.clamp(pitch, getTorsoMinimumPitch(), getTorsoMaximumPitch());
   }

   public double clampTorsoYaw(double yaw)
   {
      return MathTools.clamp(yaw, getTorsoMinimumYaw(), getTorsoMaximumYaw());
   }

   /**
    * Clamps a desired pelvis pose to the height limits, in place. The height is clamped in the walking frame, then
    * the pose goes back to the frame it came in.
    *
    * @return whether the height was outside the limits
    */
   public boolean clampPelvisHeight(FramePose3DBasics pelvisPose, ReferenceFrame walkingFrame)
   {
      FramePose3D poseInWalkingFrame = new FramePose3D(pelvisPose);
      poseInWalkingFrame.changeFrame(walkingFrame);
      double clampedHeight = clampPelvisHeight(poseInWalkingFrame.getZ());
      if (clampedHeight == poseInWalkingFrame.getZ())
         return false;

      poseInWalkingFrame.setZ(clampedHeight);
      poseInWalkingFrame.changeFrame(pelvisPose.getReferenceFrame());
      pelvisPose.set(poseInWalkingFrame);
      return true;
   }

   /**
    * Clamps a desired chest orientation to the torso limits, in place. The yaw, pitch and roll are clamped in the
    * walking frame, then the orientation goes back to the frame it came in.
    *
    * @return whether the orientation was outside the limits
    */
   public boolean clampTorsoOrientation(FrameOrientation3DBasics chestOrientation, ReferenceFrame walkingFrame)
   {
      FrameQuaternion orientationInWalkingFrame = new FrameQuaternion(chestOrientation);
      orientationInWalkingFrame.changeFrame(walkingFrame);
      double yaw = orientationInWalkingFrame.getYaw();
      double pitch = orientationInWalkingFrame.getPitch();
      double roll = orientationInWalkingFrame.getRoll();
      double clampedYaw = clampTorsoYaw(yaw);
      double clampedPitch = clampTorsoPitch(pitch);
      double clampedRoll = clampTorsoRoll(roll);
      if (clampedYaw == yaw && clampedPitch == pitch && clampedRoll == roll)
         return false;

      orientationInWalkingFrame.setYawPitchRoll(clampedYaw, clampedPitch, clampedRoll);
      orientationInWalkingFrame.changeFrame(chestOrientation.getReferenceFrame());
      chestOrientation.set(orientationInWalkingFrame);
      return true;
   }
}
