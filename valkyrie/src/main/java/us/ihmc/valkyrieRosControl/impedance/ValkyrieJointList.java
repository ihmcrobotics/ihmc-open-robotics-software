package us.ihmc.valkyrieRosControl.impedance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ValkyrieJointList
{
   static final List<String> allJoints = new ArrayList<>();
   static final List<String> torqueJoints = new ArrayList<>();
   static final List<String> impedanceJoints = new ArrayList<>();

   static
   {
      // Arms
      allJoints.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
      allJoints.addAll(Arrays.asList("rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"));

      // Torso
      allJoints.addAll(Arrays.asList("torsoYaw", "torsoPitch", "torsoRoll"));

      // Legs
      allJoints.addAll(Arrays.asList("leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"));
      allJoints.addAll(Arrays.asList("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"));

      torqueJoints.add("torsoPitch");
      torqueJoints.add("torsoRoll");
      torqueJoints.add("leftAnklePitch");
      torqueJoints.add("leftAnkleRoll");
      torqueJoints.add("rightAnklePitch");
      torqueJoints.add("rightAnkleRoll");

      for (String joint : allJoints)
      {
         if (!torqueJoints.contains(joint))
         {
            impedanceJoints.add(joint);
         }
      }
   }

   public enum ValkyrieImpedanceJoint
   {
      leftShoulderPitch, leftShoulderRoll, leftShoulderYaw, leftElbowPitch, rightShoulderPitch, rightShoulderRoll, rightShoulderYaw, rightElbowPitch, torsoYaw, leftHipYaw, leftHipRoll, leftHipPitch, leftKneePitch, rightHipYaw, rightHipRoll, rightHipPitch, rightKneePitch
   }
}
