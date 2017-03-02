package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSextant;

public enum RhinoBeetleJointName
{
   FRONT_RIGHT_YAW(RobotSextant.FRONT_RIGHT, LegJointName.HIP_YAW),
   FRONT_RIGHT_UPPER_PITCH(RobotSextant.FRONT_RIGHT, LegJointName.HIP_PITCH),
   FRONT_RIGHT_LOWER_PITCH(RobotSextant.FRONT_RIGHT, LegJointName.KNEE_PITCH),
   
   MIDDLE_RIGHT_YAW(RobotSextant.MIDDLE_RIGHT, LegJointName.HIP_YAW),
   MIDDLE_RIGHT_UPPER_PITCH(RobotSextant.MIDDLE_RIGHT, LegJointName.HIP_PITCH),
   MIDDLE_RIGHT_LOWER_PITCH(RobotSextant.MIDDLE_RIGHT, LegJointName.KNEE_PITCH),
   
   HIND_RIGHT_YAW(RobotSextant.HIND_RIGHT, LegJointName.HIP_YAW),
   HIND_RIGHT_UPPER_PITCH(RobotSextant.HIND_RIGHT, LegJointName.HIP_PITCH),
   HIND_RIGHT_LOWER_PITCH(RobotSextant.HIND_RIGHT, LegJointName.KNEE_PITCH),
   
   FRONT_LEFT_YAW(RobotSextant.FRONT_LEFT, LegJointName.HIP_YAW),
   FRONT_LEFT_UPPER_PITCH(RobotSextant.FRONT_LEFT, LegJointName.HIP_PITCH),
   FRONT_LEFT_LOWER_PITCH(RobotSextant.FRONT_LEFT, LegJointName.KNEE_PITCH),
   
   MIDDLE_LEFT_YAW(RobotSextant.MIDDLE_LEFT, LegJointName.HIP_YAW),
   MIDDLE_LEFT_UPPER_PITCH(RobotSextant.MIDDLE_LEFT, LegJointName.HIP_PITCH),
   MIDDLE_LEFT_LOWER_PITCH(RobotSextant.MIDDLE_LEFT, LegJointName.KNEE_PITCH),
   
   HIND_LEFT_YAW(RobotSextant.HIND_LEFT, LegJointName.HIP_YAW),
   HIND_LEFT_UPPER_PITCH(RobotSextant.HIND_LEFT, LegJointName.HIP_PITCH),
   HIND_LEFT_LOWER_PITCH(RobotSextant.HIND_LEFT, LegJointName.KNEE_PITCH);
   
   public static RhinoBeetleJointName[] values = values();
   private final RobotSextant sextant;
   private final LegJointName legJointName;
   private final String name;
   
   private RhinoBeetleJointName(RobotSextant sextant, LegJointName legJointName)
   {
      this.sextant = sextant;
      this.legJointName = legJointName;
      this.name = this.toString();
   }
   
   public RobotSextant getSextant()
   {
      return sextant;
   }
   
   public LegJointName getLegJointName()
   {
      return legJointName;
   }

   public String getName()
   {
      return name;
   }
}
