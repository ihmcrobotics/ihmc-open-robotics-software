package us.ihmc.llaQuadrupedController.model;

import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public enum LLAQuadrupedOrderedJointMap
{
   FRONT_LEFT_HIP_ROLL,
   FRONT_LEFT_HIP_PITCH,
   FRONT_LEFT_KNEE_PITCH,
   FRONT_RIGHT_HIP_ROLL,
   FRONT_RIGHT_HIP_PITCH,
   FRONT_RIGHT_KNEE_PITCH,
   HIND_LEFT_HIP_ROLL,
   HIND_LEFT_HIP_PITCH,
   HIND_LEFT_KNEE_PITCH,
   HIND_RIGHT_HIP_ROLL,
   HIND_RIGHT_HIP_PITCH,
   HIND_RIGHT_KNEE_PITCH
   ;

   public static final LLAQuadrupedOrderedJointMap[] values = values();
   private final String lowercaseName;

   private LLAQuadrupedOrderedJointMap()
   {
      lowercaseName = toString().toLowerCase();
   }
   
   public String getName()
   {
      return lowercaseName;
   }

   public static int getNumJoints()
   {
      return values.length;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      switch (this)
      {
      case FRONT_LEFT_HIP_ROLL:  
      case FRONT_LEFT_HIP_PITCH: 
      case FRONT_LEFT_KNEE_PITCH:
         return RobotQuadrant.FRONT_LEFT;
      case FRONT_RIGHT_HIP_ROLL: 
      case FRONT_RIGHT_HIP_PITCH:
      case FRONT_RIGHT_KNEE_PITCH:
         return RobotQuadrant.FRONT_RIGHT;
      case HIND_LEFT_HIP_ROLL:  
      case HIND_LEFT_HIP_PITCH: 
      case HIND_LEFT_KNEE_PITCH:
         return RobotQuadrant.HIND_LEFT;
      case HIND_RIGHT_HIP_ROLL:  
      case HIND_RIGHT_HIP_PITCH: 
      case HIND_RIGHT_KNEE_PITCH:
         return RobotQuadrant.HIND_RIGHT;
      default:
         throw new RuntimeException("RobotQuadrant undefined for " + this.name());
      }
   }
   
   public LegJointName getLegJointName()
   {
      switch (this)
      {
      case FRONT_LEFT_HIP_ROLL:  
      case FRONT_RIGHT_HIP_ROLL: 
      case HIND_LEFT_HIP_ROLL:  
      case HIND_RIGHT_HIP_ROLL:  
         return LegJointName.HIP_ROLL;
      case FRONT_LEFT_HIP_PITCH: 
      case FRONT_RIGHT_HIP_PITCH:
      case HIND_LEFT_HIP_PITCH: 
      case HIND_RIGHT_HIP_PITCH: 
         return LegJointName.HIP_PITCH;
      case FRONT_LEFT_KNEE_PITCH:
      case FRONT_RIGHT_KNEE_PITCH:
      case HIND_LEFT_KNEE_PITCH:
      case HIND_RIGHT_KNEE_PITCH:
         return LegJointName.KNEE_PITCH;
      default:
         return null;
      }
   }
}
