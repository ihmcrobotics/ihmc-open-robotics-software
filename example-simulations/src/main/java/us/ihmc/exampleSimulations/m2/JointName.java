package us.ihmc.exampleSimulations.m2;

public enum JointName
{
   HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE, ANKLE_PITCH, ANKLE_ROLL;

   public String getShortName()
   {
      switch (this)
      {
         case HIP_YAW :
         {
            return "hip_yaw";
         }

         case HIP_ROLL :
         {
            return "hip_roll";
         }

         case HIP_PITCH :
         {
            return "hip_pitch";
         }

         case KNEE :
         {
            return "knee";
         }

         case ANKLE_PITCH :
         {
            return "ankle_pitch";
         }

         case ANKLE_ROLL :
         {
            return "ankle_roll";
         }

         default :
         {
            throw new RuntimeException("Should not get to here");
         }
      }
   }
}
