package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

public enum R2LegLinkName
{
   HIP_PITCH_LINK, UPPER_THIGH, LOWER_THIGH, SHIN, ANKLE_ROLL_LINK, FOOT;

   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
         case HIP_PITCH_LINK :
         {
            return "hipPitchLink";
         }
         
         case UPPER_THIGH :
         {
            return "upperThigh";
         }

         case LOWER_THIGH :
         {
            return "lowerThigh";
         }

         case SHIN :
         {
            return "shin";
         }
         
         case ANKLE_ROLL_LINK :
         {
            return "ankleRollLink";
         }

         case FOOT :
         {
            return "foot";
         }

         default :
         {
            throw new RuntimeException("Should not get to here");
         }
      }
   }


   public String getCamelCaseNameForMiddleOfExpression()
   {
      switch (this)
      {
         case HIP_PITCH_LINK :
         {
            return "HipPitchLink";
         }
      
         case UPPER_THIGH :
         {
            return "UpperThigh";
         }

         case LOWER_THIGH :
         {
            return "LowerThigh";
         }

         case SHIN :
         {
            return "Shin";
         }
         
         case ANKLE_ROLL_LINK :
         {
            return "AnkleRollLink";
         }

         case FOOT :
         {
            return "Foot";
         }

         default :
         {
            throw new RuntimeException("Should not get to here");
         }
      }
   }


   public String toString()
   {
      return getCamelCaseNameForMiddleOfExpression();
   }
}
