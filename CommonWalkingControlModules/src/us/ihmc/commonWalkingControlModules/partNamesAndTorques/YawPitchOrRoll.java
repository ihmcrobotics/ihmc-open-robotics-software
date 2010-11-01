package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

public enum YawPitchOrRoll
{
   YAW, PITCH, ROLL;

   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
         case YAW :
         {
            return "yaw";
         }

         case PITCH :
         {
            return "pitch";
         }

         case ROLL :
         {
            return "roll";
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
         case YAW :
         {
            return "Yaw";
         }

         case PITCH :
         {
            return "Pitch";
         }

         case ROLL :
         {
            return "Roll";
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

