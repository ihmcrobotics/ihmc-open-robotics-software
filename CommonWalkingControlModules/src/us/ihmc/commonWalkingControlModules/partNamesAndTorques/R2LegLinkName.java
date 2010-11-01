package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

public enum R2LegLinkName
{
   UPPER_THIGH, LOWER_THIGH, SHIN, FOOT;

   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
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
