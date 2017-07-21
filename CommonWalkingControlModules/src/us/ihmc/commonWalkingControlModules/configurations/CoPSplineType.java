package us.ihmc.commonWalkingControlModules.configurations;

public enum CoPSplineType
{
   LINEAR, CUBIC;

   public int getNumberOfCoefficients()
   {
      switch (this)
      {
      case CUBIC: return 4;
      default: return 2;
      }
   } 
}
