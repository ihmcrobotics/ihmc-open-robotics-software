package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

public enum ContactMotion
{
   CONSTANT(3), LINEAR(4);

   private final int numberOfCoefficients;
   private ContactMotion(int numberOfCoefficients)
   {
      this.numberOfCoefficients = numberOfCoefficients;
   }

   public int getNumberOfCoefficients()
   {
      return numberOfCoefficients;
   }

}
