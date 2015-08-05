package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

public class SimpleTranslationalPathParameters implements TranslationalPathParameters
{
   private double forwardStepLength, backwardStepLength, sidewardStepLength, nominalStepWidth, minimumStepWidth;

   public SimpleTranslationalPathParameters(double forwardStepLength, double backwardStepLength, double sidewardStepLength, double nominalStepWidth,
           double minimumStepWidth)
   {
      this.forwardStepLength = forwardStepLength;
      this.backwardStepLength = backwardStepLength;
      this.sidewardStepLength = sidewardStepLength;
      this.nominalStepWidth = nominalStepWidth;
      this.minimumStepWidth = minimumStepWidth;
   }

   @Override
   public double getForwardStepLength()
   {
      return forwardStepLength;
   }

   @Override
   public double getBackwardStepLength()
   {
      return backwardStepLength;
   }

   @Override
   public double getSidewardStepLength()
   {
      return sidewardStepLength;
   }

   @Override
   public double getNominalStepWidth()
   {
      return nominalStepWidth;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minimumStepWidth;
   }

   public void setForwardStepLength(double forwardStepLength)
   {
      this.forwardStepLength = forwardStepLength;
   }

   public void setBackwardStepLength(double backwardStepLength)
   {
      this.backwardStepLength = backwardStepLength;
   }

   public void setSidewardStepLength(double sidewardStepLength)
   {
      this.sidewardStepLength = sidewardStepLength;
   }

   public void setNominalStepWidth(double nominalStepWidth)
   {
      this.nominalStepWidth = nominalStepWidth;
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth = minimumStepWidth;
   }

}
