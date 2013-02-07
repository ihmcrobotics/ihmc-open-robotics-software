package us.ihmc.commonWalkingControlModules.trajectories;

public class CenterOfMassHeightPartialDerivativesData
{
   private double comHeight, partialDzDx, partialDzDy, partialD2zDx2, partialD2zDy2, partialD2zDxDy;

   public void set(CenterOfMassHeightPartialDerivativesData centerOfMassHeightPartialDerivativesData)
   {
      this.comHeight = centerOfMassHeightPartialDerivativesData.comHeight; 
      this.partialDzDx = centerOfMassHeightPartialDerivativesData.partialDzDx; 
      this.partialDzDy = centerOfMassHeightPartialDerivativesData.partialDzDy;
      this.partialD2zDx2 = centerOfMassHeightPartialDerivativesData.partialD2zDx2; 
      this.partialD2zDy2 = centerOfMassHeightPartialDerivativesData.partialD2zDy2; 
      this.partialD2zDxDy = centerOfMassHeightPartialDerivativesData.partialD2zDxDy;
   }
   
   public double getCoMHeight()
   {
      return comHeight;
   }
   
   public void setCoMHeight(double comHeight)
   {
      this.comHeight = comHeight;
   }
   
   public double getPartialDzDx()
   {
      return partialDzDx;
   }

   public void setPartialDzDx(double partialDzDx)
   {
      this.partialDzDx = partialDzDx;
   }

   public double getPartialDzDy()
   {
      return partialDzDy;
   }

   public void setPartialDzDy(double partialDzDy)
   {
      this.partialDzDy = partialDzDy;
   }

   public double getPartialD2zDx2()
   {
      return partialD2zDx2;
   }

   public void setPartialD2zDx2(double partialD2zDx2)
   {
      this.partialD2zDx2 = partialD2zDx2;
   }

   public double getPartialD2zDy2()
   {
      return partialD2zDy2;
   }

   public void setPartialD2zDy2(double partialD2zDy2)
   {
      this.partialD2zDy2 = partialD2zDy2;
   }

   public double getPartialD2zDxDy()
   {
      return partialD2zDxDy;
   }

   public void setPartialD2zDxDy(double partialD2zDxDy)
   {
      this.partialD2zDxDy = partialD2zDxDy;
   }

   
}
