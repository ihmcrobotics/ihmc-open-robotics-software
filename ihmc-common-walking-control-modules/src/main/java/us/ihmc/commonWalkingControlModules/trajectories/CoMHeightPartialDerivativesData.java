package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CoMHeightPartialDerivativesData
{
   private ReferenceFrame frameOfCoMHeight;
   private double comHeight, partialDzDx, partialDzDy, partialD2zDx2, partialD2zDy2, partialD2zDxDy;

   public void set(CoMHeightPartialDerivativesData centerOfMassHeightPartialDerivativesData)
   {
      setCoMHeight(centerOfMassHeightPartialDerivativesData.getFrameOfCoMHeight(), centerOfMassHeightPartialDerivativesData.getComHeight());
      setPartialDzDx(centerOfMassHeightPartialDerivativesData.getPartialDzDx());
      setPartialDzDy(centerOfMassHeightPartialDerivativesData.getPartialDzDy());
      setPartialD2zDx2(centerOfMassHeightPartialDerivativesData.getPartialD2zDx2());
      setPartialD2zDy2(centerOfMassHeightPartialDerivativesData.getPartialD2zDy2());
      setPartialD2zDxDy(centerOfMassHeightPartialDerivativesData.getPartialD2zDxDy());
   }

   public ReferenceFrame getFrameOfCoMHeight()
   {
      return frameOfCoMHeight;
   }

   public double getComHeight()
   {
      return comHeight;
   }

   public void setCoMHeight(ReferenceFrame referenceFrame, double comHeight)
   {
      this.frameOfCoMHeight = referenceFrame;
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
