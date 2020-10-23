package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CoMHeightPartialDerivativesData implements CoMHeightPartialDerivativesDataBasics
{
   private ReferenceFrame frameOfCoMHeight;
   private double comHeight;
   private double partialDzDx, partialDzDy;
   private double partialD2zDx2, partialD2zDy2, partialD2zDxDy;
   private double partialD3zDx3, partialD3zDy3, partialD3zDx2Dy, partialD3zDxDy2;

   public ReferenceFrame getFrameOfCoMHeight()
   {
      return frameOfCoMHeight;
   }

   public double getComHeight()
   {
      return comHeight;
   }

   public double getPartialDzDx()
   {
      return partialDzDx;
   }

   public double getPartialDzDy()
   {
      return partialDzDy;
   }

   public double getPartialD2zDx2()
   {
      return partialD2zDx2;
   }

   public double getPartialD2zDy2()
   {
      return partialD2zDy2;
   }

   public double getPartialD2zDxDy()
   {
      return partialD2zDxDy;
   }

   public double getPartialD3zDx3()
   {
      return partialD3zDx3;
   }

   public double getPartialD3zDy3()
   {
      return partialD3zDy3;
   }

   public double getPartialD3zDx2Dy()
   {
      return partialD3zDx2Dy;
   }

   public double getPartialD3zDxDy2()
   {
      return partialD3zDxDy2;
   }

   public void setCoMHeight(ReferenceFrame referenceFrame, double comHeight)
   {
      this.frameOfCoMHeight = referenceFrame;
      this.comHeight = comHeight;
   }

   public void setPartialDzDx(double partialDzDx)
   {
      this.partialDzDx = partialDzDx;
   }

   public void setPartialDzDy(double partialDzDy)
   {
      this.partialDzDy = partialDzDy;
   }

   public void setPartialD2zDx2(double partialD2zDx2)
   {
      this.partialD2zDx2 = partialD2zDx2;
   }

   public void setPartialD2zDy2(double partialD2zDy2)
   {
      this.partialD2zDy2 = partialD2zDy2;
   }

   public void setPartialD2zDxDy(double partialD2zDxDy)
   {
      this.partialD2zDxDy = partialD2zDxDy;
   }

   public void setPartialD3zDx3(double partialD3zDx3)
   {
      this.partialD3zDx3 = partialD3zDx3;
   }

   public void setPartialD3zDy3(double partialD3zDy3)
   {
      this.partialD3zDy3 = partialD3zDy3;
   }

   public void setPartialD3zDx2Dy(double partialD3zDx2Dy)
   {
      this.partialD3zDx2Dy = partialD3zDx2Dy;
   }

   public void setPartialD3zDxDy2(double partialD3zDxDy2)
   {
      this.partialD3zDxDy2 = partialD3zDxDy2;
   }
}
