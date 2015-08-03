package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.ReferenceFrame;

public class CoMHeightPartialDerivativesData
{
   private ReferenceFrame frameOfCoMHeight;
   private double comHeight, partialDzDx, partialDzDy, partialD2zDx2, partialD2zDy2, partialD2zDxDy;

   public void set(CoMHeightPartialDerivativesData centerOfMassHeightPartialDerivativesData)
   {
      this.comHeight = centerOfMassHeightPartialDerivativesData.comHeight; 
      this.partialDzDx = centerOfMassHeightPartialDerivativesData.partialDzDx; 
      this.partialDzDy = centerOfMassHeightPartialDerivativesData.partialDzDy;
      this.partialD2zDx2 = centerOfMassHeightPartialDerivativesData.partialD2zDx2; 
      this.partialD2zDy2 = centerOfMassHeightPartialDerivativesData.partialD2zDy2; 
      this.partialD2zDxDy = centerOfMassHeightPartialDerivativesData.partialD2zDxDy;
   }
   
   public void getCoMHeight(FramePoint framePointToPack)
   {
      framePointToPack.setIncludingFrame(frameOfCoMHeight, 0.0, 0.0, comHeight);
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

   public boolean isFlat(double epsilon)
   {
      return MathTools.epsilonEquals(partialD2zDx2, 0.0, epsilon) && MathTools.epsilonEquals(partialD2zDxDy, 0.0, epsilon) && 
            MathTools.epsilonEquals(partialD2zDy2, 0.0, epsilon) && MathTools.epsilonEquals(partialDzDx, 0.0, epsilon) && 
            MathTools.epsilonEquals(partialDzDy, 0.0, epsilon);
   }
}
