package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CenterOfMassHeightOutputData
{
   private double desiredCenterOfMassHeight;
   private final FrameVector2d desiredCenterOfMassHeightSlope = new FrameVector2d(ReferenceFrame.getWorldFrame());
   private final FrameVector2d desiredCenterOfMassHeightSecondDerivative = new FrameVector2d(ReferenceFrame.getWorldFrame());
   private double ddzdxdy;
         
   public double getDesiredCenterOfMassHeight()
   {
      return desiredCenterOfMassHeight;
   }

   public void getDesiredCenterOfMassHeightSlope(FrameVector2d slopeToPack)
   {
      slopeToPack.setIncludingFrame(desiredCenterOfMassHeightSlope);
   }

   public void getDesiredCenterOfMassHeightSecondDerivative(FrameVector2d secondDerivativeToPack)
   {
      secondDerivativeToPack.setIncludingFrame(desiredCenterOfMassHeightSecondDerivative);
   }
   
   public void setDesiredCenterOfMassHeight(double desiredCenterOfMassHeight)
   {
      this.desiredCenterOfMassHeight = desiredCenterOfMassHeight;
   }

   public void setDesiredCenterOfMassHeightSlope(FrameVector2d desiredCenterOfMassHeightSlope)
   {
      this.desiredCenterOfMassHeightSlope.set(desiredCenterOfMassHeightSlope);
   }

   public void setDesiredCenterOfMassHeightSecondDerivative(FrameVector2d desiredCenterOfMassHeightSecondDerivative)
   {
      this.desiredCenterOfMassHeightSecondDerivative.set(desiredCenterOfMassHeightSecondDerivative);
   }
   
   public void get(CenterOfMassHeightOutputData outputDataToPack)
   {
      outputDataToPack.desiredCenterOfMassHeight = this.desiredCenterOfMassHeight;
      outputDataToPack.desiredCenterOfMassHeightSlope.set(this.desiredCenterOfMassHeightSlope);
      outputDataToPack.desiredCenterOfMassHeightSecondDerivative.set(this.desiredCenterOfMassHeightSecondDerivative);
   }
   
   public void set(CenterOfMassHeightOutputData outputData)
   {
      this.desiredCenterOfMassHeight = outputData.desiredCenterOfMassHeight;
      this.desiredCenterOfMassHeightSlope.set(outputData.desiredCenterOfMassHeightSlope);
      this.desiredCenterOfMassHeightSecondDerivative.set(outputData.desiredCenterOfMassHeightSecondDerivative);
   }

   public double getDdzdxdy()
   {
      return ddzdxdy;
   }

   public void setDdzdxdy(double ddzdxdy)
   {
      this.ddzdxdy = ddzdxdy;
   }
}
