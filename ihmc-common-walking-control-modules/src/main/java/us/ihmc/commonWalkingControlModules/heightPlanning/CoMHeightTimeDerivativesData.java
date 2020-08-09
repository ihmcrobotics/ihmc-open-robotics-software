package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;

public class CoMHeightTimeDerivativesData implements CoMHeightTimeDerivativesDataBasics
{
   private ReferenceFrame frameOfCenterOfMassHeight;
   private double comHeight, comHeightVelocity, comHeightAcceleration, comHeightJerk;

   public ReferenceFrame getReferenceFrame()
   {
      return frameOfCenterOfMassHeight;
   }

   public double getComHeightInFrame()
   {
      return comHeight;
   }

   public void getComHeight(FramePoint3DBasics framePointToPack)
   {
      framePointToPack.setIncludingFrame(frameOfCenterOfMassHeight, 0.0, 0.0, comHeight);
   }

   public void setComHeight(ReferenceFrame referenceFrame, double comHeight)
   {
      this.frameOfCenterOfMassHeight = referenceFrame;
      this.comHeight = comHeight;
   }

   public double getComHeightVelocity()
   {
      return comHeightVelocity;
   }

   public void setComHeightVelocity(double comHeightVelocity)
   {
      this.comHeightVelocity = comHeightVelocity;
   }

   public double getComHeightAcceleration()
   {
      return comHeightAcceleration;
   }

   public void setComHeightAcceleration(double comHeightAcceleration)
   {
      this.comHeightAcceleration = comHeightAcceleration;
   }

   public double getComHeightJerk()
   {
      return comHeightJerk;
   }

   public void setComHeightJerk(double comHeightJerk)
   {
      this.comHeightJerk = comHeightJerk;
   }
}
