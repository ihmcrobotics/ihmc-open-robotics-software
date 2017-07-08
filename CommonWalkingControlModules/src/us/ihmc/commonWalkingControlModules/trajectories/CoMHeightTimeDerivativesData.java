package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CoMHeightTimeDerivativesData
{
   private ReferenceFrame frameOfCenterOfMassHeight;
   private double comHeight, comHeightVelocity, comHeightAcceleration;

   public void getComHeight(FramePoint3D framePointToPack)
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

   public void set(CoMHeightTimeDerivativesData heightZData)
   {
      this.frameOfCenterOfMassHeight = heightZData.frameOfCenterOfMassHeight;
      this.comHeight = heightZData.comHeight;
      this.comHeightVelocity = heightZData.comHeightVelocity;
      this.comHeightAcceleration = heightZData.comHeightAcceleration;
   }

}
