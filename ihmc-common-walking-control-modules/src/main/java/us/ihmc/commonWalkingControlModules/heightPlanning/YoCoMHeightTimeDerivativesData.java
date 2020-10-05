package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoCoMHeightTimeDerivativesData implements CoMHeightTimeDerivativesDataBasics
{
   private ReferenceFrame frameOfCenterOfMassHeight;
   private final YoDouble comHeight, comHeightVelocity, comHeightAcceleration, comHeightJerk;

   public YoCoMHeightTimeDerivativesData(String namePrefix, YoRegistry registry)
   {
      comHeight = new YoDouble(namePrefix + "CoMHeight", registry);
      comHeightVelocity = new YoDouble(namePrefix + "comHeightVelocity", registry);
      comHeightAcceleration = new YoDouble(namePrefix + "comHeightAcceleration", registry);
      comHeightJerk = new YoDouble(namePrefix + "comHeightJerk", registry);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return frameOfCenterOfMassHeight;
   }

   public double getComHeightInFrame()
   {
      return comHeight.getDoubleValue();
   }

   public void getComHeight(FramePoint3DBasics framePointToPack)
   {
      framePointToPack.setIncludingFrame(frameOfCenterOfMassHeight, 0.0, 0.0, comHeight.getValue());
   }

   public void setComHeight(ReferenceFrame referenceFrame, double comHeight)
   {
      this.frameOfCenterOfMassHeight = referenceFrame;
      this.comHeight.set(comHeight);
   }

   public double getComHeightVelocity()
   {
      return comHeightVelocity.getValue();
   }

   public void setComHeightVelocity(double comHeightVelocity)
   {
      this.comHeightVelocity.set(comHeightVelocity);
   }

   public double getComHeightAcceleration()
   {
      return comHeightAcceleration.getDoubleValue();
   }

   public void setComHeightAcceleration(double comHeightAcceleration)
   {
      this.comHeightAcceleration.set(comHeightAcceleration);
   }

   public double getComHeightJerk()
   {
      return comHeightJerk.getDoubleValue();
   }

   public void setComHeightJerk(double comHeightJerk)
   {
      this.comHeightJerk.set(comHeightJerk);
   }
}
