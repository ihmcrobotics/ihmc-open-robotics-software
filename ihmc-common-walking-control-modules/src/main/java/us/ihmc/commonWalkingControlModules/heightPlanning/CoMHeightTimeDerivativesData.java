package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CoMHeightTimeDerivativesData
{
   private ReferenceFrame frameOfCenterOfMassHeight;
   private final YoDouble comHeight, comHeightVelocity, comHeightAcceleration, comHeightJerk;

   public CoMHeightTimeDerivativesData(String namePrefix, YoVariableRegistry registry)
   {
      comHeight = new YoDouble(namePrefix + "CoMHeight", registry);
      comHeightVelocity = new YoDouble(namePrefix + "comHeightVelocity", registry);
      comHeightAcceleration = new YoDouble(namePrefix + "comHeightAcceleration", registry);
      comHeightJerk = new YoDouble(namePrefix + "comHeightJerk", registry);
   }

   public void getComHeight(FramePoint3D framePointToPack)
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

   public void set(CoMHeightTimeDerivativesData heightZData)
   {
      setComHeight(heightZData.frameOfCenterOfMassHeight, heightZData.comHeight.getDoubleValue());
      setComHeightVelocity(heightZData.getComHeightVelocity());
      setComHeightAcceleration(heightZData.getComHeightAcceleration());
      setComHeightJerk(heightZData.getComHeightJerk());
   }

}
