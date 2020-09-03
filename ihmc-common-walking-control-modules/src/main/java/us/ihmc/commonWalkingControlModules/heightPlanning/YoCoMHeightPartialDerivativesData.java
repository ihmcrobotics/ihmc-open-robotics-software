package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoCoMHeightPartialDerivativesData implements CoMHeightPartialDerivativesDataBasics
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private ReferenceFrame frameOfCoMHeight;
   private final YoDouble comHeight = new YoDouble("partialZ", registry);
   private final YoDouble partialDzDx = new YoDouble("partialDzDx", registry);
   private final YoDouble partialDzDy = new YoDouble("partialDzDy", registry);
   private final YoDouble partialD2zDx2 = new YoDouble("partialD2zDx2", registry);
   private final YoDouble partialD2zDy2 = new YoDouble("partialD2zDy2", registry);
   private final YoDouble partialD2zDxDy = new YoDouble("partialD2zDxDy", registry);
   private final YoDouble partialD3zDx3 = new YoDouble("partialD3zDx3", registry);
   private final YoDouble partialD3zDy3 = new YoDouble("partialD3zDy3", registry);
   private final YoDouble partialD3zDx2Dy = new YoDouble("partialD3zDx2Dy", registry);
   private final YoDouble partialD3zDxDy2 = new YoDouble("partialD3zDxDy2", registry);

   public YoCoMHeightPartialDerivativesData(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public ReferenceFrame getFrameOfCoMHeight()
   {
      return frameOfCoMHeight;
   }

   public double getComHeight()
   {
      return comHeight.getDoubleValue();
   }

   public double getPartialDzDx()
   {
      return partialDzDx.getDoubleValue();
   }

   public double getPartialDzDy()
   {
      return partialDzDy.getDoubleValue();
   }

   public double getPartialD2zDx2()
   {
      return partialD2zDx2.getDoubleValue();
   }

   public double getPartialD2zDy2()
   {
      return partialD2zDy2.getDoubleValue();
   }

   public double getPartialD2zDxDy()
   {
      return partialD2zDxDy.getDoubleValue();
   }

   public double getPartialD3zDx3()
   {
      return partialD3zDx3.getDoubleValue();
   }

   public double getPartialD3zDy3()
   {
      return partialD3zDy3.getDoubleValue();
   }

   public double getPartialD3zDx2Dy()
   {
      return partialD3zDx2Dy.getDoubleValue();
   }

   public double getPartialD3zDxDy2()
   {
      return partialD3zDxDy2.getDoubleValue();
   }

   public void setCoMHeight(ReferenceFrame referenceFrame, double comHeight)
   {
      this.frameOfCoMHeight = referenceFrame;
      this.comHeight.set(comHeight);
   }

   public void setPartialDzDx(double partialDzDx)
   {
      this.partialDzDx.set(partialDzDx);
   }

   public void setPartialDzDy(double partialDzDy)
   {
      this.partialDzDy.set(partialDzDy);
   }

   public void setPartialD2zDx2(double partialD2zDx2)
   {
      this.partialD2zDx2.set(partialD2zDx2);
   }

   public void setPartialD2zDy2(double partialD2zDy2)
   {
      this.partialD2zDy2.set(partialD2zDy2);
   }

   public void setPartialD2zDxDy(double partialD2zDxDy)
   {
      this.partialD2zDxDy.set(partialD2zDxDy);
   }

   public void setPartialD3zDx3(double partialD3zDx3)
   {
      this.partialD3zDx3.set(partialD3zDx3);
   }

   public void setPartialD3zDy3(double partialD3zDy3)
   {
      this.partialD3zDy3.set(partialD3zDy3);
   }

   public void setPartialD3zDx2Dy(double partialD3zDx2Dy)
   {
      this.partialD3zDx2Dy.set(partialD3zDx2Dy);
   }

   public void setPartialD3zDxDy2(double partialD3zDxDy2)
   {
      this.partialD3zDxDy2.set(partialD3zDxDy2);
   }
}
