package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class AlwaysInContactFootSwitch implements FootSwitchInterface
{
   
   private final ReferenceFrame measurementFrame;
   private final double weightOnFoot;
   
   public AlwaysInContactFootSwitch(double weightOnFoot, ReferenceFrame measurementFrame)
   {
      this.measurementFrame = measurementFrame;
      this.weightOnFoot = weightOnFoot;
   }

   @Override
   public boolean hasFootHitGround()
   {
      return true;
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      copToPack.setToNaN(measurementFrame);
   }

   @Override
   public void updateCoP()
   {
      
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setToZero();
      if (hasFootHitGround())
         footWrenchToPack.setLinearPartZ(weightOnFoot / 4.0);

   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
   }

   @Override
   public void reset()
   {
      
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return false;
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      
   }

   @Override
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {
      
   }

   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {

   }

}
