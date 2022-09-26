package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;

/**
 * Command that holds output from the {@link LinearMomentumRateControlModule} going to the walking controller state
 * machine that might be running at a slower rate then the ICP feedback.
 *
 * @author Georg Wiedebach
 */
public class LinearMomentumRateControlModuleOutput
{
   /**
    * The desired feedback CMP. It is used by the walking controller to check some conditions (e.g. it's placement
    * before toe off).
    */
   private final FramePoint2D desiredCMP = new FramePoint2D();

   public void setDesiredCMP(FramePoint2DReadOnly desiredCMP)
   {
      this.desiredCMP.setIncludingFrame(desiredCMP);
   }

   public FramePoint2D getDesiredCMP()
   {
      return desiredCMP;
   }


   public void set(LinearMomentumRateControlModuleOutput other)
   {
      desiredCMP.setIncludingFrame(other.desiredCMP);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof LinearMomentumRateControlModuleOutput)
      {
         LinearMomentumRateControlModuleOutput other = (LinearMomentumRateControlModuleOutput) obj;
         if (!desiredCMP.equals(other.desiredCMP))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
