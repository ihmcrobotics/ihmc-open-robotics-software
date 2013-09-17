package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint2d;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.GlitchFilteredBooleanYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class YoContactPoint extends ContactPoint
{
   private final GlitchFilteredBooleanYoVariable isPositionFixValid;
   private final YoFramePoint2d yoPosition2d;
   private int windowSize;

   public YoContactPoint(String namePrefix, int index, FramePoint2d point2d, YoVariableRegistry parentRegistry)
   {
      // If delayTimeBeforeTrustingContacts not provided, initialize windowSize to -1 so it has to be initialized before being used 
      this(namePrefix, index, point2d, -1, parentRegistry);
   }

   public YoContactPoint(String namePrefix, int index, FramePoint2d point2d, int windowSize, YoVariableRegistry parentRegistry)
   {
      super(point2d);

      yoPosition2d = new YoFramePoint2d(namePrefix + "Contact" + index, point2d.getReferenceFrame(), parentRegistry);

      this.windowSize = windowSize;
      isPositionFixValid = new GlitchFilteredBooleanYoVariable(namePrefix + "positionFixValid" + index, parentRegistry, windowSize);
   }

   public void setDelayWindowBeforeTrustingContacts(int windowSize)
   {
      this.windowSize = windowSize;
      isPositionFixValid.setWindowSize(windowSize);
   }

   public boolean isTrusted()
   {
      return isPositionFixValid.getBooleanValue();
   }

   public void setTrusted(boolean isTrusted)
   {
      isPositionFixValid.set(isTrusted);
   }

   /**
    * The status is filtered and the point can be trusted only after a certain number of calls of this method, number computed based on the delayTimeBeforeTrustingContacts.
    * @param isTrusted
    */
   public void updateTrustedStatus(boolean isTrusted)
   {
      if (windowSize < 0)
         throw new RuntimeException("delayWindowBeforeTrustingContacts needs to be initialized before calling updateTrustedStatus.");
      
      isPositionFixValid.update(isTrusted);
   }

   public YoFramePoint2d getYoPosition2d()
   {
      return yoPosition2d;
   }
}
