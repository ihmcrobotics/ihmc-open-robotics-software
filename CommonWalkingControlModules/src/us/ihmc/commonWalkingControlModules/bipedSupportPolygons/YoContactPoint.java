package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint2d;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.GlitchFilteredBooleanYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class YoContactPoint extends ContactPoint
{
   private final YoVariableRegistry registry;
   private final GlitchFilteredBooleanYoVariable isPositionFixValid;
   private final YoFramePoint2d yoPosition2d;
   private final BooleanYoVariable isInContact;
   private int windowSize;

   public YoContactPoint(String namePrefix, int index, FramePoint2d point2d, YoVariableRegistry parentRegistry)
   {
      // If delayTimeBeforeTrustingContacts not provided, initialize windowSize to Integer.MAX_VALUE so it has to be initialized before being used 
      this(namePrefix, index, point2d, Integer.MAX_VALUE, parentRegistry);
   }

   public YoContactPoint(String namePrefix, int index, FramePoint2d point2d, int windowSize, YoVariableRegistry parentRegistry)
   {
      super(point2d);
      
      //TODO: Check if it is better to create an actual child registry
      registry = parentRegistry;

      yoPosition2d = new YoFramePoint2d(namePrefix + "Contact" + index, point2d.getReferenceFrame(), registry);
      isInContact = new BooleanYoVariable(namePrefix + "InContact" + index, registry);
      
      this.windowSize = windowSize;
      isPositionFixValid = new GlitchFilteredBooleanYoVariable(namePrefix + "positionFixValid" + index, registry, windowSize);
   }

   public void setDelayWindowBeforeTrustingContacts(int windowSize)
   {
      this.windowSize = windowSize;
      isPositionFixValid.setWindowSize(windowSize);
   }

   public boolean isInContact()
   {
      return isInContact.getBooleanValue();
   }

   public void setInContact(boolean inContact)
   {
      isInContact.set(inContact);
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
      if (windowSize == Integer.MAX_VALUE)
         throw new RuntimeException("delayWindowBeforeTrustingContacts needs to be initialized before calling updateTrustedStatus.");
      
      isPositionFixValid.update(isTrusted);
   }

   public YoFramePoint2d getYoPosition2d()
   {
      return yoPosition2d;
   }
}
