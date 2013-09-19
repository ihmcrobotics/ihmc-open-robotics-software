package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint2d;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class YoContactPoint extends ContactPoint
{
   private final YoVariableRegistry registry;
   private final YoFramePoint2d yoPosition2d;
   private final BooleanYoVariable isInContact;

   public YoContactPoint(String namePrefix, int index, FramePoint2d point2d, PlaneContactState parentContactState, YoVariableRegistry parentRegistry)
   {
      super(point2d, parentContactState);
      
      //TODO: Check if it is better to create an actual child registry
      registry = parentRegistry;

      yoPosition2d = new YoFramePoint2d(namePrefix + "Contact" + index, point2d.getReferenceFrame(), registry);
      isInContact = new BooleanYoVariable(namePrefix + "InContact" + index, registry);
   }

   public boolean isInContact()
   {
      return isInContact.getBooleanValue();
   }

   public void setInContact(boolean inContact)
   {
      isInContact.set(inContact);
   }

   public YoFramePoint2d getYoPosition2d()
   {
      return yoPosition2d;
   }
}
