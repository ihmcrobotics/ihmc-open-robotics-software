package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;


public class YoContactPoint extends ContactPoint
{
   private final YoVariableRegistry registry;
   private final YoFramePoint yoPosition;
   // TODO Needs to go away
   private final YoFramePoint2d yoPosition2d;
   private final BooleanYoVariable isInContact;
   private final String namePrefix;

   public YoContactPoint(String namePrefix, int index, FramePoint2d point2d, PlaneContactState parentContactState, YoVariableRegistry parentRegistry)
   {
      super(point2d, parentContactState);
      
      this.namePrefix = namePrefix;
      
      //TODO: Check if it is better to create an actual child registry
      registry = parentRegistry;

      yoPosition = new YoFramePoint(namePrefix + "Contact" + index, point2d.getReferenceFrame(), registry);
      // TODO Needs to go away
      yoPosition2d = new YoFramePoint2d(namePrefix + "Contact2d" + index, point2d.getReferenceFrame(), registry);
      isInContact = new BooleanYoVariable(namePrefix + "InContact" + index, registry);
   }

   public boolean isInContact()
   {
      return isInContact.getBooleanValue();
   }

   public void setInContact(boolean inContact)
   {
      super.setInContact(inContact);
      isInContact.set(inContact);
   }
   
   // TODO Needs to go away
   public FramePoint2d getPosition2d()
   {
      FramePoint2d position2d = super.getPosition2d();
      yoPosition2d.set(position2d);
      return position2d;
   }

   public FramePoint getPosition()
   {
      FramePoint position = super.getPosition();
      yoPosition.set(position);
      return position;
   }
   
   public String toString()
   {
      return namePrefix + ", in contact: " + isInContact() + ", position: " + getPosition2d().toString();
   }
}
