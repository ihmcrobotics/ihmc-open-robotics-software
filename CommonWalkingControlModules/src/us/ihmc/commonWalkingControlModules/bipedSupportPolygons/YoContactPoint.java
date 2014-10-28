package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import javax.vecmath.Point2d;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;


public class YoContactPoint implements ContactPointInterface
{
   private final YoVariableRegistry registry;
   private final YoFramePoint yoPosition;
   // TODO Needs to go away
   private final YoFramePoint2d yoPosition2d;
   private final BooleanYoVariable isInContact;
   private final String namePrefix;
   private final PlaneContactState parentContactState;
   
   public YoContactPoint(String namePrefix, int index, FramePoint2d point2d, PlaneContactState parentContactState, YoVariableRegistry parentRegistry)
   {
      this.parentContactState = parentContactState;
      this.namePrefix = namePrefix;
      
      //TODO: Check if it is better to create an actual child registry
      registry = parentRegistry;

      yoPosition = new YoFramePoint(namePrefix + "Contact" + index, point2d.getReferenceFrame(), registry);
      // TODO Needs to go away
      yoPosition2d = new YoFramePoint2d(namePrefix + "Contact2d" + index, point2d.getReferenceFrame(), registry);
      isInContact = new BooleanYoVariable(namePrefix + "InContact" + index, registry);
      
      yoPosition.set(point2d.getX(), point2d.getY(), 0.0);
      yoPosition2d.set(point2d.getX(), point2d.getY());
   }

   public boolean isInContact()
   {
      return isInContact.getBooleanValue();
   }

   public void setInContact(boolean inContact)
   {
      isInContact.set(inContact);
   }
   
   // TODO Needs to go away
   public FramePoint2d getPosition2d()
   {
      return yoPosition2d.getFramePoint2dCopy();
   }

   public FramePoint getPosition()
   {
      return yoPosition.getFramePointCopy();
   }
   
   public void setPosition(Point2d contactPointLocation)
   {
      yoPosition2d.set(contactPointLocation.getX(), contactPointLocation.getY());
      yoPosition.set(contactPointLocation.getX(), contactPointLocation.getY(), 0.0);    
   }
   
   public void setPosition(FramePoint2d contactPointLocation)
   {
      yoPosition2d.set(contactPointLocation);
      yoPosition.setXY(contactPointLocation);
   }
   
   public PlaneContactState getParentContactState()
   {
      return parentContactState;
   }
   
   public String toString()
   {
      return namePrefix + ", in contact: " + isInContact() + ", position: " + getPosition2d().toString();
   }

}
