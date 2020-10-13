package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class YoContactPoint implements ContactPointBasics
{
   private final YoRegistry registry;
   private final YoFramePoint3D yoPosition;
   private final YoBoolean isInContact;
   private final String namePrefix;
   private final PlaneContactState parentContactState;

   public YoContactPoint(String namePrefix, int index, FramePoint2D contactPointPosition2d, PlaneContactState parentContactState,
                         YoRegistry parentRegistry)
   {
      this(namePrefix, index, contactPointPosition2d.getReferenceFrame(), parentContactState, parentRegistry);
      set(contactPointPosition2d);
   }

   public YoContactPoint(String namePrefix, int index, FramePoint3D contactPointPosition, PlaneContactState parentContactState,
                         YoRegistry parentRegistry)
   {
      this(namePrefix, index, contactPointPosition.getReferenceFrame(), parentContactState, parentRegistry);
      set(contactPointPosition);
   }

   public YoContactPoint(String namePrefix, int index, ReferenceFrame pointFrame, PlaneContactState parentContactState, YoRegistry parentRegistry)
   {
      this.parentContactState = parentContactState;
      this.namePrefix = namePrefix;

      //TODO: Check if it is better to create an actual child registry
      registry = parentRegistry;

      yoPosition = new YoFramePoint3D(namePrefix + "Contact" + index, pointFrame, registry);
      isInContact = new YoBoolean(namePrefix + "InContact" + index, registry);
   }

   public String getNamePrefix()
   {
      return yoPosition.getNamePrefix();
   }

   public String getNameSuffix()
   {
      return yoPosition.getNameSuffix();
   }

   @Override
   public boolean isInContact()
   {
      return isInContact.getBooleanValue();
   }

   @Override
   public void setInContact(boolean inContact)
   {
      isInContact.set(inContact);
   }

   @Override
   public void setX(double x)
   {
      yoPosition.setX(x);
   }

   @Override
   public void setY(double y)
   {
      yoPosition.setY(y);
   }

   @Override
   public void setZ(double z)
   {
      yoPosition.setZ(z);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return yoPosition.getReferenceFrame();
   }

   @Override
   public double getX()
   {
      return yoPosition.getX();
   }

   @Override
   public double getY()
   {
      return yoPosition.getY();
   }

   @Override
   public double getZ()
   {
      return yoPosition.getZ();
   }

   @Override
   public PlaneContactState getParentContactState()
   {
      return parentContactState;
   }

   public boolean epsilonEquals(FramePoint2D contactPointPosition2d, double threshold)
   {
      yoPosition.checkReferenceFrameMatch(contactPointPosition2d);
      if (!MathTools.epsilonEquals(yoPosition.getX(), contactPointPosition2d.getX(), threshold))
         return false;
      if (!MathTools.epsilonEquals(yoPosition.getY(), contactPointPosition2d.getY(), threshold))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return namePrefix + ", in contact: " + isInContact() + ", position: " + yoPosition.toString();
   }
}
