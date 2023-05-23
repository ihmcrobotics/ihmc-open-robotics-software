package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoDetectedContactPoint
{
   private final YoFramePoint3D contactPointPosition;
   private final YoFrameVector3D contactPointNormal;

   public YoDetectedContactPoint(String nameSuffix, YoRegistry registry)
   {
      contactPointPosition = new YoFramePoint3D("contactPosition_" + nameSuffix, ReferenceFrame.getWorldFrame(), registry);
      contactPointNormal = new YoFrameVector3D("contactNormal_" + nameSuffix, ReferenceFrame.getWorldFrame(), registry);
   }

   public void setVerticalContactNormal()
   {
      contactPointNormal.set(Axis3D.Z);
   }

   public void update(Tuple3DReadOnly position, Tuple3DReadOnly normal)
   {
      contactPointPosition.set(position);
      contactPointNormal.set(normal);
   }

   public void clearContact()
   {
      contactPointPosition.setToNaN();
   }

   public boolean isInContact()
   {
      return !contactPointPosition.containsNaN();
   }

   public YoFramePoint3D getContactPointPosition()
   {
      return contactPointPosition;
   }

   public YoFrameVector3D getContactPointNormal()
   {
      return contactPointNormal;
   }
}