package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DetectedContactPoint
{
   private static final AppearanceDefinition color = YoAppearance.Blue();
   private static final double radius = 0.01;
   private static final double arrowScale = 0.3;

   private final YoFramePoint3D contactPointPosition;
   private final YoFrameVector3D contactPointNormal;

   public DetectedContactPoint(String nameSuffix, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      contactPointPosition = new YoFramePoint3D("contactPosition_" + nameSuffix, ReferenceFrame.getWorldFrame(), registry);
      contactPointNormal = new YoFrameVector3D("contactNormal_" + nameSuffix, ReferenceFrame.getWorldFrame(), registry);

      if (graphicsListRegistry != null)
      {
         YoGraphicPosition contactPointGraphic = new YoGraphicPosition("contactPositionGraphic_" + nameSuffix, contactPointPosition, radius, color);
         YoGraphicVector contactNormalGraphic = new YoGraphicVector("contactNormalGraphic" + nameSuffix, contactPointPosition, contactPointNormal, arrowScale, color);
         graphicsListRegistry.registerYoGraphic("Kinematic-Detected Contact Points", contactPointGraphic);
         graphicsListRegistry.registerYoGraphic("Kinematic-Detected Contact Points", contactNormalGraphic);
      }
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