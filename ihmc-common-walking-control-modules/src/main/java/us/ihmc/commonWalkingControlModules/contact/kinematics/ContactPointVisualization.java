package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ContactPointVisualization
{
   private static int visualizerIndex = 0;

   private final YoFramePoint3D contactPointPosition;
   private final YoFrameVector3D contactPointNormal;

   public ContactPointVisualization(YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      int index = visualizerIndex++;
      contactPointPosition = new YoFramePoint3D("contactPoint" + index, ReferenceFrame.getWorldFrame(), registry);
      contactPointNormal = new YoFrameVector3D("contactNormal" + index, ReferenceFrame.getWorldFrame(), registry);

      YoGraphicPosition contactPointGraphic = new YoGraphicPosition("contactPointGraphic" + index, contactPointPosition, 0.01, YoAppearance.Red());
      YoGraphicVector contactNormalGraphic = new YoGraphicVector("contactPointVector" + index, contactPointPosition, contactPointNormal, 0.1, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic("Contact Points", contactPointGraphic);
      graphicsListRegistry.registerYoGraphic("Contact Points", contactNormalGraphic);
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

   public void hide()
   {
      contactPointPosition.setToNaN();
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