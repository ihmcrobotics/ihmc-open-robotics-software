package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoDetectedContactPoint
{
   private final String name;
   private final YoFramePoint3D contactPointPosition;
   private final YoFrameVector3D contactPointNormal;

   public YoDetectedContactPoint(String name, YoRegistry registry)
   {
      this.name = name;
      contactPointPosition = new YoFramePoint3D("contactPosition_" + name, ReferenceFrame.getWorldFrame(), registry);
      contactPointNormal = new YoFrameVector3D("contactNormal_" + name, ReferenceFrame.getWorldFrame(), registry);
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

   public YoGraphicDefinition getSCS2Graphic()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(name + "Normal", contactPointPosition, contactPointNormal, 0.2, ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(name + "InverseNormal", contactPointPosition, contactPointNormal, -0.2, ColorDefinitions.LightSeaGreen()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "Position", contactPointPosition, 0.03, ColorDefinitions.Red()));
      return group;
   }
}