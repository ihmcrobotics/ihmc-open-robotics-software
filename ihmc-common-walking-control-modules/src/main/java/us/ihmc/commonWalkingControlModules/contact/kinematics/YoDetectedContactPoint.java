package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoDetectedContactPoint
{
   private static final double CONTACT_POINT_GRAPHIC_DIAMETER = 0.01;
   private static final double SURFACE_NORMAL_GRAPHIC_SCALE = 0.2;

   private final String name;
   private final YoFramePoint3D contactPointPosition;
   private final YoFrameVector3D contactPointNormal;

   private final YoFramePoint3D environmentProjectionPoint;
   private final YoDouble robotEnvironmentSignedDistance;

   private FrameShape3DReadOnly robotShape;
   private FrameShape3DBasics environmentShape;

   public YoDetectedContactPoint(String name, YoRegistry registry)
   {
      this.name = name;
      contactPointPosition = new YoFramePoint3D("contactPosition_" + name, ReferenceFrame.getWorldFrame(), registry);
      contactPointNormal = new YoFrameVector3D("contactNormal_" + name, ReferenceFrame.getWorldFrame(), registry);

      environmentProjectionPoint = new YoFramePoint3D("environmentProjectionPosition_" + name, ReferenceFrame.getWorldFrame(), registry);
      robotEnvironmentSignedDistance = new YoDouble("robotEnvironmentSignedDistance_" + name, registry);
   }

   public void setVerticalContactNormal()
   {
      contactPointNormal.set(Axis3D.Z);
   }

   public void clearContact()
   {
      contactPointPosition.setToNaN();
      environmentProjectionPoint.setToNaN();
      contactPointNormal.setToNaN();

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

   public YoFramePoint3D getEnvironmentProjectionPoint()
   {
      return environmentProjectionPoint;
   }

   public void setRobotEnvironmentSignedDistance(double robotEnvironmentSignedDistance)
   {
      this.robotEnvironmentSignedDistance.set(robotEnvironmentSignedDistance);
   }

   public double getRobotEnvironmentSignedDistance()
   {
      return robotEnvironmentSignedDistance.getDoubleValue();
   }

   public void setRobotShape(FrameShape3DReadOnly robotShape)
   {
      this.robotShape = robotShape;
   }

   public void setEnvironmentShape(FrameShape3DBasics environmentShape)
   {
      this.environmentShape = environmentShape;
   }

   public FrameShape3DReadOnly getRobotShape()
   {
      return robotShape;
   }

   public FrameShape3DReadOnly getEnvironmentShape()
   {
      return environmentShape;
   }

   public YoGraphicDefinition getSCS2Graphic()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(name + "Normal", contactPointPosition, contactPointNormal, SURFACE_NORMAL_GRAPHIC_SCALE, ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(name + "InverseNormal", contactPointPosition, contactPointNormal, -SURFACE_NORMAL_GRAPHIC_SCALE, ColorDefinitions.LightSeaGreen()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "Position", contactPointPosition, CONTACT_POINT_GRAPHIC_DIAMETER, ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "EnvironmentPosition", environmentProjectionPoint, CONTACT_POINT_GRAPHIC_DIAMETER, ColorDefinitions.Yellow()));
      return group;
   }
}