package us.ihmc.avatar.testTools.scs2;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicDefinitionFactory
{
   public static YoGraphicArrow3DDefinition newYoGraphicArrow3DDefinition(String name,
                                                                          YoFrameTuple3D origin,
                                                                          YoFrameTuple3D direction,
                                                                          double scale,
                                                                          ColorDefinition color)
   {
      boolean scaleLength = true;
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      boolean scaleRadius = true;
      double bodyRadius = scale * 0.015;
      double headRadius = bodyRadius * 2.5;
      return newYoGraphicArrow3DDefinition(name, origin, direction, scaleLength, bodyLength, headLength, scaleRadius, bodyRadius, headRadius, color);
   }

   public static YoGraphicArrow3DDefinition newYoGraphicArrow3DDefinition(String name,
                                                                          YoFrameTuple3D origin,
                                                                          YoFrameTuple3D direction,
                                                                          boolean scaleLength,
                                                                          double bodyLength,
                                                                          double headLength,
                                                                          boolean scaleRadius,
                                                                          double bodyRadius,
                                                                          double headRadius,
                                                                          ColorDefinition color)
   {
      YoGraphicArrow3DDefinition definition = new YoGraphicArrow3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setOrigin(newYoTuple3DDefinition(origin));
      definition.setDirection(newYoTuple3DDefinition(direction));
      definition.setScaleLength(scaleLength);
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setScaleRadius(scaleRadius);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   public static YoGraphicPoint3DDefinition newYoGraphicPoint3DDefinition(String name, YoFrameTuple3D position, double size, ColorDefinition color)
   {
      YoGraphicPoint3DDefinition definition = new YoGraphicPoint3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(position));
      definition.setSize(size);
      definition.setColor(color);
      return definition;
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoTuple2D tuple2D)
   {
      return newYoTuple2DDefinition(tuple2D == null ? null : tuple2D.getYoX(), tuple2D == null ? null : tuple2D.getYoY(), null);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoFrameTuple2D frameTuple2D)
   {
      return newYoTuple2DDefinition(frameTuple2D == null ? null : frameTuple2D.getYoX(),
                                    frameTuple2D == null ? null : frameTuple2D.getYoY(),
                                    frameTuple2D == null ? null : frameTuple2D.getReferenceFrame());
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoDouble yoX, YoDouble yoY, ReferenceFrame frame)
   {
      YoTuple2DDefinition definition = new YoTuple2DDefinition();
      definition.setX(toPropertyName(yoX));
      definition.setY(toPropertyName(yoY));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoTuple3D tuple3D)
   {
      return newYoTuple3DDefinition(tuple3D == null ? null : tuple3D.getYoX(),
                                    tuple3D == null ? null : tuple3D.getYoY(),
                                    tuple3D == null ? null : tuple3D.getYoZ(),
                                    null);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoFrameTuple3D frameTuple3D)
   {
      return newYoTuple3DDefinition(frameTuple3D == null ? null : frameTuple3D.getYoX(),
                                    frameTuple3D == null ? null : frameTuple3D.getYoY(),
                                    frameTuple3D == null ? null : frameTuple3D.getYoZ(),
                                    frameTuple3D == null ? null : frameTuple3D.getReferenceFrame());
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoDouble yoX, YoDouble yoY, YoDouble yoZ, ReferenceFrame frame)
   {
      YoTuple3DDefinition definition = new YoTuple3DDefinition();
      definition.setX(toPropertyName(yoX));
      definition.setY(toPropertyName(yoY));
      definition.setZ(toPropertyName(yoZ));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   public static String toPropertyName(ReferenceFrame referenceFrame)
   {
      return referenceFrame == null ? null : referenceFrame.getNameId();
   }

   public static String toPropertyName(YoVariable yoVariable)
   {
      return yoVariable == null ? null : yoVariable.getFullNameString();
   }
}
