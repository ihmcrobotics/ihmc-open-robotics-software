package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoMHeightTrajectoryWaypoint
{
   private final FramePoint3D waypoint = new FramePoint3D();
   private final FramePoint3D minWaypoint = new FramePoint3D();
   private final FramePoint3D maxWaypoint = new FramePoint3D();

   private final YoFramePoint3D yoWaypoint;
   private final YoFramePoint3D yoMinWaypoint;
   private final YoFramePoint3D yoMaxWaypoint;

   public CoMHeightTrajectoryWaypoint(String name, YoRegistry registry)
   {
      yoWaypoint = new YoFramePoint3D(name + "InWorld", ReferenceFrame.getWorldFrame(), registry);
      yoMinWaypoint = new YoFramePoint3D(name + "MinInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoMaxWaypoint = new YoFramePoint3D(name + "MaxInWorld", ReferenceFrame.getWorldFrame(), registry);
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      waypoint.setToZero(referenceFrame);
      minWaypoint.setToZero(referenceFrame);
      maxWaypoint.setToZero(referenceFrame);
   }

   public void setX(double x)
   {
      waypoint.setX(x);
      minWaypoint.setX(x);
      maxWaypoint.setX(x);
   }

   public void setY(double y)
   {
      waypoint.setY(y);
      minWaypoint.setY(y);
      maxWaypoint.setY(y);
   }

   public void setXY(double x, double y)
   {
      setX(x);
      setY(y);
   }

   public void setHeight(double height)
   {
      waypoint.setZ(height);
   }

   public void addHeightOffset(double offset)
   {
      double desiredHeight = MathTools.clamp(waypoint.getZ() + offset, minWaypoint.getZ(), maxWaypoint.getZ());
      waypoint.setZ(desiredHeight);
   }

   public void setMinMax(double zMin, double zMax)
   {
      minWaypoint.setZ(zMin);
      maxWaypoint.setZ(zMax);
   }

   public double getX()
   {
      return waypoint.getX();
   }

   public double getHeight()
   {
      return waypoint.getZ();
   }

   public double getMinHeight()
   {
      return minWaypoint.getZ();
   }

   public double getMaxHeight()
   {
      return maxWaypoint.getZ();
   }

   public FramePoint3DReadOnly getWaypoint()
   {
      return waypoint;
   }

   public FramePoint3DReadOnly getMinWaypoint()
   {
      return minWaypoint;
   }

   public FramePoint3DReadOnly getMaxWaypoint()
   {
      return maxWaypoint;
   }

   public void update()
   {
      yoWaypoint.setMatchingFrame(waypoint);
      yoMinWaypoint.setMatchingFrame(minWaypoint);
      yoMaxWaypoint.setMatchingFrame(maxWaypoint);
   }

   public void setupViz(String graphicListName, String name, AppearanceDefinition color, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double pointSize = 0.03;

      YoGraphicPosition pointD0Viz = new YoGraphicPosition(name, yoWaypoint, pointSize, color);
      YoGraphicPosition pointD0MinViz = new YoGraphicPosition(name + "Min", yoMinWaypoint, 0.8 * pointSize, color);
      YoGraphicPosition pointD0MaxViz = new YoGraphicPosition(name + "Max", yoMaxWaypoint, 0.9 * pointSize, color);

      yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0Viz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MinViz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MaxViz);
   }

   public YoGraphicDefinition createSCS2YoGraphics(String name, ColorDefinition color)
   {
      double pointSize = 0.03;

      return new YoGraphicListDefinition(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name, yoWaypoint, pointSize, color),
                                         YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "Min", yoMinWaypoint, 0.8 * pointSize, color),
                                         YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + "Max", yoMaxWaypoint, 0.9 * pointSize, color));
   }
}
