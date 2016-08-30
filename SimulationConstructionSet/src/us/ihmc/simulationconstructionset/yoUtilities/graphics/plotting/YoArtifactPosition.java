package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;

public class YoArtifactPosition extends YoArtifact
{
   private static final int LEGEND_RADIUS = 20;
   private static final BasicStroke STROKE = new BasicStroke(1.2f);
   
   private final YoFramePoint2d point;
   private final Vector2d radii = new Vector2d();
   private final GraphicType graphicType;
   
   private final Point2d tempPoint = new Point2d();

   public YoArtifactPosition(String namePrefix, String nameSuffix, GraphicType type, Color color, double radius, YoVariableRegistry registry)
   {
      this(namePrefix+nameSuffix, new DoubleYoVariable(namePrefix + "X" + nameSuffix, registry), new DoubleYoVariable(namePrefix + "Y" + nameSuffix, registry), type, color, radius);
   }
   
   public YoArtifactPosition(String name, DoubleYoVariable x, DoubleYoVariable y, GraphicType type, Color color, double radius)
   {
      this(name, new YoFramePoint2d(x, y, ReferenceFrame.getWorldFrame()), type, color, radius);
   }
   
   public YoArtifactPosition(String name, YoFramePoint2d point, GraphicType type, Color color, double radius)
   {
      super(name, new double[] {radius, type.ordinal()}, color, point.getYoX(), point.getYoY());
      
      this.point = point;
      this.graphicType = type;
      this.radii.set(radius, radius);
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int centerX, int centerY)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);
      
      switch (graphicType)
      {
         case BALL :
            graphics.drawEmptyCircle(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case SOLID_BALL :
            graphics.drawFilledCircle(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case CROSS :
            graphics.drawCross(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case BALL_WITH_CROSS :
            graphics.drawCircleWithCross(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case ROTATED_CROSS :
            graphics.drawRotatedCross(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case BALL_WITH_ROTATED_CROSS :
            graphics.drawCircleWithRotatedCross(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case DIAMOND :
            graphics.drawDiamond(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case DIAMOND_WITH_CROSS :
            graphics.drawDiamondWithCross(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case SQUARE :
            graphics.drawSquare(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case SQUARE_WITH_CROSS :
            graphics.drawSquareWithCross(centerX, centerY, LEGEND_RADIUS, color);
            break;
         case ELLIPSOID :
            double radius = radii.getX();
            radii.setX(radii.getX() * 1.2);
            radii.setY(radii.getY() * 0.8);
            graphics.drawEmptyCircle(centerX, centerY, LEGEND_RADIUS, color);
            radii.set(radius, radius);
            break;
      }
   }

   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      point.get(tempPoint);
      drawLocal(graphics);
   }

   @Override
   public void drawHistoryEntry(Graphics2DAdapter graphics, double[] entry)
   {
      tempPoint.set(entry[0], entry[1]);
      drawLocal(graphics);
   }

   private void drawLocal(Graphics2DAdapter graphics)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);
      
      switch (graphicType)
      {
         case BALL :
            graphics.drawEmptyCircle(tempPoint, radii);
            break;
         case SOLID_BALL :
            graphics.drawFilledCircle(tempPoint, radii);
            break;
         case CROSS :
            graphics.drawCross(tempPoint, radii);
            break;
         case BALL_WITH_CROSS :
            graphics.drawCircleWithCross(tempPoint, radii);
            break;
         case ROTATED_CROSS :
            graphics.drawRotatedCross(tempPoint, radii);
            break;
         case BALL_WITH_ROTATED_CROSS :
            graphics.drawCircleWithRotatedCross(tempPoint, radii);
            break;
         case DIAMOND :
            graphics.drawDiamond(tempPoint, radii);
            break;
         case DIAMOND_WITH_CROSS :
            graphics.drawDiamondWithCross(tempPoint, radii);
            break;
         case SQUARE :
            graphics.drawSquare(tempPoint, radii);
            break;
         case SQUARE_WITH_CROSS :
            graphics.drawSquareWithCross(tempPoint, radii);
            break;
         case ELLIPSOID :
            double radius = radii.getX();
            radii.setX(radii.getX() * 1.2);
            radii.setY(radii.getY() * 0.8);
            graphics.drawEmptyCircle(tempPoint, radii);
            radii.set(radius, radius);
            break;
      }
   }
   
   @Override
   public RemoteGraphicType getRemoteGraphicType() 
   {
      return RemoteGraphicType.POSITION_ARTIFACT;
   }
   
   public DoubleYoVariable getYoX()
   {
      return point.getYoX();
   }
   
   public DoubleYoVariable getYoY()
   {
      return point.getYoY();
   }
   
   public void setPosition(Point2d point2d)
   {
      point.set(point2d);
   }
}
