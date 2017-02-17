package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoArtifactPosition extends YoArtifact
{
   private static final int LEGEND_RADIUS = 20;
   private static final BasicStroke STROKE = new BasicStroke(1.2f);
   
   private final YoFramePoint2d point;
   private final Vector2D radii = new Vector2D();
   private final GraphicType graphicType;
   
   private final Point2D tempPoint = new Point2D();
   private final Vector2D legendRadii = new Vector2D();

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
   public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);
      
      legendRadii.set(LEGEND_RADIUS, LEGEND_RADIUS);
      
      switch (graphicType)
      {
         case BALL :
            graphics.drawOval(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case SOLID_BALL :
            graphics.drawOvalFilled(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case CROSS :
            graphics.drawCross(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case BALL_WITH_CROSS :
            graphics.drawCircleWithCross(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case ROTATED_CROSS :
            graphics.drawRotatedCross(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case BALL_WITH_ROTATED_CROSS :
            graphics.drawCircleWithRotatedCross(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case DIAMOND :
            graphics.drawDiamond(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case DIAMOND_WITH_CROSS :
            graphics.drawDiamondWithCross(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case SQUARE :
            graphics.drawRectangle(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case SQUARE_WITH_CROSS :
            graphics.drawSquareWithCross(graphics.getScreenFrame(), origin, legendRadii);
            break;
         case ELLIPSOID :
            legendRadii.set(LEGEND_RADIUS * 1.2, LEGEND_RADIUS * 0.8);
            graphics.drawOval(graphics.getScreenFrame(), origin, legendRadii);
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
      
      if (Double.isNaN(tempPoint.getX()) || Double.isNaN(tempPoint.getY()))
      {
         return;
      }
      
      switch (graphicType)
      {
         case BALL :
            graphics.drawOval(tempPoint, radii);
            break;
         case SOLID_BALL :
            graphics.drawOvalFilled(tempPoint, radii);
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
            graphics.drawOval(tempPoint, radii);
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
   
   public void setPosition(Point2D point2d)
   {
      point.set(point2d);
   }
}
