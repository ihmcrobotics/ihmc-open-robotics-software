package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Drawing2DTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.RemoteYoGraphic;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;


public class YoArtifactPosition extends Artifact implements RemoteYoGraphic
{
   private static final long serialVersionUID = -1824071784539220859L;
   
   private final ArrayList<double[]> historicalPositions = new ArrayList<double[]>();

   private final DoubleYoVariable x;
   private final DoubleYoVariable y;
   private final GraphicType graphicType;
   private final double scale;

   public YoArtifactPosition(String namePrefix, String nameSuffix, YoGraphicPosition.GraphicType type, Color color, double scale, YoVariableRegistry registry)
   {
      this(namePrefix+nameSuffix, new DoubleYoVariable(namePrefix + "X" + nameSuffix, registry), new DoubleYoVariable(namePrefix + "Y" + nameSuffix, registry), type, color, scale);
   }
   
   public YoArtifactPosition(String name, DoubleYoVariable x, DoubleYoVariable y, YoGraphicPosition.GraphicType type, Color color, double scale)
   {
      super(name);
      this.x = x;
      this.y = y;
      
      this.graphicType = type;
      this.color = color;
      this.scale = scale;
   }

   public void takeHistorySnapshot()
   {
      if (getRecordHistory())
      {
         synchronized(historicalPositions)
         {
            historicalPositions.add(new double[]{x.getDoubleValue(), y.getDoubleValue()});
         }
      }
   }
   
   private final Point3d position = new Point3d();
   private static final int MIN_RADIUS = 5;
   private static final int MAX_RADIUS = 25;

   public void drawLegend(Graphics graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      double radius = Math.round(4.0 * scale * scaleFactor);
      radius = MathTools.clipToMinMax(radius, MIN_RADIUS, MAX_RADIUS);
      draw(graphics, Xcenter, Ycenter, (int) radius);
   }

   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         getPosition(position);
         draw(graphics, position.x, position.y, Xcenter, Ycenter, scaleFactor);
      }
   }
   
   public void getPosition(Point3d point3d)
   {
      point3d.set(x.getDoubleValue(), y.getDoubleValue(), 0.0);
   }
   
   public void setPosition(Point2d point2d)
   {
      x.set(point2d.getX());
      y.set(point2d.getY());
   }

   
   private void draw(Graphics graphics, double xWorld, double yWorld, int Xcenter, int Ycenter, double scaleFactor)
   {
      if (Double.isNaN(xWorld) || Double.isNaN(yWorld))
         return;

      int x = (int) (xWorld * scaleFactor) + Xcenter;
      int y = (int) (-yWorld * scaleFactor) + Ycenter;
      int radius = (int) (4.0 * scale * scaleFactor);

      draw(graphics, x, y, radius);
   }


   private void draw(Graphics graphics, int x, int y, int radius)
   {
      YoGraphicPosition.GraphicType type = graphicType;
      switch (type)
      {
         case BALL :
            Drawing2DTools.drawEmptyCircle(graphics, x, y, radius, color);

            break;

         case SOLID_BALL :
            Drawing2DTools.drawFilledCircle(graphics, x, y, radius, color);

            break;

         case CROSS :
            Drawing2DTools.drawCircleWithCross(graphics, x, y, radius, color);

            break;

         case BALL_WITH_CROSS :
            Drawing2DTools.drawCircleWithCross(graphics, x, y, radius, color);

            break;

         case ROTATED_CROSS :
            Drawing2DTools.drawCircleWithRotatedCross(graphics, x, y, radius, color);

            break;

         case BALL_WITH_ROTATED_CROSS :
            Drawing2DTools.drawCircleWithRotatedCross(graphics, x, y, radius, color);

            break;

         case DIAMOND :
            Drawing2DTools.drawDiamond(graphics, x, y, radius, color);

            break;

         case DIAMOND_WITH_CROSS :
            Drawing2DTools.drawDiamondWithCross(graphics, x, y, radius, color);

            break;

         case SQUARE :
            Drawing2DTools.drawDiamondWithCross(graphics, x, y, radius, color);

            break;

         case SQUARE_WITH_CROSS :
            Drawing2DTools.drawDiamondWithCross(graphics, x, y, radius, color);

            break;
      default:
         throw new RuntimeException("type "+ type +" not implemented");
      }
   }
   
   public void drawHistory(Graphics graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      synchronized(historicalPositions)
      {
         for (double[] position : historicalPositions)
         {
            double xWorld = position[0];
            double yWorld = position[1];

            draw(graphics, xWorld, yWorld, Xcenter, Ycenter, scaleFactor);
         }
      }
   }

   public RemoteGraphicType getRemoteGraphicType() 
   {
      return RemoteGraphicType.POSITION_ARTIFACT;
   }
   
   public DoubleYoVariable[] getVariables() 
   {
      return new DoubleYoVariable[]{x, y};
   }

   public double[] getConstants() 
   {
      return new double[]{scale, graphicType.ordinal()};
   }
   
   public AppearanceDefinition getAppearance()
   {
      return new YoAppearanceRGBColor(color, 0.0);
   }
   
}
