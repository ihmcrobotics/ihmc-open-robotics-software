package us.ihmc.robotics.geometry;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Polygon;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;

import javax.swing.JPanel;
import javax.swing.event.MouseInputListener;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class FrameGeometry2dPlotter extends JPanel implements MouseInputListener
{
   private static final long serialVersionUID = 2505175418803166784L;

   private ArrayList<FramePoint2d> testPoints = new ArrayList<FramePoint2d>();

   private FrameConvexPolygon2d polygonToCheckInside = null;

   private LinkedHashMap<Color, FrameLineGroup> frameLineGroups = new LinkedHashMap<Color, FrameLineGroup>();
   private LinkedHashMap<Color, FramePointGroup> framePointGroups = new LinkedHashMap<Color, FramePointGroup>();
   private LinkedHashMap<Color, FrameConvexPolygonGroup> frameConvexPolygonGroups = new LinkedHashMap<Color, FrameConvexPolygonGroup>();
   private LinkedHashMap<Color, FrameLineSegmentGroup> frameLineSegmentGroups = new LinkedHashMap<Color, FrameLineSegmentGroup>();

   private double xCenter, yCenter, scale;
   private int pointPixels = 2;

   private class FrameLineGroup
   {
      private ArrayList<FrameLine2d> frameLines = new ArrayList<FrameLine2d>();
      private final Color color;

      public FrameLineGroup(Color color)
      {
         this.color = color;
      }

      public void addFrameLine(FrameLine2d frameLine2d)
      {
         frameLines.add(frameLine2d);
      }

      public void addFrameLines(ArrayList<FrameLine2d> frameLines2d)
      {
         frameLines.addAll(frameLines2d);
      }

      public void addFrameLines(FrameLine2d[] frameLines2d)
      {
         for (FrameLine2d frameLine : frameLines2d)
         {
            frameLines.add(frameLine);
         }
      }
   }

   private class FramePointGroup
   {
      private ArrayList<FramePoint2d> framePoints = new ArrayList<FramePoint2d>();
      private final Color color;

      public FramePointGroup(Color color)
      {
         this.color = color;
      }

      public void addFramePoint(FramePoint2d framePoint2d)
      {
         framePoints.add(framePoint2d);
      }

      public void addFramePoints(ArrayList<FramePoint2d> framePoints2d)
      {
         framePoints.addAll(framePoints2d);
      }

      @SuppressWarnings("unused")
      public void addFramePoints(FramePoint2d[] framePoints2d)
      {
         for (FramePoint2d framePoint : framePoints2d)
         {
            framePoints.add(framePoint);
         }
      }

   }

   private class FrameConvexPolygonGroup
   {
      private ArrayList<FrameConvexPolygon2d> polygons = new ArrayList<FrameConvexPolygon2d>();
      private final Color color;

      public FrameConvexPolygonGroup(Color color)
      {
         this.color = color;
      }

      public void addFrameConvexPolygon2d(FrameConvexPolygon2d frameConvexPolygon2d)
      {
         polygons.add(frameConvexPolygon2d);
      }

      public void addFrameConvexPolygon2ds(ArrayList<FrameConvexPolygon2d> frameConvexPolygon2ds)
      {
         polygons.addAll(frameConvexPolygon2ds);
      }

      @SuppressWarnings("unused")
      public void addFrameConvexPolygon2ds(FrameConvexPolygon2d[] frameConvexPolygon2ds)
      {
         for (FrameConvexPolygon2d frameConvexPolygon2d : frameConvexPolygon2ds)
         {
            polygons.add(frameConvexPolygon2d);
         }
      }
   }

   private class FrameLineSegmentGroup
   {
      private ArrayList<FrameLineSegment2d> frameLineSegments = new ArrayList<FrameLineSegment2d>();
      private final Color color;

      public FrameLineSegmentGroup(Color color)
      {
         this.color = color;
      }

      public void addFrameLineSegment(FrameLineSegment2d frameLineSegment2d)
      {
         frameLineSegments.add(frameLineSegment2d);
      }

      public void addFrameLineSegments(ArrayList<FrameLineSegment2d> frameLineSegments2d)
      {
         frameLineSegments.addAll(frameLineSegments2d);
      }
   }

   public FrameGeometry2dPlotter(double xCenter, double yCenter, double scale)
   {
      this.scale = scale;
      this.xCenter = xCenter;
      this.yCenter = yCenter;

      this.addMouseListener(this);
      this.addMouseMotionListener(this);
   }

   public void setDrawPointsLarge()
   {
      pointPixels = 8;
   }

   public void setDrawPointsMedium()
   {
      pointPixels = 4;
   }

   public void setPointPixels(int pointPixels)
   {
      this.pointPixels = pointPixels;
   }

   public synchronized void addFrameLine2d(FrameLine2d frameLine2d)
   {
      addFrameLine2d(frameLine2d, Color.black);
   }

   public synchronized void addFrameLines2d(ArrayList<FrameLine2d> frameLines2d)
   {
      addFrameLines2d(frameLines2d, Color.black);
   }

   public synchronized void addFrameLine2d(FrameLine2d frameLine2d, Color color)
   {
      FrameLineGroup frameLineGroup = frameLineGroups.get(color);
      if (frameLineGroup == null)
      {
         frameLineGroup = new FrameLineGroup(color);
         frameLineGroups.put(color, frameLineGroup);
      }

      frameLineGroup.addFrameLine(frameLine2d);
   }

   public synchronized void addFramePoints2d(ArrayList<FramePoint2d> framePoints, Color color)
   {
      FramePointGroup framePointGroup = framePointGroups.get(color);
      if (framePointGroup == null)
      {
         framePointGroup = new FramePointGroup(color);
         framePointGroups.put(color, framePointGroup);
      }

      framePointGroup.addFramePoints(framePoints);
   }

   public synchronized void addFramePoint2d(FramePoint2d framePoint, Color color)
   {
      FramePointGroup framePointGroup = framePointGroups.get(color);
      if (framePointGroup == null)
      {
         framePointGroup = new FramePointGroup(color);
         framePointGroups.put(color, framePointGroup);
      }

      framePointGroup.addFramePoint(framePoint);
   }

   public synchronized void addFrameLines2d(ArrayList<FrameLine2d> frameLines2d, Color color)
   {
      FrameLineGroup frameLineGroup = frameLineGroups.get(color);
      if (frameLineGroup == null)
      {
         frameLineGroup = new FrameLineGroup(color);
         frameLineGroups.put(color, frameLineGroup);
      }

      frameLineGroup.addFrameLines(frameLines2d);

   }

   public synchronized void addFrameLines2d(FrameLine2d[] frameLines2d, Color color)
   {
      FrameLineGroup frameLineGroup = frameLineGroups.get(color);
      if (frameLineGroup == null)
      {
         frameLineGroup = new FrameLineGroup(color);
         frameLineGroups.put(color, frameLineGroup);
      }

      frameLineGroup.addFrameLines(frameLines2d);
   }

   public synchronized void addFrameLineSegment2d(FrameLineSegment2d frameLinesSegment2d, Color color)
   {
      FrameLineSegmentGroup frameLineSegmentGroup = frameLineSegmentGroups.get(color);

      if (frameLineSegmentGroup == null)
      {
         frameLineSegmentGroup = new FrameLineSegmentGroup(color);
         frameLineSegmentGroups.put(color, frameLineSegmentGroup);
      }

      frameLineSegmentGroup.addFrameLineSegment(frameLinesSegment2d);

   }

   public synchronized void addFrameLineSegments2d(ArrayList<FrameLineSegment2d> frameLinesSegments2d, Color color)
   {
      FrameLineSegmentGroup frameLineSegmentGroup = frameLineSegmentGroups.get(color);

      if (frameLineSegmentGroup == null)
      {
         frameLineSegmentGroup = new FrameLineSegmentGroup(color);
         frameLineSegmentGroups.put(color, frameLineSegmentGroup);
      }

      frameLineSegmentGroup.addFrameLineSegments(frameLinesSegments2d);
   }

   public synchronized void addPolygon(FrameConvexPolygon2d polygon)
   {
      addPolygon(polygon, Color.BLACK);
   }

   public synchronized void addPolygon(FrameConvexPolygon2d polygon, Color color)
   {
      FrameConvexPolygonGroup frameConvexPolygonGroup = frameConvexPolygonGroups.get(color);
      if (frameConvexPolygonGroup == null)
      {
         frameConvexPolygonGroup = new FrameConvexPolygonGroup(color);
         frameConvexPolygonGroups.put(color, frameConvexPolygonGroup);
      }

      frameConvexPolygonGroup.addFrameConvexPolygon2d(polygon);
   }

   public synchronized void addFrameConvexPolygons(ArrayList<FrameConvexPolygon2d> frameConvexPolygons, Color color)
   {
      FrameConvexPolygonGroup frameConvexPolygonGroup = frameConvexPolygonGroups.get(color);
      if (frameConvexPolygonGroup == null)
      {
         frameConvexPolygonGroup = new FrameConvexPolygonGroup(color);
         frameConvexPolygonGroups.put(color, frameConvexPolygonGroup);
      }

      frameConvexPolygonGroup.addFrameConvexPolygon2ds(frameConvexPolygons);
   }

   public synchronized void addConvexPolygons(ArrayList<ConvexPolygon2d> convexPolygons, Color color)
   {
      FrameConvexPolygonGroup frameConvexPolygonGroup = frameConvexPolygonGroups.get(color);
      if (frameConvexPolygonGroup == null)
      {
         frameConvexPolygonGroup = new FrameConvexPolygonGroup(color);
         frameConvexPolygonGroups.put(color, frameConvexPolygonGroup);
      }

      ArrayList<FrameConvexPolygon2d> frameConvexPolygons = new ArrayList<FrameConvexPolygon2d>();
      for (ConvexPolygon2d polygon : convexPolygons)
      {
         frameConvexPolygons.add(new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), polygon));
      }

      frameConvexPolygonGroup.addFrameConvexPolygon2ds(frameConvexPolygons);
   }

   public synchronized void addConvexPolygons(ConvexPolygon2d[] convexPolygons, Color color)
   {
      FrameConvexPolygonGroup frameConvexPolygonGroup = frameConvexPolygonGroups.get(color);
      if (frameConvexPolygonGroup == null)
      {
         frameConvexPolygonGroup = new FrameConvexPolygonGroup(color);
         frameConvexPolygonGroups.put(color, frameConvexPolygonGroup);
      }

      ArrayList<FrameConvexPolygon2d> frameConvexPolygons = new ArrayList<FrameConvexPolygon2d>();
      for (ConvexPolygon2d polygon : convexPolygons)
      {
         if (polygon == null)
            continue;
         frameConvexPolygons.add(new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), polygon));
      }

      frameConvexPolygonGroup.addFrameConvexPolygon2ds(frameConvexPolygons);
   }

   public synchronized void addConvexPolygon(ConvexPolygon2d convexPolygon, Color color)
   {
      FrameConvexPolygonGroup frameConvexPolygonGroup = frameConvexPolygonGroups.get(color);
      if (frameConvexPolygonGroup == null)
      {
         frameConvexPolygonGroup = new FrameConvexPolygonGroup(color);
         frameConvexPolygonGroups.put(color, frameConvexPolygonGroup);
      }

      ArrayList<FrameConvexPolygon2d> frameConvexPolygons = new ArrayList<FrameConvexPolygon2d>();

      if (convexPolygon == null)
         return;
      frameConvexPolygons.add(new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), convexPolygon));

      frameConvexPolygonGroup.addFrameConvexPolygon2ds(frameConvexPolygons);
   }

   public synchronized void addFramePoint2d(FramePoint2d framePoint2d)
   {
      this.addFramePoint2d(framePoint2d, Color.black);
   }

   public synchronized void addTestPoint(FramePoint2d testPoint)
   {
      testPoints.add(testPoint);
   }

   public synchronized void setPolygonToCheckInside(FrameConvexPolygon2d polygon)
   {
      this.polygonToCheckInside = polygon;
   }

   public synchronized void addTestPoints(ArrayList<FramePoint2d> testPoints)
   {
      this.testPoints.addAll(testPoints);
      repaint();
   }

   public synchronized void removeAllObjectsToDraw()
   {
      testPoints.clear();

      polygonToCheckInside = null;

      frameLineGroups.clear();
      framePointGroups.clear();
      frameConvexPolygonGroups.clear();
      frameLineSegmentGroups.clear();

      repaint();
   }

   public synchronized void paintComponent(Graphics graphics)
   {
      super.paintComponent(graphics);

      if (polygonToCheckInside != null)
         drawPolygon(polygonToCheckInside, graphics);

      drawFramePolygonGroups(frameConvexPolygonGroups, graphics);
      drawFrameLineGroups(frameLineGroups, graphics);
      drawFrameLineSegmentGroups(frameLineSegmentGroups, graphics);

      drawTestPoints(testPoints, polygonToCheckInside, graphics);

      //    drawFramePoint2ds(framePoint2ds, graphics);
      drawFramePointGroups(framePointGroups, graphics);
   }

   private void drawFramePolygonGroups(HashMap<Color, FrameConvexPolygonGroup> frameConvexPolygonGroups, Graphics graphics)
   {
      Collection<FrameConvexPolygonGroup> groups = frameConvexPolygonGroups.values();

      for (FrameConvexPolygonGroup frameConvexPolygonGroup : groups)
      {
         graphics.setColor(frameConvexPolygonGroup.color);

         //       drawFrameConvexPolygon(frameConvexPolygonGroup.polygons, graphics);
         drawPolygons(frameConvexPolygonGroup.polygons, graphics);
      }

   }

   private void drawFrameLineGroups(HashMap<Color, FrameLineGroup> frameLineGroups, Graphics graphics)
   {
      Collection<FrameLineGroup> groups = frameLineGroups.values();

      for (FrameLineGroup frameLineGroup : groups)
      {
         graphics.setColor(frameLineGroup.color);
         drawFrameLines(frameLineGroup.frameLines, graphics);
      }
   }

   private void drawFrameLineSegmentGroups(HashMap<Color, FrameLineSegmentGroup> frameLineSegmentGroups, Graphics graphics)
   {
      Collection<FrameLineSegmentGroup> groups = frameLineSegmentGroups.values();

      for (FrameLineSegmentGroup frameLineSegmentGroup : groups)
      {
         graphics.setColor(frameLineSegmentGroup.color);
         drawFrameLineSegments(frameLineSegmentGroup.frameLineSegments, graphics);
      }
   }

   private void drawFramePointGroups(HashMap<Color, FramePointGroup> framePointGroups, Graphics graphics)
   {
      Collection<FramePointGroup> groups = framePointGroups.values();

      for (FramePointGroup framePointGroup : groups)
      {
         graphics.setColor(framePointGroup.color);
         drawFramePoints(framePointGroup.framePoints, graphics);
      }
   }

   private void drawFrameLines(ArrayList<FrameLine2d> frameLines, Graphics graphics)
   {
      for (FrameLine2d frameLine : frameLines)
      {
         this.drawLine(frameLine, graphics);
      }
   }

   private void drawFrameLineSegments(ArrayList<FrameLineSegment2d> frameLineSegments, Graphics graphics)
   {
      for (FrameLineSegment2d frameLineSegment : frameLineSegments)
      {
         this.drawLineSegment(frameLineSegment, graphics);
      }
   }

   private void drawFramePoints(ArrayList<FramePoint2d> framePoints, Graphics graphics)
   {
      for (FramePoint2d framePoint : framePoints)
      {
         this.drawPoint(framePoint, graphics);
      }
   }

   private void drawTestPoints(ArrayList<FramePoint2d> testPoints, FrameConvexPolygon2d polygon, Graphics graphics)
   {
      for (FramePoint2d testPoint : testPoints)
      {
         boolean isInside = polygon.isPointInside(testPoint);

         if (isInside)
         {
            drawInsidePoint(testPoint, graphics);
         }
         else
         {
            drawOutsidePoint(testPoint, graphics);
         }
      }
   }

   @SuppressWarnings("unused")
   private void drawPoint(FramePoint2d testPoint, Color color, Graphics graphics)
   {
      int xInt = doubleToInt(testPoint.getX(), xCenter, scale, getWidth());
      int yInt = doubleToInt(testPoint.getY(), yCenter, -scale, getHeight());

      graphics.setColor(color);
      graphics.fillOval(xInt - 2, yInt - 2, 4, 4);
   }

   private void drawInsidePoint(FramePoint2d testPoint, Graphics graphics)
   {
      int xInt = doubleToInt(testPoint.getX(), xCenter, scale, getWidth());
      int yInt = doubleToInt(testPoint.getY(), yCenter, -scale, getHeight());

      graphics.setColor(Color.green);
      graphics.fillOval(xInt - 2, yInt - 2, 4, 4);
   }

   private void drawOutsidePoint(FramePoint2d testPoint, Graphics graphics)
   {
      int xInt = doubleToInt(testPoint.getX(), xCenter, scale, getWidth());
      int yInt = doubleToInt(testPoint.getY(), yCenter, -scale, getHeight());

      graphics.setColor(Color.red);
      graphics.fillOval(xInt - 2, yInt - 2, 4, 4);
   }

   private void drawPolygons(ArrayList<FrameConvexPolygon2d> polygons, Graphics graphics)
   {
      for (FrameConvexPolygon2d polygon : polygons)
      {
         drawPolygon(polygon, graphics);
      }
   }

   private void drawPolygon(FrameConvexPolygon2d frameConvexPolygon2d, Graphics graphics)
   {
      Polygon polygon = new Polygon();

      for (int i = 0; i < frameConvexPolygon2d.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = frameConvexPolygon2d.getVertex(i);
         int xInt = doubleToInt(vertex.getX(), xCenter, scale, getWidth());
         int yInt = doubleToInt(vertex.getY(), yCenter, -scale, getHeight());

         polygon.addPoint(xInt, yInt);
      }

      graphics.drawPolygon(polygon);
   }

   private void drawLine(FrameLine2d frameLine2d, Graphics graphics)
   {
      Point2D point = new Point2D();
      frameLine2d.getPoint2d(point);
      Vector2D vector = new Vector2D();
      frameLine2d.getNormalizedFrameVector(vector);

      double largeNumber = scale * 100; // 1.0e3;

      double X2 = point.getX() + largeNumber * vector.getX();
      double Y2 = point.getY() + largeNumber * vector.getY();

      drawLine(point.getX(), point.getY(), X2, Y2, graphics);
   }

   private void drawLineSegment(FrameLineSegment2d frameLineSegment2d, Graphics graphics)
   {
      Point2D[] points = frameLineSegment2d.lineSegment.getEndpointsCopy();

      drawLine(points[0].getX(), points[0].getY(), points[1].getX(), points[1].getY(), graphics);
   }

   private void drawPoint(FramePoint2d framePoint2d, Graphics graphics)
   {
      drawPoint(framePoint2d.getX(), framePoint2d.getY(), graphics);
   }

   private void drawLine(double X1, double Y1, double X2, double Y2, Graphics graphics)
   {
      int x1 = doubleToInt(X1, xCenter, scale, getWidth());
      int x2 = doubleToInt(X2, xCenter, scale, getWidth());
      int y1 = doubleToInt(Y1, yCenter, -scale, getHeight());
      int y2 = doubleToInt(Y2, yCenter, -scale, getHeight());

      graphics.drawLine(x1, y1, x2, y2);
   }

   private void drawPoint(double X1, double Y1, Graphics graphics)
   {
      int x1 = doubleToInt(X1, xCenter, scale, getWidth());
      int y1 = doubleToInt(Y1, yCenter, -scale, getHeight());

      graphics.fillOval(x1 - pointPixels / 2, y1 - pointPixels / 2, pointPixels, pointPixels);
   }

   private int doubleToInt(double value, double center, double scale, int extent)
   {
      return (int) ((value - center) * scale + extent / 2);
   }

   private double intToDouble(int value, double center, double scale, int extent)
   {
      return (value - extent / 2) / scale + center;
   }

   private double lastMousePressX, lastMousePressY;

   public void mouseClicked(MouseEvent e)
   {
      int clickCount = e.getClickCount();

      if (clickCount > 1)
      {
         this.xCenter = lastMousePressX;
         this.yCenter = lastMousePressY;

         this.repaint();
      }

   }

   private int buttonPressed;
   private int dragStartX;
   private int dragStartY;

   public void mousePressed(MouseEvent e)
   {
      buttonPressed = e.getButton();

      int xInt = e.getX();
      int yInt = e.getY();

      double x = intToDouble(xInt, xCenter, scale, getWidth());
      double y = intToDouble(yInt, yCenter, -scale, getHeight());

      lastMousePressX = x;
      lastMousePressY = y;

      if (buttonPressed == MouseEvent.BUTTON2)
      {
         dragStartY = e.getY();
      }
   }

   public void mouseReleased(MouseEvent e)
   {
   }

   public void mouseEntered(MouseEvent e)
   {
   }

   public void mouseExited(MouseEvent e)
   {
   }

   public void mouseDragged(MouseEvent e)
   {
      if (buttonPressed == MouseEvent.BUTTON2)
      {
         int yDifferenceFromStartOfDrag = -(e.getY() - dragStartY);
         double scalePercent = 1.0 + (((double) yDifferenceFromStartOfDrag) / 200.0);
         if (scalePercent < 0.1)
            scalePercent = 0.1;

         double newScale = scale * scalePercent;
         if (newScale < 0.1)
            newScale = 0.1;

         scale = newScale;
         dragStartY = e.getY();
         repaint();
      }
   }

   public void mouseMoved(MouseEvent e)
   {
   }

}
