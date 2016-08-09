package us.ihmc.plotting;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.border.Border;
import javax.swing.event.MouseInputAdapter;
import javax.vecmath.Point2d;

import us.ihmc.plotting.shapes.LineArtifact;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.plotting.shapes.PolygonArtifact;
import us.ihmc.plotting.shapes.ShapeArtifact;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.tools.FormattingTools;

public class Plotter extends JPanel
{
   private static final long serialVersionUID = 3113130298799362369L;

   private final ArrayList<ArtifactsChangedListener> artifactsChangedListeners = new ArrayList<ArtifactsChangedListener>();

   // show selections
   private static final boolean SHOW_SELECTION = false;
   private static final Color TEXT_COLOR = Color.WHITE;

   private boolean drawHistory = false;
   private boolean showLabels = false;

   private long updateDelayInMillis = 0;
   private long lastUpdate = 0;

   private BufferedImage backgroundImage = null;

   private Dimension preferredSize = new Dimension(275, 275);
   private final HashMap<String, Artifact> artifacts = new HashMap<String, Artifact>();

   private int centerX;
   private int centerY;
   private double offsetX = 0.0;
   private double offsetY = 0.0;
   private double plotRange = 20.0;
   private double scaleFactor;
   private double upperLeftLongitude, upperLeftLatitude, lowerRightLongitude, lowerRightLatitude;
   private double selectedX = 0.0;
   private double selectedY = 0.0;

   // rectangle area selection
   private double area1X = 0.0;
   private double area1Y = 0.0;
   private double area2X = 0.0;
   private double area2Y = 0.0;
   private double area1XTemp = 0.0;
   private double area1YTemp = 0.0;

   // drag tracking
   private int buttonPressed;
   private int dragStartY;
   // double click listener
   private DoubleClickListener doubleClickListener;

   private PolygonArtifact polygonArtifact;

   private boolean overrideAutomaticInterval = false;
   private double manualOverideInterval = 1.0;

   public Plotter()
   {
      // Initialize class variables
      this.setDoubleBuffered(true);

      // Set LayoutManager to null
      this.setLayout(null);

      // simpane
      Border raisedBevel = BorderFactory.createRaisedBevelBorder();
      Border loweredBevel = BorderFactory.createLoweredBevelBorder();
      Border compound = BorderFactory.createCompoundBorder(raisedBevel, loweredBevel);
      setBorder(compound);

      super.setBackground(new Color(180, 220, 240));

      PlotterMouseListener myListener = new PlotterMouseListener();
      this.addMouseListener(myListener);
      this.addMouseMotionListener(myListener);
   }

   public void setDrawHistory(boolean drawHistory)
   {
      this.drawHistory = drawHistory;
   }

   @Override
   public void paintComponent(Graphics graphics)
   {
      long currentTime = System.currentTimeMillis();
      if ((currentTime - lastUpdate) > updateDelayInMillis)
      {
         Graphics2D graphics2d = (Graphics2D) graphics;

         // get current size and determine scaling factor
         Dimension panelSize = this.getSize();
         int heightInPixels = (int) Math.round(panelSize.getHeight());
         int widthInPixels = (int) Math.round(panelSize.getWidth());
         centerX = widthInPixels / 2;
         centerY = heightInPixels / 2;
         scaleFactor = heightInPixels / plotRange;

         double headingOffset = 0.0;

         // set current offset
         centerX = centerX - (int) Math.round(offsetX * scaleFactor);
         centerY = centerY + (int) Math.round(offsetY * scaleFactor);

         // paint background
         super.paintComponent(graphics2d);

         // Paint all artifacts that want to be under grid (86)
         synchronized (artifacts)
         {
            for (Artifact artifact : artifacts.values())
            {
               if (artifact != null)
               {
                  if (artifact.getLevel() == 86)
                  {
                     artifact.draw(graphics2d, centerX, centerY, headingOffset, scaleFactor);
                  }
               }
               else
               {
                  System.out.println("Plotter: one of the artifacts you added was null");
               }
            }
         }

         // if we start out at 100%, then the range will be equal to the extent of the map
         // if we zoom in, the range will be less, and so we need to compute from the current
         // center the zoomed in area.
         if (backgroundImage != null)
         {
            Coordinate upperLeft = new Coordinate(upperLeftLongitude, upperLeftLatitude, Coordinate.METER);
            Coordinate lowerRight = new Coordinate(lowerRightLongitude, lowerRightLatitude, Coordinate.METER);
            Coordinate upperLeftPixels = convertFromMetersToPixels(upperLeft);
            Coordinate lowerRightPixels = convertFromMetersToPixels(lowerRight);

            int upperLeftX = (int) upperLeftPixels.getX();
            int upperLeftY = (int) upperLeftPixels.getY();
            int lowerRightX = (int) lowerRightPixels.getX();
            int lowerRightY = (int) lowerRightPixels.getY();

            graphics2d.drawImage(backgroundImage, upperLeftX, upperLeftY, lowerRightX, lowerRightY, 0, 0, backgroundImage.getWidth(), backgroundImage.getHeight(), this);
         }
         else
         {
            // change grid line scale from 1m to 10cm ehn below 10m
            double interval = Math.pow(10, MathTools.orderOfMagnitude(plotRange) - 1);

            if (overrideAutomaticInterval)
            {
               interval = manualOverideInterval;
            }

            // paint grid lines
            Coordinate upperLeftCoodinate = convertFromPixelsToMeters(new Coordinate(0, 0, Coordinate.PIXEL));
            Coordinate lowerRightCoordinate = convertFromPixelsToMeters(new Coordinate(widthInPixels, heightInPixels, Coordinate.PIXEL));

            double upperLeftCoordinateX = upperLeftCoodinate.getX();
            double lowerRightCoordinateX = lowerRightCoordinate.getX();
            double deltaX = lowerRightCoordinateX - upperLeftCoordinateX;
            int numberOfGridLinesX = (int) Math.round(Math.ceil(deltaX / interval));
            int xCountOffset = (int) Math.floor(upperLeftCoordinateX / interval);
            double minXforPlotting = xCountOffset * interval;

            for (int i = 0; i < numberOfGridLinesX; i++)
            {
               double distance = minXforPlotting + (i * interval);

               if ((i + xCountOffset) % 10 == 0)
                  graphics2d.setColor(new Color(180, 190, 210));
               else if ((i + xCountOffset) % 5 == 0)
                  graphics2d.setColor(new Color(180, 210, 230));
               else
                  graphics2d.setColor(new Color(230, 240, 250));

               // get pixel from meter for positive
               Coordinate coord = convertFromMetersToPixels(new Coordinate(distance, distance, Coordinate.METER));
               int x = (int) Math.round(coord.getX());

               // draw line
               graphics2d.drawLine(x, 0, x, heightInPixels);
               
               if (showLabels)
               {
                  Color tempColor = graphics2d.getColor();
                  graphics2d.setColor(TEXT_COLOR);
                  graphics2d.drawString(FormattingTools.getFormattedToSignificantFigures(distance, 2), x + 1, heightInPixels / 2 - 1);
                  graphics2d.setColor(tempColor);
               }
            }

            double lowerRightCoordinateY = lowerRightCoordinate.getY();
            double upperLeftCoordinateY = upperLeftCoodinate.getY();
            double deltaY = upperLeftCoordinateY - lowerRightCoordinateY;
            int numberOfGridLinesY = (int) Math.round(Math.ceil(deltaY / interval));
            int yCountOffset = (int) Math.floor(lowerRightCoordinateY / interval);
            double minYforPlotting = yCountOffset * interval;

            for (int i = 0; i < numberOfGridLinesY; i++)
            {
               double distance = minYforPlotting + (i * interval);

               if ((i + yCountOffset) % 10 == 0)
                  graphics2d.setColor(new Color(180, 190, 210));
               else if ((i + yCountOffset) % 5 == 0)
                  graphics2d.setColor(new Color(180, 210, 230));
               else
                  graphics2d.setColor(new Color(230, 240, 250));

               // get pixel from meter for positive
               Coordinate coord = convertFromMetersToPixels(new Coordinate(distance, distance, Coordinate.METER));
               int y = (int) Math.round(coord.getY());

               // draw line
               graphics2d.drawLine(0, y, widthInPixels, y);
               
               if (showLabels)
               {
                  Color tempColor = graphics2d.getColor();
                  graphics2d.setColor(TEXT_COLOR);
                  graphics2d.drawString(FormattingTools.getFormattedToSignificantFigures(distance, 2), widthInPixels / 2 + 1, y - 1);
                  graphics2d.setColor(tempColor);
               }
            }
         }

         // paint grid centerline
         graphics2d.setColor(Color.gray);

         // get pixel from meter for positive
         Coordinate coord = convertFromMetersToPixels(new Coordinate(0, 0, Coordinate.METER));
         int x0 = (int) Math.round(coord.getX());
         int y0 = (int) Math.round(coord.getY());

         // draw line
         graphics2d.drawLine(x0, 0, x0, heightInPixels);
         graphics2d.drawLine(0, y0, widthInPixels, y0);

         // Paint all artifacts history by level
         // (assumes 5 levels (0-4)
         if (drawHistory)
         {
            synchronized (artifacts)
            {
               for (int i = 0; i < 5; i++)
               {
                  for (Artifact artifact : artifacts.values())
                  {
                     // get next element
                     if (artifact != null)
                     {
                        if (artifact.getDrawHistory() && (artifact.getLevel() == i))
                        {
                           artifact.drawHistory(graphics2d, centerX, centerY, scaleFactor);
                        }
                     }
                     else
                     {
                        System.out.println(">>> a = " + artifact);
                     }
                  }
               }
            }
         }

         // Paint all artifacts by level
         // (assumes 5 levels (0-4)
         synchronized (artifacts)
         {
            for (int i = 0; i < 5; i++)
            {
               for (Artifact artifact : artifacts.values())
               {
                  // get next element
                  if (artifact != null)
                  {
                     if (artifact.getLevel() == i)
                     {
                        artifact.draw(graphics2d, centerX, centerY, headingOffset, scaleFactor); // , _orientation);
                     }
                  }
                  else
                  {
                     System.out.println(">>> a = " + artifact);
                  }
               }
            }
         }

         // paint selected destination
         if (SHOW_SELECTION)
         {
            graphics2d.setColor(Color.red);
            int xSize = 8;
            int xX = centerX + ((int) Math.round(selectedX * scaleFactor));
            int yX = centerY - ((int) Math.round(selectedY * scaleFactor));
            graphics2d.drawLine(xX - xSize, yX - xSize, xX + xSize, yX + xSize);
            graphics2d.drawLine(xX - xSize, yX + xSize, xX + xSize, yX - xSize);
         }

         // paint selected area
         if (SHOW_SELECTION)
         {
            graphics2d.setColor(Color.red);
            int areaX1Int = centerX + ((int) Math.round(area1X * scaleFactor));
            int areaY1Int = centerY - ((int) Math.round(area1Y * scaleFactor));
            int areaX2Int = centerX + ((int) Math.round(area2X * scaleFactor));
            int areaY2Int = centerY - ((int) Math.round(area2Y * scaleFactor));
            int Xmin, Xmax, Ymin, Ymax;
            if (areaX1Int > areaX2Int)
            {
               Xmax = areaX1Int;
               Xmin = areaX2Int;
            }
            else
            {
               Xmax = areaX2Int;
               Xmin = areaX1Int;
            }

            if (areaY1Int > areaY2Int)
            {
               Ymax = areaY1Int;
               Ymin = areaY2Int;
            }
            else
            {
               Ymax = areaY2Int;
               Ymin = areaY1Int;
            }

            graphics2d.drawRect(Xmin, Ymin, (Xmax - Xmin), (Ymax - Ymin));
         }

         lastUpdate = currentTime;
      }
   }

   public void setBackgroundImage(BufferedImage bgi)
   {
      if (bgi == null)
      {
         System.out.println("Passed in NULL as a background image");
      }
      else
      {
         backgroundImage = bgi;
         repaint();
      }
   }

   public void setManualGidInterval(double intervalInMeters)
   {
      this.manualOverideInterval = intervalInMeters;
      this.overrideAutomaticInterval = true;
   }

   public void disableManualGridIntervalOverride()
   {
      this.overrideAutomaticInterval = false;
   }

   public void updateArtifacts(Vector<Artifact> artifacts)
   {
      this.artifacts.clear();

      for (Artifact a : artifacts)
      {
         this.artifacts.put(a.getID(), a);
      }

      notifyArtifactsChangedListeners();
      repaint();
   }

   public void updateArtifact(Artifact newArtifact)
   {
      synchronized (artifacts)
      {
         artifacts.put(newArtifact.getID(), newArtifact);
      }

      notifyArtifactsChangedListeners();
      repaint();
   }

   public void updateArtifactNoRePaint(Artifact newArtifact)
   {
      synchronized (artifacts)
      {
         artifacts.put(newArtifact.getID(), newArtifact);
      }

      notifyArtifactsChangedListeners();
   }

   public LineArtifact createAndAddLineArtifact(String name, Line2d line, Color color)
   {
      LineArtifact lineArtifact = new LineArtifact(name, line);
      lineArtifact.setColor(color);
      addArtifact(lineArtifact);

      return lineArtifact;
   }

   public PointArtifact createAndAddPointArtifact(String name, Point2d point, Color color)
   {
      PointArtifact pointArtifact = new PointArtifact(name, point);
      pointArtifact.setColor(color);
      addArtifact(pointArtifact);

      return pointArtifact;
   }

   public void addArtifact(Artifact newArtifact)
   {
      synchronized (artifacts)
      {
         artifacts.put(newArtifact.getID(), newArtifact);
      }

      notifyArtifactsChangedListeners();
      repaint();
   }

   public void addArtifactNoRepaint(Artifact newArtifact)
   {
      synchronized (artifacts)
      {
         artifacts.put(newArtifact.getID(), newArtifact);
      }

      notifyArtifactsChangedListeners();
   }

   public ArrayList<Artifact> getArtifacts()
   {
      ArrayList<Artifact> ret = new ArrayList<Artifact>();

      ret.addAll(artifacts.values());

      return ret;
   }

   public Artifact getArtifact(String id)
   {
      return artifacts.get(id);
   }

   public void replaceArtifact(String id, Artifact newArtifact)
   {
      synchronized (artifacts)
      {
         artifacts.put(newArtifact.getID(), newArtifact);
      }

      notifyArtifactsChangedListeners();
   }

   public void removeAllArtifacts()
   {
      synchronized (artifacts)
      {
         artifacts.clear();
      }

      notifyArtifactsChangedListeners();
      repaint();
   }

   public void removeArtifact(String id)
   {
      synchronized (artifacts)
      {
         artifacts.remove(id);
      }

      notifyArtifactsChangedListeners();
      repaint();
   }

   public void removeArtifactNoRepaint(String id)
   {
      synchronized (artifacts)
      {
         artifacts.remove(id);
      }

      notifyArtifactsChangedListeners();
   }

   public void removeArtifactsStartingWith(String id)
   {
      synchronized (artifacts)
      {
         ArrayList<String> toBeRemoved = new ArrayList<String>();
         for (String key : artifacts.keySet())
         {
            if (key.startsWith(id))
               toBeRemoved.add(key);
         }

         for (String key : toBeRemoved)
         {
            artifacts.remove(key);
         }
      }

      notifyArtifactsChangedListeners();
      repaint();
   }

   public double getSelectedX()
   {
      return selectedX;
   }

   public double getSelectedY()
   {
      return selectedY;
   }

   public double getArea1X()
   {
      return area1X;
   }

   public double getArea1Y()
   {
      return area1Y;
   }

   public double getArea2X()
   {
      return area2X;
   }

   public double getArea2Y()
   {
      return area2Y;
   }

   public ArrayList<Point2d> getPolygon()
   {
      if (polygonArtifact == null)
         return null;

      return polygonArtifact.getPoints();
   }

   public void clearPolygon()
   {
      this.removeArtifactsStartingWith("polygon");
      polygonArtifact = null;
      repaint();
   }

   @SuppressWarnings("unused")
   private Point convertCoordinates(JPanel plot, Point2D.Double pt)
   {
      // get current size and determine scaling factor
      Dimension d = this.getSize();
      int h = (int) Math.round(d.getHeight());
      Math.round(d.getWidth());

      scaleFactor = h / plotRange;

      // detemine plot size
      Dimension plotD = plot.getSize();
      int plotH = (int) Math.round(plotD.getHeight());
      int plotW = (int) Math.round(plotD.getWidth());

      // place plot so bottom is cenetered at location
      int xnew = (int) Math.round((pt.x * scaleFactor) - (plotW / 2));
      int ynew = (int) Math.round((h - (pt.y * scaleFactor)) - (plotH));

      return new Point(xnew, ynew);
   }

   private double unConvertXCoordinate(int coordinate)
   {
      return new Integer(coordinate - centerX).doubleValue() / scaleFactor;
   }

   private double unConvertYCoordinate(int coordinate)
   {
      return new Integer((centerY - coordinate)).doubleValue() / scaleFactor;
   }

   private Coordinate convertFromMetersToPixels(Coordinate coordinate)
   {
      double x = coordinate.getX();
      double y = coordinate.getY();

      x = new Double(centerX + ((int) Math.round(x * scaleFactor))).doubleValue();

      y = new Double(centerY - ((int) Math.round(y * scaleFactor))).doubleValue();

      return new Coordinate(x, y, Coordinate.PIXEL);
   }

   private Coordinate convertFromPixelsToMeters(Coordinate coordinate)
   {
      double x = coordinate.getX();
      double y = coordinate.getY();

      x = (x - new Integer(centerX).doubleValue()) / scaleFactor;

      y = ((new Integer(centerY).doubleValue()) - y) / scaleFactor;

      return new Coordinate(x, y, Coordinate.METER);
   }

   public void setRangeLimit(int range, double origMapScale, double ullon, double ullat, double lrlon, double lrlat)
   {
      plotRange = range;
      upperLeftLongitude = ullon;
      upperLeftLatitude = ullat;
      lowerRightLongitude = lrlon;
      lowerRightLatitude = lrlat;

      repaint();
   }

   public void setRange(double range)
   {
      plotRange = range;
      repaint();
   }

   public double getRange()
   {
      return plotRange;
   }

   public void setOrientation(int orientation)
   {
      repaint();
   }

   public void setOffsetX(double offsetX)
   {
      this.offsetX = offsetX;
   }

   public void setOffsetY(double offsetY)
   {
      this.offsetY = offsetY;
   }

   public double getOffsetX()
   {
      return offsetX;
   }

   public double getOffsetY()
   {
      return offsetY;
   }

   @Override
   public Dimension getPreferredSize()
   {
      return preferredSize;
   }

   public void setPreferredSize(int width, int height)
   {
      preferredSize = new Dimension(width, height);
   }

   public void setDoubleClickListener(DoubleClickListener doubleClickListener)
   {
      this.doubleClickListener = doubleClickListener;
   }

   public void update(String objectID, ShapeArtifact shapeArtifact)
   {
      if (shapeArtifact == null)
      {
         removeArtifact(objectID);
      }
      else
      {
         if (shapeArtifact.getCoordinate() == null)
         {
            removeArtifact(shapeArtifact.getID());
         }
         else
         {
            ShapeArtifact targetArtifact = (ShapeArtifact) getArtifact(objectID);
            if (targetArtifact == null)
            {
               addArtifact(shapeArtifact);
            }
            else
            {
               updateArtifact(shapeArtifact);
            }
         }
      }

      repaint();
   }

   public void update(String objectID, PolygonArtifact polygonArtifact)
   {
      if (polygonArtifact == null)
      {
         removeArtifact(objectID);
      }
      else
      {
         if (polygonArtifact.getPoints().size() == 0)
         {
            removeArtifact(polygonArtifact.getID());
         }
         else
         {
            ShapeArtifact targetArtifact = (ShapeArtifact) getArtifact(objectID);
            if (targetArtifact == null)
            {
               addArtifact(polygonArtifact);
            }
            else
            {
               updateArtifact(polygonArtifact);
            }
         }
      }

      repaint();
   }

   public void setUpdateDelayInMillis(long timeInMillis)
   {
      updateDelayInMillis = timeInMillis;
   }

   public long getUpdateDelayInMillis()
   {
      return updateDelayInMillis;
   }

   public void setShowLabels(boolean showLabels)
   {
      this.showLabels = showLabels;
   }
   
   private class PlotterMouseListener extends MouseInputAdapter
   {
      @Override
      public void mouseClicked(MouseEvent e)
      {
         if (buttonPressed == MouseEvent.BUTTON1)
         {
            removeArtifact("path");
            removeArtifact("polygon");
            polygonArtifact = null;
         }
      }

      @Override
      public void mousePressed(MouseEvent e)
      {
         buttonPressed = e.getButton();

         if (buttonPressed == MouseEvent.BUTTON1)
         {
            selectedX = unConvertXCoordinate(e.getX());
            selectedY = unConvertYCoordinate(e.getY());
         }
         else if (buttonPressed == MouseEvent.BUTTON3)
         {
            selectedX = unConvertXCoordinate(e.getX());
            selectedY = unConvertYCoordinate(e.getY());
         }
         else if (buttonPressed == MouseEvent.BUTTON2)
         {
            dragStartY = e.getY();
         }

         // check for double-clicks
         if (e.getClickCount() > 1)
         {
            if (buttonPressed == MouseEvent.BUTTON1)
            {
               if (doubleClickListener != null)
               {
                  doubleClickListener.doubleClicked();
               }
            }
            else if (buttonPressed == MouseEvent.BUTTON3)
            {
               Coordinate offset = convertFromPixelsToMeters(new Coordinate(e.getX(), e.getY(), Coordinate.PIXEL));
               setOffsetX(offset.getX());
               setOffsetY(offset.getY());
               repaint();
            }
         }
      }

      @Override
      public void mouseDragged(MouseEvent e)
      {
         if (buttonPressed == MouseEvent.BUTTON1)
         {
            area1X = area1XTemp;
            area1Y = area1YTemp;
            area2X = unConvertXCoordinate(e.getX());
            area2Y = unConvertYCoordinate(e.getY());
         }
         else if (buttonPressed == MouseEvent.BUTTON3)
         {
            // do nothing
         }
         else if (buttonPressed == MouseEvent.BUTTON2)
         {
            int yDifferenceFromStartOfDrag = -(e.getY() - dragStartY);
            double scaledYChange = new Double(yDifferenceFromStartOfDrag * 0.5);
            if (getRange() < 10)
               scaledYChange = new Double(yDifferenceFromStartOfDrag * 0.01);
            double newRange = getRange() + scaledYChange;
            if (newRange < 0.1)
            {
               newRange = 0.1;
            }

            setRange(newRange);
            dragStartY = e.getY();
         }

         repaint();
      }

      @Override
      public void mouseReleased(MouseEvent e)
      {
      }
   }

   public void showInNewWindow()
   {
      JFrame frame = new JFrame("Plotter");
      frame.getContentPane().add(this, BorderLayout.CENTER);
      frame.pack();
      frame.setVisible(true);
   }

   public void addArtifactsChangedListener(ArtifactsChangedListener artifactsChangedListener)
   {
      this.artifactsChangedListeners.add(artifactsChangedListener);
   }

   public void notifyArtifactsChangedListeners()
   {
      for (ArtifactsChangedListener artifactsChangedListener : artifactsChangedListeners)
      {
         artifactsChangedListener.artifactsChanged(getArtifacts());
      }
   }

   public PlotterLegendPanel createPlotterLegendPanel()
   {
      PlotterLegendPanel ret = new PlotterLegendPanel();

      this.addArtifactsChangedListener(ret);

      return ret;
   }

   public JPanel createAndAttachPlotterLegendPanel()
   {
      JPanel ret = new JPanel();

      PlotterLegendPanel plotterLegendPanel = new PlotterLegendPanel();

      this.addArtifactsChangedListener(plotterLegendPanel);

      ret.setLayout(new BorderLayout());
      ret.add(this, "Center");
      ret.add(plotterLegendPanel, "South");

      return ret;
   }

   public static void main(String[] args)
   {
      // plotter
      Plotter p = new Plotter();
   
      JFrame f = new JFrame("Plotter Test");
      f.addWindowListener(new WindowAdapter()
      {
         @Override
         public void windowClosing(WindowEvent e)
         {
            System.exit(0);
         }
      });
   
      f.getContentPane().add(p, BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);
   }
}
