package us.ihmc.plotting;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
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
import javax.vecmath.Point2i;

import us.ihmc.plotting.shapes.LineArtifact;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.plotting.shapes.PolygonArtifact;
import us.ihmc.plotting.shapes.ShapeArtifact;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.tools.FormattingTools;

public class Plotter extends JPanel
{
   private static final boolean SHOW_LABELS_BY_DEFAULT = false;
   
   private static final long serialVersionUID = 3113130298799362369L;

   private final ArrayList<ArtifactsChangedListener> artifactsChangedListeners = new ArrayList<ArtifactsChangedListener>();

   // show selections
   private static final boolean SHOW_SELECTION = true;
   private static final Color TEXT_COLOR = Color.WHITE;

   private boolean drawHistory = false;
   private boolean showLabels = SHOW_LABELS_BY_DEFAULT;

   private long updateDelayInMillis = 0;
   private long lastUpdate = 0;

   private BufferedImage backgroundImage = null;

   private Dimension preferredSize = new Dimension(275, 275);
   private final HashMap<String, Artifact> artifacts = new HashMap<String, Artifact>();

   private Rectangle visibleRectangle = new Rectangle(275, 275);
   private PlotterTransform transform = new PlotterTransform();
   private Point2i centerOfPanelToMetersOriginInPixels = new Point2i();
   private double plotRange = 20.0;
   
   private PlotterPoint center = new PlotterPoint(transform);
   private PlotterPoint upperLeftCorner = new PlotterPoint(transform);
   private PlotterPoint lowerRightCorner = new PlotterPoint(transform);
   private PlotterPoint selected = new PlotterPoint(transform);
   private PlotterPoint selectionAreaStart = new PlotterPoint(transform);
   private PlotterPoint selectionAreaEnd = new PlotterPoint(transform);

   private DoubleClickListener doubleClickListener;

   private PolygonArtifact polygonArtifact;

   private boolean overrideAutomaticInterval = false;
   private double manualOverideInterval = 1.0;

   public Plotter()
   {
      // Initialize class variables
      setDoubleBuffered(true);

      // Set LayoutManager to null
      setLayout(null);

      // simpane
      Border raisedBevel = BorderFactory.createRaisedBevelBorder();
      Border loweredBevel = BorderFactory.createLoweredBevelBorder();
      Border compound = BorderFactory.createCompoundBorder(raisedBevel, loweredBevel);
      setBorder(compound);

      super.setBackground(new Color(180, 220, 240));

      PlotterMouseListener myListener = new PlotterMouseListener();
      addMouseListener(myListener);
      addMouseMotionListener(myListener);
      
      addComponentListener(new ComponentAdapter()
      {
         @Override
         public void componentShown(ComponentEvent e)
         {
            updateTransform();
         }
         
         @Override
         public void componentResized(ComponentEvent e)
         {
            updateTransform();
         }
      });
   }
   
   private void updateTransform()
   {
      // get current size and determine scaling factor
      computeVisibleRect(visibleRectangle);
      double smallerDimension = visibleRectangle.getHeight() <= visibleRectangle.getWidth() ? visibleRectangle.getHeight() : visibleRectangle.getWidth();
      transform.setMetersToPixels(smallerDimension / plotRange);
      transform.setPixelOriginToMetersOriginInPixels((int) (visibleRectangle.getWidth() / 2.0) + centerOfPanelToMetersOriginInPixels.getX(), (int) (visibleRectangle.getHeight() / 2.0) + centerOfPanelToMetersOriginInPixels.getY());
   }

   @Override
   public void paintComponent(Graphics graphics)
   {
      long currentTime = System.currentTimeMillis();
      if ((currentTime - lastUpdate) > updateDelayInMillis)
      {
         Graphics2D graphics2d = (Graphics2D) graphics;

         center.setInMeters(0.0, 0.0);
         upperLeftCorner.setInPixels(0, 0);
         lowerRightCorner.setInPixels((int) visibleRectangle.getWidth(), (int) visibleRectangle.getHeight());
         
         double headingOffset = 0.0;

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
                     artifact.draw(graphics2d, center.getInPixelsX(), center.getInPixelsY(), headingOffset, transform.getMetersToPixels());
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
            graphics2d.drawImage(backgroundImage, upperLeftCorner.getInPixelsX(), upperLeftCorner.getInPixelsY(), lowerRightCorner.getInPixelsX(), lowerRightCorner.getInPixelsY(), 0, 0, backgroundImage.getWidth(), backgroundImage.getHeight(), this);
         }
         else
         {
            // change grid line scale from 1m to 10cm ehn below 10m
            double gridSize = Math.pow(10, MathTools.orderOfMagnitude(transform.scalePixelsToMeters(25)) + 1);

            if (overrideAutomaticInterval)
            {
               gridSize = manualOverideInterval;
            }
            
            double overShoot = upperLeftCorner.getInMetersX() % gridSize;
            for (double gridX = upperLeftCorner.getInMetersX() - overShoot; gridX < lowerRightCorner.getInMetersX(); gridX += gridSize)
            {
               int nthGridLineFromOrigin = (int) Math.abs(gridX / gridSize);
               
               if (nthGridLineFromOrigin % 10 == 0)
               {
                  graphics2d.setColor(new Color(180, 190, 210));
               }
               else if (nthGridLineFromOrigin % 5 == 0)
               {
                  graphics2d.setColor(new Color(180, 210, 230));
               }
               else
               {
                  graphics2d.setColor(new Color(230, 240, 250));
               }

               // draw line
               int x = transform.transformMetersToPixelsX(gridX);
               graphics2d.drawLine(x, 0, x, (int) visibleRectangle.getHeight());
               
               if (showLabels)
               {
                  Color tempColor = graphics2d.getColor();
                  graphics2d.setColor(TEXT_COLOR);
                  graphics2d.drawString(FormattingTools.getFormattedToSignificantFigures(gridX, 2), x + 1, center.getInPixelsY() - 1);
                  graphics2d.setColor(tempColor);
               }
            }

            overShoot = lowerRightCorner.getInMetersY() % gridSize;
            for (double gridY = lowerRightCorner.getInMetersY() - overShoot; gridY < upperLeftCorner.getInMetersY(); gridY += gridSize)
            {
               int nthGridLineFromOrigin = (int) Math.abs(gridY / gridSize);
               
               if (nthGridLineFromOrigin % 10 == 0)
               {
                  graphics2d.setColor(new Color(180, 190, 210));
               }
               else if (nthGridLineFromOrigin % 5 == 0)
               {
                  graphics2d.setColor(new Color(180, 210, 230));
               }
               else
               {
                  graphics2d.setColor(new Color(230, 240, 250));
               }

               // draw line
               int y = transform.transformMetersToPixelsY(gridY);
               graphics2d.drawLine(0, y, (int) visibleRectangle.getWidth(), y);
               
               if (showLabels)
               {
                  Color tempColor = graphics2d.getColor();
                  graphics2d.setColor(TEXT_COLOR);
                  graphics2d.drawString(FormattingTools.getFormattedToSignificantFigures(gridY, 2), center.getInPixelsX() + 1, y - 1);
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
         graphics2d.drawLine(x0, 0, x0, (int) visibleRectangle.getHeight());
         graphics2d.drawLine(0, y0, (int) visibleRectangle.getWidth(), y0);

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
                           artifact.drawHistory(graphics2d, center.getInPixelsX(), center.getInPixelsY(), transform.getMetersToPixels());
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
                        artifact.draw(graphics2d, center.getInPixelsX(), center.getInPixelsY(), headingOffset, transform.getMetersToPixels()); // , _orientation);
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
            graphics2d.drawLine(selected.getInPixelsX() - xSize, selected.getInPixelsY() - xSize, selected.getInPixelsX() + xSize, selected.getInPixelsY() + xSize);
            graphics2d.drawLine(selected.getInPixelsX() - xSize, selected.getInPixelsY() + xSize, selected.getInPixelsX() + xSize, selected.getInPixelsY() - xSize);
         }

         // paint selected area
         if (SHOW_SELECTION)
         {
            graphics2d.setColor(Color.red);
            int Xmin, Xmax, Ymin, Ymax;
            if (selectionAreaStart.getInPixelsX() > selectionAreaEnd.getInPixelsX())
            {
               Xmax = selectionAreaStart.getInPixelsX();
               Xmin = selectionAreaEnd.getInPixelsX();
            }
            else
            {
               Xmax = selectionAreaEnd.getInPixelsX();
               Xmin = selectionAreaStart.getInPixelsX();
            }

            if (selectionAreaStart.getInPixelsY() > selectionAreaEnd.getInPixelsY())
            {
               Ymax = selectionAreaStart.getInPixelsY();
               Ymin = selectionAreaEnd.getInPixelsY();
            }
            else
            {
               Ymax = selectionAreaEnd.getInPixelsY();
               Ymin = selectionAreaStart.getInPixelsY();
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
      return selected.getInMetersX();
   }

   public double getSelectedY()
   {
      return selected.getInMetersY();
   }

   public double getArea1X()
   {
      return selectionAreaStart.getInMetersX();
   }

   public double getArea1Y()
   {
      return selectionAreaStart.getInMetersY();
   }

   public double getArea2X()
   {
      return selectionAreaEnd.getInMetersX();
   }

   public double getArea2Y()
   {
      return selectionAreaEnd.getInMetersY();
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

   private Coordinate convertFromMetersToPixels(Coordinate coordinate)
   {
      double x = coordinate.getX();
      double y = coordinate.getY();

      x = (double) center.getInPixelsX() + transform.scaleMetersToPixels(x);
      y = (double) center.getInPixelsY() - transform.scaleMetersToPixels(y);

      return new Coordinate(x, y, Coordinate.PIXEL);
   }

   public void setRangeLimit(int range, double origMapScale, double ullon, double ullat, double lrlon, double lrlat)
   {
      plotRange = range;

      updateTransform();
      repaint();
   }

   public void setRange(double range)
   {
      plotRange = range;
      updateTransform();
      repaint();
   }

   public void setDrawHistory(boolean drawHistory)
   {
      this.drawHistory = drawHistory;
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
      centerOfPanelToMetersOriginInPixels.setX(transform.scaleMetersToPixels(offsetX));
      updateTransform();
   }

   public void setOffsetY(double offsetY)
   {
      centerOfPanelToMetersOriginInPixels.setY(-transform.scaleMetersToPixels(offsetY));
      updateTransform();
   }

   public double getOffsetX()
   {
      return transform.scalePixelsToMeters(centerOfPanelToMetersOriginInPixels.getX());
   }

   public double getOffsetY()
   {
      return -transform.scalePixelsToMeters(centerOfPanelToMetersOriginInPixels.getY());
   }

   @Override
   public Dimension getPreferredSize()
   {
      return preferredSize;
   }

   public void setPreferredSize(int width, int height)
   {
      preferredSize.setSize(width, height);
      updateTransform();
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
      private int buttonPressed;
      private int dragStartY;
      
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
            selected.setInPixels(e.getX(), e.getY());
            selectionAreaStart.setInPixels(e.getX(), e.getY());
         }
         else if (buttonPressed == MouseEvent.BUTTON3)
         {
            selected.setInPixels(e.getX(), e.getY());
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
               int deltaXInPixelsFromOrigin = e.getX() - transform.getPixelOriginToMetersOriginInPixels().getX();
               int deltaYInPixelsFromOrigin = e.getY() - transform.getPixelOriginToMetersOriginInPixels().getY();
               
               centerOfPanelToMetersOriginInPixels.setX(centerOfPanelToMetersOriginInPixels.getX() + deltaXInPixelsFromOrigin);
               centerOfPanelToMetersOriginInPixels.setY(centerOfPanelToMetersOriginInPixels.getY() + deltaYInPixelsFromOrigin);
               
               updateTransform();
               repaint();
            }
         }
      }

      @Override
      public void mouseDragged(MouseEvent e)
      {
         if (buttonPressed == MouseEvent.BUTTON1)
         {
            selectionAreaEnd.setInPixels(e.getX(), e.getY());
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
      showInNewWindow("Plotter");
   }
   
   public void showInNewWindow(String title)
   {
      JFrame frame = new JFrame(title);
      frame.getContentPane().add(this, BorderLayout.CENTER);
      frame.pack();
      frame.setVisible(true);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
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
