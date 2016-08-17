package us.ihmc.plotting.plotter2d;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.MouseAdapter;
import java.util.ArrayList;
import java.util.HashMap;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.ArtifactsChangedListener;
import us.ihmc.plotting.plotter2d.frames.MetersReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PixelsReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PlotterFrameSpace;
import us.ihmc.plotting.plotter2d.frames.PlotterSpaceConverter;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.io.printing.PrintTools;

/**
 * TODO Make plotter not extend JPanel.
 * TODO Factor out artifacts.
 * TODO Fix Artifact interface.
 */
@SuppressWarnings("serial")
public class Plotter2d extends JPanel
{
   private static final boolean SHOW_LABELS_BY_DEFAULT = true;
   private static final boolean SHOW_SELECTION_BY_DEFAULT = false;
   private static final boolean SHOW_HISTORY_BY_DEFAULT = false;
   
   private boolean showLabels = SHOW_LABELS_BY_DEFAULT;
   private boolean showSelection = SHOW_SELECTION_BY_DEFAULT;
   private boolean showHistory = SHOW_HISTORY_BY_DEFAULT;
   
   private final PlotterMouseAdapter mouseAdapter = new PlotterMouseAdapter();
   private final PlotterComponentAdapter componentAdapter = new PlotterComponentAdapter();
   
   private final Vector2d metersToPixels = new Vector2d(50.0, 50.0);
   private final Rectangle visibleRectangle = new Rectangle();
   private final Dimension preferredSize = new Dimension(275, 275);
   private final Dimension gridSizePixels = new Dimension();
   
   private final PlotterSpaceConverter spaceConverter;
   private final PixelsReferenceFrame pixelsFrame;
   private final PixelsReferenceFrame screenFrame;
   private final MetersReferenceFrame metersFrame;
   
   private final PlotterPoint2d screenPosition;
   private final PlotterPoint2d upperLeftCorner;
   private final PlotterPoint2d lowerRightCorner;
   private final PlotterPoint2d centerScreen;
   private final PlotterPoint2d origin;
   private final PlotterPoint2d drawGuy;
   
   // Artifact stuff
   private final ArrayList<ArtifactsChangedListener> artifactsChangedListeners = new ArrayList<ArtifactsChangedListener>();
   private final HashMap<String, Artifact> artifacts = new HashMap<String, Artifact>();
   
   public Plotter2d()
   {
      spaceConverter = new PlotterSpaceConverter()
      {
         private Vector2d scaleVector = new Vector2d();
         
         @Override
         public Vector2d getConversionToSpace(PlotterFrameSpace plotterFrameType)
         {
            if (plotterFrameType == PlotterFrameSpace.METERS)
            {
               scaleVector.set(1.0 / metersToPixels.getX(), 1.0 / metersToPixels.getY());
            }
            else
            {
               scaleVector.set(metersToPixels.getX(), metersToPixels.getY());
            }
            return scaleVector;
         }
      };
      pixelsFrame = new PixelsReferenceFrame("pixelsFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      screenFrame = new PixelsReferenceFrame("screenFrame", pixelsFrame, spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(0.0, Math.PI, Math.PI);
            transformToParent.setTranslation(screenPosition.getX(), screenPosition.getY(), 0.0);
         }
      };
      metersFrame = new MetersReferenceFrame("metersFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      
      screenPosition = new PlotterPoint2d(pixelsFrame);
      upperLeftCorner = new PlotterPoint2d(screenFrame);
      lowerRightCorner = new PlotterPoint2d(screenFrame);
      centerScreen = new PlotterPoint2d(screenFrame);
      origin = new PlotterPoint2d(metersFrame);
      drawGuy = new PlotterPoint2d(screenFrame);
      
      screenPosition.set(-200.0, 200.0);
      
      updateFrames();
      
      setBorder(BorderFactory.createCompoundBorder(BorderFactory.createRaisedBevelBorder(), BorderFactory.createLoweredBevelBorder()));
      setBackground(PlotterColors.BACKGROUND);
      
      addMouseListener(mouseAdapter);
      addMouseMotionListener(mouseAdapter);
      addComponentListener(componentAdapter);
   }
   
   private void updateFrames()
   {
      computeVisibleRect(visibleRectangle);
      
      pixelsFrame.update();
      screenFrame.update();
      metersFrame.update();
      
      upperLeftCorner.changeFrame(screenFrame);
      lowerRightCorner.changeFrame(screenFrame);
      centerScreen.changeFrame(screenFrame);
      origin.changeFrame(metersFrame);
      
      upperLeftCorner.set(0.0, 0.0);
      lowerRightCorner.set(visibleRectangle.getWidth(), visibleRectangle.getHeight());
      centerScreen.set(lowerRightCorner.getX() / 2.0, lowerRightCorner.getY() / 2.0);
      origin.set(0.0, 0.0);
   }
   
   @Override
   protected void paintComponent(Graphics graphics)
   {
      paintComponent((Graphics2D) graphics);
   }
   
   private void paintComponent(final Graphics2D graphics2d)
   {
      updateFrames();
      
      super.paintComponent(graphics2d);
      
      origin.changeFrame(screenFrame);
      forAllArtifacts(86, new ArtifactIterator()
      {
         @Override
         public void drawArtifact(Artifact artifact)
         {
            artifact.draw(graphics2d, (int) Math.round(origin.getX()), (int) Math.round(origin.getY()), 0.0, metersToPixels);
         }
      });
      
      // change grid line scale from 1m to 10cm ehn below 10m
//      gridSizePixels.setSize(Math.pow(10, MathTools.orderOfMagnitude(25.0 * metersToPixels.getX()) + 1),
//                       Math.pow(10, MathTools.orderOfMagnitude(25.0 * metersToPixels.getY()) + 1));
      gridSizePixels.setSize(50,
                             50);
      
      upperLeftCorner.changeFrame(pixelsFrame);
      lowerRightCorner.changeFrame(pixelsFrame);
      
      double overShoot = upperLeftCorner.getX() % gridSizePixels.getWidth();
      for (double gridX = upperLeftCorner.getX() - overShoot; gridX < lowerRightCorner.getX(); gridX += gridSizePixels.getWidth())
      {
         drawGuy.changeFrame(pixelsFrame);
         drawGuy.set(upperLeftCorner);
         drawGuy.setX(gridX);
         
         int nthGridLineFromOrigin = (int) (Math.abs(drawGuy.getX()) / gridSizePixels.getWidth());
         applyColorForGridline(graphics2d, nthGridLineFromOrigin);

         drawGuy.changeFrame(screenFrame);
         graphics2d.drawLine((int) Math.round(drawGuy.getX()), 0, (int) Math.round(drawGuy.getX()), (int) visibleRectangle.getHeight());
         
         if (showLabels)
         {
            Color tempColor = graphics2d.getColor();
            graphics2d.setColor(PlotterColors.LABEL_COLOR);
            drawGuy.changeFrame(metersFrame);
            String labelString = FormattingTools.getFormattedToSignificantFigures(drawGuy.getX(), 2);
            drawGuy.changeFrame(screenFrame);
            graphics2d.drawString(labelString, (int) Math.round(drawGuy.getX()) + 1, (int) Math.round(origin.getY()) - 1);
            graphics2d.setColor(tempColor);
         }
      }
      
      overShoot = lowerRightCorner.getY() % gridSizePixels.getHeight();
      for (double gridY = lowerRightCorner.getY() - overShoot; gridY < upperLeftCorner.getY(); gridY += gridSizePixels.getHeight())
      {
         drawGuy.changeFrame(pixelsFrame);
         drawGuy.set(upperLeftCorner);
         drawGuy.setY(gridY);
         
         int nthGridLineFromOrigin = (int) (Math.abs(drawGuy.getY()) / gridSizePixels.getWidth());
         applyColorForGridline(graphics2d, nthGridLineFromOrigin);

         drawGuy.changeFrame(screenFrame);
         graphics2d.drawLine(0, (int) Math.round(drawGuy.getY()), (int) visibleRectangle.getWidth(), (int) Math.round(drawGuy.getY()));
         
         if (showLabels)
         {
            Color tempColor = graphics2d.getColor();
            graphics2d.setColor(PlotterColors.LABEL_COLOR);
            drawGuy.changeFrame(metersFrame);
            String labelString = FormattingTools.getFormattedToSignificantFigures(drawGuy.getY(), 2);
            drawGuy.changeFrame(screenFrame);
            graphics2d.drawString(labelString, (int) Math.round(origin.getX()) + 1, (int) Math.round(drawGuy.getY()) - 1);
            graphics2d.setColor(tempColor);
         }
      }
      
      // paint grid centerline
      graphics2d.setColor(PlotterColors.GRID_AXIS);
      graphics2d.drawLine((int) Math.round(origin.getX()), 0, (int) Math.round(origin.getX()), (int) visibleRectangle.getHeight());
      graphics2d.drawLine(0, (int) Math.round(origin.getY()), (int) visibleRectangle.getWidth(), (int) Math.round(origin.getY()));
      
      for (int artifactLevel = 0; artifactLevel < 5; artifactLevel++)
      {
         forAllArtifacts(artifactLevel, new ArtifactIterator()
         {
            @Override
            public void drawArtifact(Artifact artifact)
            {
               if (showHistory && artifact.getDrawHistory())
               {
                  artifact.drawHistory(graphics2d, (int) Math.round(origin.getX()), (int) Math.round(origin.getY()), 0.0, metersToPixels);
               }
            }
         });
      }
      
      for (int artifactLevel = 0; artifactLevel < 5; artifactLevel++)
      {
         forAllArtifacts(artifactLevel, new ArtifactIterator()
         {
            @Override
            public void drawArtifact(Artifact artifact)
            {
               artifact.draw(graphics2d, (int) Math.round(origin.getX()), (int) Math.round(origin.getY()), 0.0, metersToPixels);
            }
         });
      }
   }

   private void applyColorForGridline(final Graphics2D graphics2d, int nthGridLineFromOrigin)
   {
      if (nthGridLineFromOrigin % 10 == 0)
      {
         graphics2d.setColor(PlotterColors.GRID_EVERY_10);
      }
      else if (nthGridLineFromOrigin % 5 == 0)
      {
         graphics2d.setColor(PlotterColors.GRID_EVERY_5);
      }
      else
      {
         graphics2d.setColor(PlotterColors.GRID_EVERY_1);
      }
   }
   
   private void forAllArtifacts(int level, ArtifactIterator artifactIterator)
   {
      synchronized (artifacts)
      {
         for (Artifact artifact : artifacts.values())
         {
            if (artifact != null)
            {
               if (artifact.getLevel() == level)
               {
                  artifactIterator.drawArtifact(artifact);
               }
            }
            else
            {
               PrintTools.error("Plotter: one of the artifacts you added was null");
            }
         }
      }
   }
   
   private interface ArtifactIterator
   {
      public void drawArtifact(Artifact artifact); 
   }
   
   private class PlotterMouseAdapter extends MouseAdapter
   {
      
   }
   
   private class PlotterComponentAdapter extends ComponentAdapter
   {
      @Override
      public void componentShown(ComponentEvent componentEvent)
      {
         updateFrames();
      }
      
      @Override
      public void componentResized(ComponentEvent componentEvent)
      {
         updateFrames();
      }
   }
   
   @Override
   public Dimension getPreferredSize()
   {
      return preferredSize;
   }
   
   @Override
   @Deprecated
   public void setPreferredSize(Dimension preferredSize)
   {
      this.preferredSize.setSize(preferredSize);
   }
   
   public void setPreferredSize(int width, int height)
   {
      preferredSize.setSize(width, height);
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
}
