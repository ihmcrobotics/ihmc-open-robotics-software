package us.ihmc.plotting;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.Stroke;
import java.awt.Toolkit;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.graphicsDescription.plotting.PlotterColors;
import us.ihmc.graphicsDescription.plotting.PlotterPoint2d;
import us.ihmc.graphicsDescription.plotting.PlotterVector2d;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.plotting.artifact.ArtifactsChangedListener;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.PointListArtifact;
import us.ihmc.graphicsDescription.plotting.frames.MetersReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PixelsReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PlotterFrameSpace;
import us.ihmc.graphicsDescription.plotting.frames.PlotterSpaceConverter;
import us.ihmc.graphicsDescription.yoGraphics.plotting.PlotterInterface;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.io.printing.PrintTools;

/**
 * TODO Deprecate archaic methods
 * TODO Factor out artifacts.
 * TODO Fix color field in Artifact
 * TODO Fix zoom in for vector
 */
@SuppressWarnings("serial")
public class Plotter implements PlotterInterface
{
   private static final boolean SHOW_LABELS_BY_DEFAULT = true;
   private static final boolean SHOW_SELECTION_BY_DEFAULT = false;
   private static final boolean SHOW_HISTORY_BY_DEFAULT = false;
   private static final boolean ENABLE_XY_ZOOM_BY_DEFAULT = false;
   private static final boolean ENABLE_ROTATION_BY_DEFAULT = true;
   
   private boolean showLabels = SHOW_LABELS_BY_DEFAULT;
   private boolean showSelection = SHOW_SELECTION_BY_DEFAULT;
   private boolean showHistory = SHOW_HISTORY_BY_DEFAULT;
   private boolean xyZoomEnabled = ENABLE_XY_ZOOM_BY_DEFAULT;
   private boolean rotationEnabled = ENABLE_ROTATION_BY_DEFAULT;
   
   private final PlotterColors plotterColors;
   
   private final JPanel panel;
   
   private final PlotterMouseAdapter mouseAdapter;
   private final PlotterComponentAdapter componentAdapter;
   
   private final Plotter2DAdapter plotter2dAdapter;
   private final Graphics2DAdapter graphics2dAdapter;
   private final Stroke normalStroke;
   private final Stroke dashedStroke;
   
   private final Vector2D metersToPixels = new Vector2D(50.0, 50.0);
   private final Rectangle visibleRectangle = new Rectangle();
   private final Dimension preferredSize = new Dimension(500, 500);
   private final PlotterVector2d gridSize;
   private BufferedImage backgroundImage = null;
   
   private final PlotterSpaceConverter spaceConverter;
   private final PixelsReferenceFrame pixelsFrame;
   private final PixelsReferenceFrame screenFrame;
   private final MetersReferenceFrame metersFrame;
   
   private double screenRotation = 0.0;
   private final Vector3D tempTranslation = new Vector3D();
   private final Line2d tempGridLine = new Line2d();
   private final PlotterPoint2d screenPosition;
   private final PlotterPoint2d upperLeftCorner;
   private final PlotterPoint2d lowerRightCorner;
   private final PlotterPoint2d focusPoint;
   private final PlotterPoint2d origin;
   private final PlotterPoint2d gridLinePencil;
   private final PlotterPoint2d selected;
   private final PlotterPoint2d selectionAreaStart;
   private final PlotterPoint2d selectionAreaEnd;
   private final PlotterPoint2d imageFirstCorner;
   private final PlotterPoint2d imageSecondCorner;
   private final PlotterPoint2d labelPosition;
   
   // Artifact stuff
   private final ArrayList<ArtifactsChangedListener> artifactsChangedListeners = new ArrayList<ArtifactsChangedListener>();
   private final HashMap<String, Artifact> artifacts = new HashMap<String, Artifact>();
   
   public Plotter()
   {
      this(PlotterColors.simulationConstructionSetStyle(), false);
   }
   
   public Plotter(PlotterColors plotterColors, boolean highQuality)
   {
      this.plotterColors = plotterColors;
      
      panel = new JPanel()
      {
         @Override
         protected void paintComponent(Graphics graphics)
         {
            plotter2dAdapter.setGraphics2d((Graphics2D) graphics);
            updateFrames();
            super.paintComponent(graphics);
            Plotter.this.paintComponent(plotter2dAdapter);
         }
         
         @Override
         public Dimension getPreferredSize()
         {
            return Plotter.this.preferredSize;
         }
         
         @Override
         public void setPreferredSize(Dimension preferredSize)
         {
            Plotter.this.preferredSize.setSize(preferredSize);
         }
      };
      
      spaceConverter = new PlotterSpaceConverter()
      {
         private Vector2D scaleVector = new Vector2D();
         
         @Override
         public Vector2D getConversionToSpace(PlotterFrameSpace plotterFrameType)
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
            screenPosition.changeFrame(pixelsFrame);
            transformToParent.setIdentity();
            tempTranslation.set(screenPosition.getX() + getPlotterWidthPixels() / 2.0, screenPosition.getY() - getPlotterHeightPixels() / 2.0, 0.0);
            transformToParent.transform(tempTranslation);
            transformToParent.setTranslation(tempTranslation);
            transformToParent.appendYawRotation(screenRotation);
            tempTranslation.set(-getPlotterWidthPixels() / 2.0, getPlotterHeightPixels() / 2.0, 0.0);
            transformToParent.transform(tempTranslation);
            transformToParent.setTranslation(tempTranslation);
            transformToParent.appendPitchRotation(Math.PI);
            transformToParent.appendYawRotation(Math.PI);
         }
      };
      metersFrame = new MetersReferenceFrame("metersFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      
      gridSize = new PlotterVector2d(pixelsFrame);
      screenPosition = new PlotterPoint2d(pixelsFrame);
      upperLeftCorner = new PlotterPoint2d(screenFrame);
      lowerRightCorner = new PlotterPoint2d(screenFrame);
      origin = new PlotterPoint2d(metersFrame);
      focusPoint = new PlotterPoint2d(screenFrame);
      gridLinePencil = new PlotterPoint2d(screenFrame);
      selected = new PlotterPoint2d(screenFrame);
      selectionAreaStart = new PlotterPoint2d(screenFrame);
      selectionAreaEnd = new PlotterPoint2d(screenFrame);
      imageFirstCorner = new PlotterPoint2d(screenFrame);
      imageSecondCorner = new PlotterPoint2d(screenFrame);
      labelPosition = new PlotterPoint2d(screenFrame);
      
      screenPosition.set(-getPlotterWidthPixels() / 2.0, getPlotterHeightPixels() / 2.0);
      focusPoint.setIncludingFrame(metersFrame, 0.0, 0.0);
      
      updateFrames();
      
      normalStroke = new BasicStroke(1.0f);
      if (highQuality)
      {
         dashedStroke = new BasicStroke(1.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 1.0f, new float[] {3.0f}, 0.0f);
      }
      else
      {
         dashedStroke = new BasicStroke(1.0f);
      }
      plotter2dAdapter = new Plotter2DAdapter(metersFrame, screenFrame, pixelsFrame);
      graphics2dAdapter = new Graphics2DAdapter(plotter2dAdapter);
      
      panel.setBorder(BorderFactory.createCompoundBorder(BorderFactory.createRaisedBevelBorder(), BorderFactory.createLoweredBevelBorder()));
      panel.setBackground(plotterColors.getBackgroundColor());
      
      mouseAdapter = new PlotterMouseAdapter();
      componentAdapter = new PlotterComponentAdapter();
      
      panel.addMouseListener(mouseAdapter);
      panel.addMouseMotionListener(mouseAdapter);
      panel.addComponentListener(componentAdapter);
   }
   
   private void updateFrames()
   {
      panel.computeVisibleRect(visibleRectangle);
      
      pixelsFrame.update();
      screenFrame.update();
      metersFrame.update();
      
      upperLeftCorner.setIncludingFrame(screenFrame, 0.0, 0.0);
      lowerRightCorner.setIncludingFrame(screenFrame, getPlotterWidthPixels(), getPlotterHeightPixels());
      origin.setIncludingFrame(metersFrame, 0.0, 0.0);
   }
   
   public void setScale(double pixelsPerMeterX, double pixelsPerMeterY)
   {
      focusPoint.changeFrame(metersFrame);
      metersToPixels.set(pixelsPerMeterX, pixelsPerMeterY);
      
      centerOnFocusPoint();
   }
   
   private void centerOnFocusPoint()
   {
      focusPoint.changeFrame(pixelsFrame);
      
      screenPosition.changeFrame(pixelsFrame);
      screenPosition.set(focusPoint);
      screenPosition.add(-getPlotterWidthPixels() / 2.0, getPlotterHeightPixels() / 2.0);

      updateFrames();
   }

   private boolean isInitialized()
   {
      return visibleRectangle.getWidth() > 0.0;
   }
   
   private double getPlotterWidthPixels()
   {
      return isInitialized() ? visibleRectangle.getWidth() : preferredSize.getWidth();
   }
   
   private double getPlotterHeightPixels()
   {
      return isInitialized() ? visibleRectangle.getHeight() : preferredSize.getHeight();
   }

   private void paintComponent(final Plotter2DAdapter graphics2d)
   {
      origin.changeFrame(screenFrame);
      forAllArtifacts(86, new ArtifactIterator()
      {
         @Override
         public void drawArtifact(Artifact artifact)
         {
            if (artifact.isVisible())
            {
               artifact.draw(graphics2dAdapter);
            }
         }
      });
      
      if (backgroundImage != null)
      {
         imageFirstCorner.setToZero();
         imageSecondCorner.set(backgroundImage.getWidth(), backgroundImage.getHeight());
         graphics2d.drawImage(backgroundImage, upperLeftCorner, lowerRightCorner, imageFirstCorner, imageSecondCorner, panel);
      }
      else
      {
         // change grid line scale from 1m to 10cm ehn below 10m
         gridSize.set(calculateGridSizePixels(metersToPixels.getX()), calculateGridSizePixels(metersToPixels.getY()));
         
         upperLeftCorner.changeFrame(pixelsFrame);
         lowerRightCorner.changeFrame(pixelsFrame);
         
         focusPoint.changeFrame(pixelsFrame);
         double gridStart = (Math.round(focusPoint.getX() / gridSize.getX()) - 20.0) * gridSize.getX();
         double gridEnd = (Math.round(focusPoint.getX() / gridSize.getX()) + 20.0) * gridSize.getX();
         
         for (double gridX = gridStart; gridX < gridEnd; gridX += gridSize.getX())
         {
            gridLinePencil.setIncludingFrame(pixelsFrame, gridX, 0.0);
            
            gridLinePencil.changeFrame(metersFrame);
            gridSize.changeFrame(metersFrame);
            int nthGridLineFromOrigin = (int) (Math.abs(gridLinePencil.getX()) / gridSize.getX());
            if (MathTools.epsilonEquals(Math.abs(gridLinePencil.getX()) % gridSize.getX(), gridSize.getX(), 1e-7))
            {
               nthGridLineFromOrigin++;
            }
            applyParametersForGridline(graphics2d, nthGridLineFromOrigin);
   
            gridLinePencil.changeFrame(pixelsFrame);
            gridSize.changeFrame(pixelsFrame);
            tempGridLine.set(gridLinePencil.getX(), gridLinePencil.getY(), 0.0, 1.0);
            graphics2d.drawLine(pixelsFrame, tempGridLine);
            
            if (showLabels)
            {
               graphics2d.setColor(plotterColors.getLabelColor());
               gridLinePencil.changeFrame(metersFrame);
               String labelString = FormattingTools.getFormattedToSignificantFigures(gridLinePencil.getX(), 2);
               gridLinePencil.changeFrame(pixelsFrame);
               origin.changeFrame(pixelsFrame);
               if (MathTools.epsilonEquals(screenRotation, 0.0, 1e-3) && origin.getY() > upperLeftCorner.getY() - 14)
               {
                  gridLinePencil.setY(upperLeftCorner.getY() - 14);
               }
               else if (MathTools.epsilonEquals(screenRotation, 0.0, 1e-3) && origin.getY() < lowerRightCorner.getY())
               {
                  gridLinePencil.setY(lowerRightCorner.getY() + 6);
               }
               else
               {
                  gridLinePencil.setY(origin.getY() + 1);
               }
               labelPosition.setIncludingFrame(gridLinePencil);
               labelPosition.add(1.0, 0.0);
               graphics2d.drawString(labelString, labelPosition);
            }
         }
         
         gridStart = (Math.round(focusPoint.getY() / gridSize.getY()) - 20.0) * gridSize.getY();
         gridEnd = (Math.round(focusPoint.getY() / gridSize.getY()) + 20.0) * gridSize.getY();
         
         for (double gridY = gridStart; gridY < gridEnd; gridY += gridSize.getY())
         {
            gridLinePencil.setIncludingFrame(pixelsFrame, 0.0, gridY);
            
            gridLinePencil.changeFrame(metersFrame);
            gridSize.changeFrame(metersFrame);
            int nthGridLineFromOrigin = (int) (Math.abs(gridLinePencil.getY()) / gridSize.getY());
            if (MathTools.epsilonEquals(Math.abs(gridLinePencil.getY()) % gridSize.getY(), gridSize.getY(), 1e-5))
            {
               nthGridLineFromOrigin++;
            }
            applyParametersForGridline(graphics2d, nthGridLineFromOrigin);
   
            gridLinePencil.changeFrame(pixelsFrame);
            gridSize.changeFrame(pixelsFrame);
            tempGridLine.set(gridLinePencil.getX(), gridLinePencil.getY(), 1.0, 0.0);
            graphics2d.drawLine(pixelsFrame, tempGridLine);
            
            if (showLabels)
            {
               graphics2d.setColor(plotterColors.getLabelColor());
               gridLinePencil.changeFrame(metersFrame);
               String labelString = FormattingTools.getFormattedToSignificantFigures(gridLinePencil.getY(), 2);
               gridLinePencil.changeFrame(pixelsFrame);
               origin.changeFrame(pixelsFrame);
               if (MathTools.epsilonEquals(screenRotation, 0.0, 1e-3) && origin.getX() > lowerRightCorner.getX() - 30)
               {
                  gridLinePencil.setX(lowerRightCorner.getX() - 30);
               }
               else if (MathTools.epsilonEquals(screenRotation, 0.0, 1e-3) && origin.getX() < upperLeftCorner.getX())
               {
                  gridLinePencil.setX(upperLeftCorner.getX() + 6);
               }
               else
               {
                  gridLinePencil.setX(origin.getX() + 1);
               }
               labelPosition.setIncludingFrame(gridLinePencil);
               labelPosition.add(0.0, 1.0);
               graphics2d.drawString(labelString, labelPosition);
            }
         }
      }
      
      // paint grid centerline
      origin.changeFrame(pixelsFrame);
      graphics2d.setStroke(normalStroke);
      graphics2d.setColor(plotterColors.getGridAxisColor());
      tempGridLine.set(origin.getX(), origin.getY(), 1.0, 0.0);
      graphics2d.drawLine(pixelsFrame, tempGridLine);
      tempGridLine.set(origin.getX(), origin.getY(), 0.0, 1.0);
      graphics2d.drawLine(pixelsFrame, tempGridLine);
      
      for (int artifactLevel = 0; artifactLevel < 5; artifactLevel++)
      {
         forAllArtifacts(artifactLevel, new ArtifactIterator()
         {
            @Override
            public void drawArtifact(Artifact artifact)
            {
               if (showHistory && artifact.getDrawHistory() && artifact.isVisible())
               {
                  artifact.drawHistory(graphics2dAdapter);
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
               if (artifact.isVisible())
               {
                  artifact.draw(graphics2dAdapter);
               }
            }
         });
      }
      
      // paint selected destination
      if (showSelection)
      {
         selected.changeFrame(screenFrame);
         graphics2d.setColor(plotterColors.getSelectionColor());
         double crossSize = 8.0;
         graphics2d.drawLineSegment(screenFrame, selected.getX() - crossSize,
                                                 selected.getY() - crossSize,
                                                 selected.getX() + crossSize,
                                                 selected.getY() + crossSize);
         graphics2d.drawLineSegment(screenFrame, selected.getX() - crossSize,
                                                 selected.getY() + crossSize,
                                                 selected.getX() + crossSize,
                                                 selected.getY() - crossSize);
      }

      // paint selected area
      if (showSelection)
      {
         graphics2d.setColor(plotterColors.getSelectionColor());
         double Xmin, Xmax, Ymin, Ymax;
         if (selectionAreaStart.getX() > selectionAreaEnd.getX())
         {
            Xmax = selectionAreaStart.getX();
            Xmin = selectionAreaEnd.getX();
         }
         else
         {
            Xmax = selectionAreaEnd.getX();
            Xmin = selectionAreaStart.getX();
         }

         if (selectionAreaStart.getY() > selectionAreaEnd.getY())
         {
            Ymax = selectionAreaStart.getY();
            Ymin = selectionAreaEnd.getY();
         }
         else
         {
            Ymax = selectionAreaEnd.getY();
            Ymin = selectionAreaStart.getY();
         }

         graphics2d.drawRectangle(screenFrame, Xmin, Ymin, Xmax - Xmin, Ymax - Ymin);
      }
   }

   private double calculateGridSizePixels(double pixelsPerMeter)
   {
      double medianGridWidthInPixels = Toolkit.getDefaultToolkit().getScreenResolution();
      double desiredMeters = medianGridWidthInPixels / pixelsPerMeter;
      double decimalPlace = Math.log10(desiredMeters);
      double orderOfMagnitude = Math.floor(decimalPlace);
      double nextOrderOfMagnitude = Math.pow(10, orderOfMagnitude + 1);
      double percentageToNextOrderOfMagnitude = desiredMeters / nextOrderOfMagnitude;
      
      double remainder = percentageToNextOrderOfMagnitude % 0.5;
      double roundToNearestPoint5 = remainder >= 0.25 ? percentageToNextOrderOfMagnitude + (0.5 - remainder) : percentageToNextOrderOfMagnitude - remainder;
      
      double gridSizeMeters;
      if (roundToNearestPoint5 > 0.0)
      {
         gridSizeMeters = nextOrderOfMagnitude * roundToNearestPoint5;
      }
      else
      {
         gridSizeMeters = Math.pow(10, orderOfMagnitude);
      }
      double gridSizePixels = gridSizeMeters * pixelsPerMeter;
      
      return gridSizePixels;
   }

   private void applyParametersForGridline(final Plotter2DAdapter graphics2d, int nthGridLineFromOrigin)
   {
      if (nthGridLineFromOrigin % 10 == 0)
      {
         graphics2d.setStroke(normalStroke);
         graphics2d.setColor(plotterColors.getGridEveryTenColor());
      }
      else if (nthGridLineFromOrigin % 5 == 0)
      {
         graphics2d.setStroke(dashedStroke);
         graphics2d.setColor(plotterColors.getGridEveryFiveColor());
      }
      else
      {
         graphics2d.setStroke(dashedStroke);
         graphics2d.setColor(plotterColors.getGridEveryOneColor());
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
      private int buttonPressed;
      private PlotterPoint2d middleMouseDragStart = new PlotterPoint2d(screenFrame);
      private PlotterPoint2d middleMouseDragEnd = new PlotterPoint2d(screenFrame);
      private PlotterPoint2d rightMouseDragStart = new PlotterPoint2d(screenFrame);
      private PlotterPoint2d rightMouseDragEnd = new PlotterPoint2d(screenFrame);

      @Override
      public void mousePressed(MouseEvent mouseEvent)
      {
         buttonPressed = mouseEvent.getButton();

         if (buttonPressed == MouseEvent.BUTTON1)
         {
            selected.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
            selectionAreaStart.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
         }
         else if (buttonPressed == MouseEvent.BUTTON2)
         {
            middleMouseDragStart.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
         }
         else if (buttonPressed == MouseEvent.BUTTON3)
         {
            rightMouseDragStart.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
            
            if (mouseEvent.getClickCount() > 1)
            {
               focusPoint.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
               centerOnFocusPoint();
               panel.repaint();
            }
         }
      }

      @Override
      public void mouseDragged(MouseEvent mouseEvent)
      {
         if (buttonPressed == MouseEvent.BUTTON1)
         {
            selectionAreaEnd.changeFrame(screenFrame);
            selectionAreaEnd.set(mouseEvent.getX(), mouseEvent.getY());
            
            panel.repaint();
         }
         else if (buttonPressed == MouseEvent.BUTTON2)
         {
            middleMouseDragEnd.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
            
            double deltaDragY = middleMouseDragStart.getY() - middleMouseDragEnd.getY();
            double deltaDragX = middleMouseDragStart.getX() - middleMouseDragEnd.getX();
            
            double scaledXChange;
            if (xyZoomEnabled && mouseEvent.isControlDown())
            {
               scaledXChange = 1.0 + (deltaDragX * 0.005);
            }
            else
            {
               scaledXChange = 1.0 + (deltaDragY * 0.005);
            }
            double scaledYChange = 1.0 + (deltaDragY * 0.005);
            
            focusPoint.setIncludingFrame(screenFrame, getPlotterWidthPixels() / 2.0, getPlotterHeightPixels() / 2.0);
            
            setScale(metersToPixels.getX() * scaledXChange, metersToPixels.getY() * scaledYChange);
            
            middleMouseDragStart.set(middleMouseDragEnd);
            
            panel.repaint();
         }
         else if (buttonPressed == MouseEvent.BUTTON3)
         {
            rightMouseDragEnd.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
            
            focusPoint.changeFrame(screenFrame);
            
            rightMouseDragEnd.sub(rightMouseDragStart);

            if (rotationEnabled && mouseEvent.isAltDown())
            {
               screenRotation += 0.01 * (double) rightMouseDragEnd.getX();
            }
            else
            {
               rightMouseDragEnd.negate();
               focusPoint.add(rightMouseDragEnd);
            }
            
            centerOnFocusPoint();
            panel.repaint();
            
            rightMouseDragStart.setIncludingFrame(screenFrame, mouseEvent.getX(), mouseEvent.getY());
         }
      }
   }
   
   private class PlotterComponentAdapter extends ComponentAdapter
   {
      @Override
      public void componentResized(ComponentEvent componentEvent)
      {
         centerOnFocusPoint();
         panel.repaint();
      }
   }
   
   public Dimension getPreferredSize()
   {
      return preferredSize;
   }
   
   public void setPreferredSize(int width, int height)
   {
      preferredSize.setSize(width, height);
   }

   public void setFocusPointY(double focusPointY)
   {
      focusPoint.changeFrame(metersFrame);
      focusPoint.setY(focusPointY);
      
      centerOnFocusPoint();
   }
   
   public void setFocusPointX(double focusPointX)
   {
      focusPoint.changeFrame(metersFrame);
      focusPoint.setX(focusPointX);
      
      centerOnFocusPoint();
   }

   public void setFocusOrientationYaw(double focusOrientationYaw)
   {
      screenRotation = focusOrientationYaw - Math.PI / 2.0;
      
      centerOnFocusPoint();
   }
   
   public double getFocusPointX()
   {
      focusPoint.changeFrame(metersFrame);
      return focusPoint.getX();
   }
   
   public double getFocusPointY()
   {
      focusPoint.changeFrame(metersFrame);
      return focusPoint.getY();
   }
   
   /**
    * Specify amount of meters that occupy view in X and Y.
    */
   public void setViewRange(double viewRangeInX, double viewRangeInY)
   {
      setScale(getPlotterWidthPixels() / viewRangeInX, getPlotterHeightPixels() / viewRangeInY);
   }
   
   public void setViewRangeX(double viewRangeInX)
   {
      setScale(getPlotterWidthPixels() / viewRangeInX, metersToPixels.getY());
   }
   
   public void setViewRangeY(double viewRangeInY)
   {
      setScale(metersToPixels.getX(), getPlotterHeightPixels() / viewRangeInY);
   }
   
   public void setViewRange(double minimumViewRange)
   {
      double smallestDimension;
      smallestDimension = Math.min(getPlotterWidthPixels(), getPlotterHeightPixels());
      double newPixelsPerMeter = smallestDimension / minimumViewRange;
      setScale(newPixelsPerMeter, newPixelsPerMeter);
   }
   
   public void setScale(double pixelsPerMeter)
   {
      setScale(pixelsPerMeter, pixelsPerMeter);
   }

   public void setShowLabels(boolean showLabels)
   {
      this.showLabels = showLabels;
   }
   
   public void setXYZoomEnabled(boolean xyZoomEnabled)
   {
      this.xyZoomEnabled = xyZoomEnabled;
   }
   
   public void setRotationEnabled(boolean rotationEnabled)
   {
      this.rotationEnabled = rotationEnabled;
   }

   public void showInNewWindow()
   {
      showInNewWindow("Plotter", false);
   }
   
   public void showInNewWindow(String title, boolean showLegend)
   {
      JFrame frame = new JFrame(title);
      if (showLegend)
      {
         frame.getContentPane().add(createAndAttachPlotterLegendPanel(), BorderLayout.CENTER);
      }
      else
      {
         frame.getContentPane().add(panel, BorderLayout.CENTER);
      }
      frame.pack();
      frame.setVisible(true);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   }
   
   public double getSelectedX()
   {
      selected.changeFrame(metersFrame);
      return selected.getX();
   }

   public double getSelectedY()
   {
      selected.changeFrame(metersFrame);
      return selected.getY();
   }
   
   public void setBackgroundImage(BufferedImage backgroundImage)
   {
      this.backgroundImage = backgroundImage;
      panel.repaint();
   }

   public double getViewRange()
   {
      if (getPlotterWidthPixels() <= getPlotterHeightPixels())
      {
         return metersToPixels.getX() * getPlotterWidthPixels();
      }
      else
      {
         return metersToPixels.getY() * getPlotterHeightPixels();
      }
   }

   public double getViewRangeX()
   {
      return metersToPixels.getX() * getPlotterWidthPixels();
   }

   public double getViewRangeY()
   {
      return metersToPixels.getY() * getPlotterWidthPixels();
   }

   public void setDrawHistory(boolean drawHistory)
   {
      this.showHistory = drawHistory;
   }

   public void update()
   {
      panel.repaint();
   }
   
   public JPanel getJPanel()
   {
      return panel;
   }
   
   public Graphics2DAdapter getGraphics2DAdapter()
   {
      return graphics2dAdapter;
   }
   
   public Plotter2DAdapter getPlotter2DAdapter()
   {
      return plotter2dAdapter;
   }

   public PlotterLegendPanel createPlotterLegendPanel()
   {
      PlotterLegendPanel plotterLegendPanel = new PlotterLegendPanel(plotter2dAdapter);
      addArtifactsChangedListener(plotterLegendPanel);
      return plotterLegendPanel;
   }

   public JPanel createAndAttachPlotterLegendPanel()
   {
      JPanel flashyNewJayPanel = new JPanel();
      PlotterLegendPanel plotterLegendPanel = new PlotterLegendPanel(plotter2dAdapter);
      plotterLegendPanel.setPreferredSize(new Dimension(500, 500));
      addArtifactsChangedListener(plotterLegendPanel);
      flashyNewJayPanel.setLayout(new BorderLayout());
      flashyNewJayPanel.add(panel, "Center");
      flashyNewJayPanel.add(plotterLegendPanel, "South");
      return flashyNewJayPanel;
   }
   
   // BEGIN ARTIFACT GARBAGE //
   
   public void updateArtifacts(Vector<Artifact> artifacts)
   {
      this.artifacts.clear();

      for (Artifact a : artifacts)
      {
         this.artifacts.put(a.getID(), a);
      }

      notifyArtifactsChangedListeners();
      panel.repaint();
   }

   public void updateArtifact(Artifact newArtifact)
   {
      synchronized (artifacts)
      {
         artifacts.put(newArtifact.getID(), newArtifact);
      }

      notifyArtifactsChangedListeners();
      panel.repaint();
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

   public PointListArtifact createAndAddPointArtifact(String name, Point2D point, Color color)
   {
      PointListArtifact pointArtifact = new PointListArtifact(name, point);
      pointArtifact.setColor(color);
      addArtifact(pointArtifact);

      return pointArtifact;
   }

   @Override
   public void addArtifact(Artifact newArtifact)
   {
      synchronized (artifacts)
      {
         artifacts.put(newArtifact.getID(), newArtifact);
      }

      notifyArtifactsChangedListeners();
      panel.repaint();
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
      panel.repaint();
   }

   public void removeArtifact(String id)
   {
      synchronized (artifacts)
      {
         artifacts.remove(id);
      }

      notifyArtifactsChangedListeners();
      panel.repaint();
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
      panel.repaint();
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
   
   // END ARTIFACT GARBAGE //
}
