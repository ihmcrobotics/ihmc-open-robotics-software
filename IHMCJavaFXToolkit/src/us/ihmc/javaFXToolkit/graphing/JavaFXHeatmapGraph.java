package us.ihmc.javaFXToolkit.graphing;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Optional;

import javax.imageio.ImageIO;

import gnu.trove.map.hash.TObjectIntHashMap;
import javafx.application.Platform;
import javafx.embed.swing.JFXPanel;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.Group;
import javafx.scene.ParallelCamera;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.color.ColorConversions;
import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.color.Gradient;

public class JavaFXHeatmapGraph
{
   private final JFXPanel javaFXPanel;
   private Group rootGroup;
   private final GraphIndicesHolder graphIndicesHolder;
   private final DataEntryHolder dataEntryHolder;
   
   private final Point2D focusPoint;
   private final AffineTransform transformToCanvasSpace;
   
   private TObjectIntHashMap<Point2D> heatmap;

   private DoubleYoVariable x;
   private DoubleYoVariable y;
   private GraphicsContext graphicsContext;
   private Scene scene;

   private Optional<Point2D> adjustingViewRangeMax;
   private Optional<Point2D> adjustingViewRangeMin;
   private JavaFXGraphColors colors;
   private Canvas canvas;
   private ParallelCamera parallelCamera;
   private Point2D gridCenter;
   private Point2D plotPencil;
   private Point2D viewRange;

   public JavaFXHeatmapGraph(YoVariableRegistry registry, GraphIndicesHolder graphIndicesHolder, SelectedVariableHolder selectedVariableHolder,
                             DataEntryHolder dataEntryHolder, TimeDataHolder dataBuffer)
   {
      javaFXPanel = new JFXPanel();
      this.graphIndicesHolder = graphIndicesHolder;
      this.dataEntryHolder = dataEntryHolder;

      heatmap = new TObjectIntHashMap<Point2D>(dataBuffer.getTimeData().length);
      
      adjustingViewRangeMax = Optional.empty();
      adjustingViewRangeMin = Optional.empty();

      focusPoint = new Point2D(1.0, 1.0);
      transformToCanvasSpace = new AffineTransform();
      transformToCanvasSpace.setScale(50.0, 50.0, 1.0);

      gridCenter = new Point2D();
      plotPencil = new Point2D();
      viewRange = new Point2D();

      colors = JavaFXGraphColors.javaFXStyle();

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            rootGroup = new Group();
            canvas = new Canvas();
            graphicsContext = canvas.getGraphicsContext2D();
            rootGroup.getChildren().add(canvas);

            scene = new Scene(rootGroup);
            javaFXPanel.setScene(scene);

            parallelCamera = new ParallelCamera();
            scene.setCamera(parallelCamera);
         }
      });
   }

   public void update()
   {
      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            // Update canvas for panel resize
            canvas.setWidth(javaFXPanel.getWidth());
            canvas.setHeight(javaFXPanel.getHeight());

            transformToCanvasSpace.setTranslationX((javaFXPanel.getWidth() / 2.0) - (focusPoint.getX() * transformToCanvasSpace.getScaleX()));
            transformToCanvasSpace.setTranslationY((javaFXPanel.getHeight() / 2.0) - (focusPoint.getY() * transformToCanvasSpace.getScaleY()));

            // save graphics context
            graphicsContext.save();

            // background
            graphicsContext.setFill(colors.getBackgroundColor());
            graphicsContext.fillRect(0.0, 0.0, javaFXPanel.getWidth(), javaFXPanel.getHeight());

            // grid lines
            graphicsContext.setStroke(colors.getGridAxisColor());
            gridCenter.set(0.0, 0.0);
            transformToCanvasSpace.transform(gridCenter);
            graphicsContext.strokeLine(0, gridCenter.getY(), javaFXPanel.getWidth(), gridCenter.getY());
            graphicsContext.strokeLine(gridCenter.getX(), 0.0, gridCenter.getX(), javaFXPanel.getHeight());

            plotXYHeatmap();
            
            graphicsContext.setStroke(colors.getLabelColor());
            graphicsContext.strokeText(x.getName(), canvas.getWidth() / 2, canvas.getHeight() - 5);
            graphicsContext.rotate(-90.0);
            graphicsContext.strokeText(y.getName(), -canvas.getHeight() / 2, 10);
            graphicsContext.rotate(90.0);

            // restore context for next draw
            graphicsContext.restore();
         }
      });
   }

   private void plotXYHeatmap()
   {
      DataEntry xDataEntry = dataEntryHolder.getEntry(x);
      DataEntry yDataEntry = dataEntryHolder.getEntry(y);

      double discreteX = 0.09;
      double discreteY = 0.3;
      
      for (int i = graphIndicesHolder.getInPoint(); i < graphIndicesHolder.getIndex(); i++)
      {
         double roundedX = MathTools.roundToPrecision(xDataEntry.getData()[i], discreteX);
         double roundedY = MathTools.roundToPrecision(yDataEntry.getData()[i], discreteY);
         
         plotPencil.set(roundedX, roundedY);
         
         heatmap.adjustOrPutValue(plotPencil, 1, 1);
         int heat = heatmap.get(plotPencil);
         
         adjustViewRange(plotPencil.getX(), plotPencil.getY());
         
         transformToCanvasSpace.transform(plotPencil);
         
         graphicsContext.setFill(getHeatColor(heat));
         fillRect(plotPencil.getX(), plotPencil.getY(), discreteX * transformToCanvasSpace.getScaleX(), discreteY * transformToCanvasSpace.getScaleY());
      }
      
      heatmap.clear();
   }
   
   private Color getHeatColor(int heat)
   {
      double maxHeat = 30.0;
      int heatIndex = (int) MathTools.roundToPrecision(MathTools.clamp((heat / maxHeat) * 500.0, 0.0, 499.0), 1.0);
      return ColorConversions.awtToJfx(Gradient.GRADIENT_RAINBOW[heatIndex]);
   }

   private void fillRect(double x, double y, double width, double height)
   {
      double upperLeftX = x - (width / 2.0);
      double upperLeftY = y + (height / 2.0);
      graphicsContext.fillRect(upperLeftX, upperLeftY, width, height);
   }

   public void setXVariable(DoubleYoVariable x)
   {
      this.x = x;
   }

   public void setYVariable(DoubleYoVariable y)
   {
      this.y = y;
   }

   public Scene getScene()
   {
      return new Scene(rootGroup);
   }

   public JFXPanel getPanel()
   {
      return javaFXPanel;
   }

   private void setViewRange(double viewRangeXMeters, double viewRangeYMeters)
   {
      transformToCanvasSpace.setScale(javaFXPanel.getWidth() / viewRangeXMeters, javaFXPanel.getHeight() / viewRangeYMeters, 1.0);
   }

   public BufferedImage snapshot()
   {
      WritableImage snapshot = scene.snapshot(null);
      BufferedImage fromFXImage = SwingFXUtils.fromFXImage(snapshot, null);
      BufferedImage pngImage = null;
      byte[] imageInByte;
      try
      {
         ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
         ImageIO.write(fromFXImage, "png", byteArrayOutputStream);
         byteArrayOutputStream.flush();
         imageInByte = byteArrayOutputStream.toByteArray();
         byteArrayOutputStream.close();
         InputStream in = new ByteArrayInputStream(imageInByte);
         pngImage = ImageIO.read(in);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      return pngImage;
   }

   private void adjustViewRange(double xInMeters, double yInMeters)
   {
      viewRange.set(xInMeters, yInMeters);

      if (!adjustingViewRangeMax.isPresent())
      {
         adjustingViewRangeMax = Optional.of(new Point2D(viewRange.getX() + Epsilons.ONE_THOUSANDTH, viewRange.getY() + Epsilons.ONE_THOUSANDTH));
         adjustingViewRangeMin = Optional.of(new Point2D(viewRange.getX() - Epsilons.ONE_THOUSANDTH, viewRange.getY() - Epsilons.ONE_THOUSANDTH));
         focusPoint.setX(xInMeters);
         focusPoint.setY(yInMeters);
         return;
      }

      if (xInMeters > adjustingViewRangeMax.get().getX())
      {
         adjustingViewRangeMax.get().setX(xInMeters);
      }
      if (yInMeters > adjustingViewRangeMax.get().getY())
      {
         adjustingViewRangeMax.get().setY(yInMeters);
      }
      if (xInMeters < adjustingViewRangeMin.get().getX())
      {
         adjustingViewRangeMin.get().setX(xInMeters);
      }
      if (yInMeters < adjustingViewRangeMin.get().getY())
      {
         adjustingViewRangeMin.get().setY(yInMeters);
      }

      focusPoint.setX(adjustingViewRangeMin.get().getX() + ((adjustingViewRangeMax.get().getX() - adjustingViewRangeMin.get().getX()) / 2.0));
      focusPoint.setY(adjustingViewRangeMin.get().getY() + ((adjustingViewRangeMax.get().getY() - adjustingViewRangeMin.get().getY()) / 2.0));

      setViewRange((adjustingViewRangeMax.get().getX() - adjustingViewRangeMin.get().getX()) * 1.10,
                   (adjustingViewRangeMax.get().getY() - adjustingViewRangeMin.get().getY()) * 1.10);
   }
}
