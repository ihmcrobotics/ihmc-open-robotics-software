


package us.ihmc.imageProcessing;

import boofcv.abst.feature.detect.line.DetectLineHoughPolar;
import boofcv.alg.filter.binary.ThresholdImageOps;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.feature.detect.line.FactoryDetectLineAlgs;
import boofcv.gui.binary.VisualizeBinaryData;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageSInt16;
import boofcv.struct.image.ImageSingleBand;
import boofcv.struct.image.ImageUInt8;
import georegression.struct.line.LineParametric2D_F32;
import georegression.struct.point.Point2D_F32;
import jxl.format.RGB;
import us.ihmc.imageProcessing.ImageFilters.ColorFilter;
import us.ihmc.imageProcessing.ImageFilters.CropFilter;
import us.ihmc.imageProcessing.driving.VanishingPointDetector;
import us.ihmc.imageProcessing.utilities.LinePainter;
import us.ihmc.imageProcessing.utilities.PaintableImageViewer;
import us.ihmc.imageProcessing.utilities.VideoPlayer;
import us.ihmc.utilities.camera.ImageViewer;
import us.ihmc.utilities.camera.VideoListener;
import us.ihmc.utilities.math.geometry.Line2d;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.vecmath.Point2d;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

public class DRCRoadDetectionTest implements VideoListener, KeyListener
{
   boolean PAUSE = false;

   private ImageViewer rawImageViewer = new ImageViewer();
   private PaintableImageViewer analyzedImageViewer = new PaintableImageViewer();
   private LinePainter linePainter = new LinePainter(4.0f);
   private VanishingPointDetector vanishingPointDetector = new VanishingPointDetector(Color.cyan, 20);

   JFrame f;
   // adjusts edge threshold for identifying pixels belonging to a line

   // adjust the maximum number of found lines in the image
   private int maxLines = 8;
   private int localMaxRadius = 5;
   private int minCounts = 22;
   private double resolutionRange = 1;
   private double resolutionAngle = Math.toRadians(1);
   private float edgeThreshold = 11;
   float mean = 58;    // (float)ImageStatistics.mean(input);

   private ColorFilter filter;

   public DRCRoadDetectionTest()
   {
      filter = new ColorFilter();
      filter.addColorToLookFor(new RGB(48, 48, 46));
      filter.addColorToLookFor(new RGB(50, 50, 50));
      filter.addColorToLookFor(new RGB(55, 56, 50));

      filter.addColorToLookFor(new RGB(68, 68, 66));
      filter.addColorToLookFor(new RGB(71, 72, 67));


      analyzedImageViewer.addPostProcessor(linePainter);
      analyzedImageViewer.addPostProcessor(vanishingPointDetector);
      setUpJFrame();

      // process();
   }


   /**
    * Detects lines inside the image using different types of Hough detectors
    *
    * @param image     Input image.
    * @param imageType Type of image processed by line detector.
    * @param derivType Type of image derivative.
    */
   public <T extends ImageSingleBand<?>, D extends ImageSingleBand<?>> List<LineParametric2D_F32> detectLines(BufferedImage image, Class<T> imageType, Class<D> derivType)
   {
      // convert the line into a single band image
      T input = ConvertBufferedImage.convertFromSingle(image, null, imageType);

      // Comment/uncomment to try a different type of line detector
      DetectLineHoughPolar<T, D> detector = FactoryDetectLineAlgs.houghPolar(localMaxRadius, minCounts, resolutionRange, resolutionAngle, edgeThreshold, maxLines, imageType, derivType);

      // DetectLineHoughFoot<T,D> detector = FactoryDetectLineAlgs.houghFoot(3, 8, 5, edgeThreshold,
      // maxLines, imageType, derivType);
      // DetectLineHoughFootSubimage<T,D> detector = FactoryDetectLineAlgs.houghFootSub(3, 8, 5, edgeThreshold,
      // maxLines, 2, 2, imageType, derivType);

      List<LineParametric2D_F32> found = detector.detect(input);

      // ArrayList<LineParametric2D_F32> finalList = new ArrayList<LineParametric2D_F32>();
      // for (LineParametric2D_F32 checkLine : found)
      // {
      // if ((checkLine.getSlopeY() / checkLine.getSlopeX()) > 0.1 || (checkLine.getSlopeY() / checkLine.getSlopeX()) < -0.1)
      // {
      // finalList.add(checkLine);
      // }

      // }


      return found;
   }


   public BufferedImage filterbinaryExample(BufferedImage image)
   {
      // convert into a usable format
      ImageFloat32 input = ConvertBufferedImage.convertFromSingle(image, null, ImageFloat32.class);
      ImageUInt8 binary = new ImageUInt8(input.width, input.height);

      // the mean pixel value is often a reasonable threshold when creating a binary image


      // create a binary image
      ThresholdImageOps.threshold(input, binary, mean, true);

      // Render the binary image for output and display it in a window
      BufferedImage visualBinary = VisualizeBinaryData.renderBinary(binary, null);

      //    ShowImages.showWindow(visualBinary,"Binary Image");
      return visualBinary;
   }

   public void updateImage(BufferedImage bufferedImage)
   {
      process(bufferedImage);
   }

   private void process(BufferedImage input)
   {
      if (!PAUSE)
      {
         CropFilter cropFilter = new CropFilter(0, 0, input.getWidth(), input.getHeight() - input.getHeight() / 5);
         BufferedImage croppedImage = new BufferedImage(input.getWidth(), input.getHeight() - input.getHeight() / 5, BufferedImage.TYPE_INT_RGB);
         cropFilter.filter(input, croppedImage);
         filter.filter(croppedImage, croppedImage);

         // BoxBlurFilter boxBlur = new BoxBlurFilter(5, 5, 1);
         // boxBlur.filter(input, input);

         // input = filterbinaryExample(input);

         List<LineParametric2D_F32> list = detectLines(croppedImage, ImageUInt8.class, ImageSInt16.class);
         ArrayList<Line2d> lines = new ArrayList<Line2d>();
         for (LineParametric2D_F32 lineParametric2D_f32 : list)
         {
            Point2D_F32 pointOnLine1 = lineParametric2D_f32.getPointOnLine(0.0f);
            Point2d p1 = new Point2d(pointOnLine1.getX(), pointOnLine1.getY());
            Point2D_F32 pointOnLine2 = lineParametric2D_f32.getPointOnLine(1.0f);
            Point2d p2 = new Point2d(pointOnLine2.getX(), pointOnLine2.getY());

            Line2d line2d = new Line2d(p1, p2);
            lines.add(line2d);
         }
         linePainter.setLines(lines);
         vanishingPointDetector.setLines(lines);

         repackIfImageSizeChanges(croppedImage.getWidth(), croppedImage.getHeight());
         analyzedImageViewer.updateImage(croppedImage);
         repackRawIfImageSizeChanges(input.getWidth(), input.getHeight());
         rawImageViewer.updateImage(input);
      }
   }

   private void repackIfImageSizeChanges(int width, int height)
   {
      if (analyzedImageViewer.getWidth() < width || analyzedImageViewer.getHeight() < height)
      {
         Dimension dimension = new Dimension(width, height);
         analyzedImageViewer.setPreferredSize(dimension);
         linePainter.setImageHeight(height);
         f.pack();
      }
   }

   private void repackRawIfImageSizeChanges(int width, int height)
   {
      if (rawImageViewer.getWidth() < width || rawImageViewer.getHeight() < height)
      {
         Dimension dimension = new Dimension(width, height);
         rawImageViewer.setPreferredSize(dimension);
         f.pack();
      }
   }

   public void setUpJFrame()
   {
      JFrame tmp = new JFrame();
      tmp.getContentPane().setLayout(new GridLayout(1, 6));

      {
         final JSlider slider = new JSlider(1, 20, localMaxRadius);
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               localMaxRadius = slider.getValue();


               System.out.println("localMaxRadius: " + localMaxRadius);
               //               process();

            }
         });

         tmp.getContentPane().add(slider);
      }


      {
         final JSlider slider = new JSlider(1, 200, minCounts);
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               minCounts = slider.getValue();


               System.out.println("minCounts: " + minCounts);
               //               process();

            }
         });

         tmp.getContentPane().add(slider);
      }

      {
         final JSlider slider = new JSlider(1, 200, new Double(resolutionRange).intValue() * 10);
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               resolutionRange = new Double(slider.getValue()) / 10.0;


               System.out.println("resolutionRange: " + resolutionRange);
               //               process();

            }
         });

         tmp.getContentPane().add(slider);
      }

      {
         final JSlider slider = new JSlider(1, 200, new Double(edgeThreshold).intValue());
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               edgeThreshold = new Float(slider.getValue());


               System.out.println("edgeThreshold: " + edgeThreshold);
               //               process();

            }
         });

         tmp.getContentPane().add(slider);
      }

      {
         final JSlider slider = new JSlider(1, 200, new Float(mean).intValue());
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               mean = new Float(slider.getValue());


               System.out.println("mean: " + mean);
               //               process();

            }
         });

         tmp.getContentPane().add(slider);
      }


      {
         final JSlider slider = new JSlider(0, 500, new Double(filter.getThreshold()).intValue());
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               filter.setThreshold(new Double(slider.getValue()));


               System.out.println("filter threshold: " + filter.getThreshold());

            }
         });

         tmp.getContentPane().add(slider);
      }


      tmp.pack();

      tmp.setVisible(true);
      tmp.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      f = new JFrame("Driving Algorithm Test");
      f.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            System.exit(0);
         }
      });
      f.addKeyListener(this);

      f.getContentPane().add(rawImageViewer, BorderLayout.WEST);
      f.getContentPane().add(analyzedImageViewer, BorderLayout.EAST);
      f.pack();
      f.setVisible(true);

   }

   public static void main(String args[])
   {
      DRCRoadDetectionTest drcRoadDetectionTest = new DRCRoadDetectionTest();
      final VideoPlayer videoPlayer = new VideoPlayer("./media/videos/run1.mov", drcRoadDetectionTest, true);
      videoPlayer.start();
   }

   public void keyTyped(KeyEvent e)
   {

   }

   public void keyPressed(KeyEvent e)
   {
      if (e.getKeyCode() == KeyEvent.VK_SPACE)
      {
         PAUSE = !PAUSE;
      }
   }

   public void keyReleased(KeyEvent e)
   {

   }
}
