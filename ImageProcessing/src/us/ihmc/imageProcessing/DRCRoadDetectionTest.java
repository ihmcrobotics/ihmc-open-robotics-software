


package us.ihmc.imageProcessing;

import georegression.struct.line.LineParametric2D_F32;
import georegression.struct.point.Point2D_F32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import jxl.format.RGB;
import us.ihmc.imageProcessing.ImageFilters.ColorFilter;
import us.ihmc.imageProcessing.ImageFilters.CropFilter;
import us.ihmc.imageProcessing.driving.VanishingPointDetector;
import us.ihmc.imageProcessing.utilities.CirclePainter;
import us.ihmc.imageProcessing.utilities.LinePainter;
import us.ihmc.imageProcessing.utilities.PaintableImageViewer;
import us.ihmc.imageProcessing.utilities.VideoPlayer;
import us.ihmc.utilities.camera.VideoListener;
import us.ihmc.utilities.math.geometry.Line2d;
import boofcv.abst.feature.detect.edge.DetectEdgeContour;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.abst.feature.detect.interest.InterestPointDetector;
import boofcv.abst.feature.detect.line.DetectLineHoughPolar;
import boofcv.alg.filter.binary.BinaryImageOps;
import boofcv.alg.filter.binary.ThresholdImageOps;
import boofcv.alg.misc.ImageStatistics;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.feature.detect.edge.FactoryDetectEdgeContour;
import boofcv.factory.feature.detect.interest.FactoryInterestPoint;
import boofcv.factory.feature.detect.line.FactoryDetectLineAlgs;
import boofcv.gui.binary.VisualizeBinaryData;
import boofcv.gui.feature.FancyInterestPointRender;
import boofcv.gui.feature.FancyInterestPointRender.Circle;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.BoofDefaults;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageSInt16;
import boofcv.struct.image.ImageSInt32;
import boofcv.struct.image.ImageSingleBand;
import boofcv.struct.image.ImageUInt8;

public class DRCRoadDetectionTest implements VideoListener, KeyListener
{
   boolean PAUSE = false;

   private PaintableImageViewer rawImageViewer = new PaintableImageViewer();
   private PaintableImageViewer analyzedImageViewer = new PaintableImageViewer();
   private LinePainter linePainter = new LinePainter(4.0f);
   private CirclePainter circlePainter = new CirclePainter(4.0f);

   private VanishingPointDetector vanishingPointDetector = new VanishingPointDetector(Color.cyan, 20);

   JFrame f;

   // adjusts edge threshold for identifying pixels belonging to a line

   // adjust the maximum number of found lines in the image


   private int maxLines = 10;
   private int localMaxRadius = 3;
   private int minCounts = 100;
   private double resolutionRange = 1;
   private double resolutionAngle = Math.PI / 180;
   private float edgeThreshold = 25;
   float mean = 58;    // (float)ImageStatistics.mean(input);


   private double lowThresh = 0.01;
   private double highThresh = 0.15;

   private ColorFilter filter;

   public DRCRoadDetectionTest()
   {
      filterCone.setThreshold(25);
      filterCone.filterHorizon(false);
      filterCone.addColorToLookFor(new RGB(84, 51, 46));
      filterCone.addColorToLookFor(new RGB(63, 28, 22));
      filterCone.addColorToLookFor(new RGB(161, 91, 91));
      filterCone.addColorToLookFor(new RGB(69, 41, 29));


      filter = new ColorFilter();
      filter.filterHorizon(true);

      // middle line
      // filter.addColorToLookFor(new RGB(152, 128, 32));
      // filter.addColorToLookFor(new RGB(152, 128, 30));
      // filter.addColorToLookFor(new RGB(135, 113, 37));
      // filter.addColorToLookFor(new RGB(69, 63, 41));
      // filter.addColorToLookFor(new RGB(77, 67, 40));



      // side lines
      filter.addColorToLookFor(new RGB(148, 144, 135));
      filter.addColorToLookFor(new RGB(180, 180, 172));
      filter.addColorToLookFor(new RGB(151, 151, 143));
      filter.addColorToLookFor(new RGB(181, 177, 168));

      // road
      // filter.addColorToLookFor(new RGB(55,56, 38));

      // filter.addColorToLookFor(new RGB(64, 64, 56));
      // /  filter.addColorToLookFor(new RGB(75, 70, 64));
      // filter.addColorToLookFor(new RGB(97, 88, 81));
      // filter.addColorToLookFor(new RGB(56, 47, 48));
      // filter.addColorToLookFor(new RGB(87, 88, 72));



      // tuned for new world
//    filter.addColorToLookFor(new RGB(48, 48, 46));
//    filter.addColorToLookFor(new RGB(50, 50, 50));
//    filter.addColorToLookFor(new RGB(55, 56, 50));
//
//    filter.addColorToLookFor(new RGB(68, 68, 66));
//    filter.addColorToLookFor(new RGB(71, 72, 67));


      analyzedImageViewer.addPostProcessor(linePainter);
      analyzedImageViewer.addPostProcessor(circlePainter);

      analyzedImageViewer.addPostProcessor(vanishingPointDetector);
      rawImageViewer.addPostProcessor(vanishingPointDetector);
      setUpJFrame();

      // process();
   }




   /**
    * Draws each edge in the image a different color
    */
   public BufferedImage canny(BufferedImage source)
   {
      DetectEdgeContour<ImageUInt8> contour = FactoryDetectEdgeContour.canny(lowThresh, highThresh, true, ImageUInt8.class, ImageSInt16.class);

      ImageUInt8 gray = ConvertBufferedImage.convertFrom(source, (ImageUInt8) null);

      contour.process(gray);

      List<List<Point2D_I32>> edges = contour.getContours();

      // draw each edge a different color
      BufferedImage out = new BufferedImage(gray.width, gray.height, BufferedImage.TYPE_INT_BGR);

      Random rand = new Random();
      for (List<Point2D_I32> l : edges)
      {
         int rgb = rand.nextInt() | 0x101010;

         for (Point2D_I32 p : l)
         {
            out.setRGB(p.x, p.y, rgb);
         }
      }

      return out;
   }




   /**
    * Detects lines inside the image using different types of Hough detectors
    *
    * @param image     Input image.
    * @param imageType Type of image processed by line detector.
    * @param derivType Type of image derivative.
    */
   public <T extends ImageSingleBand<?>, D extends ImageSingleBand<?>> List<LineParametric2D_F32> detectLines(BufferedImage image, Class<T> imageType,
           Class<D> derivType)
   {
      // convert the line into a single band image
      T input = ConvertBufferedImage.convertFromSingle(image, null, imageType);

      // Comment/uncomment to try a different type of line detector
      DetectLineHoughPolar<T, D> detector = FactoryDetectLineAlgs.houghPolar(localMaxRadius, minCounts, resolutionRange, resolutionAngle, edgeThreshold,
                                               maxLines, imageType, derivType);

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


   public void updateImage(BufferedImage bufferedImage)
   {
      // bufferedImage = UtilImageIO.loadImage(DRCRoadDetectionTest.class.getResource("exampleVideo/coneFinder.jpg").getFile());
      process(bufferedImage);
   }

   static BufferedImage deepCopy(BufferedImage bi)
   {
      ColorModel cm = bi.getColorModel();
      boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
      WritableRaster raster = bi.copyData(null);

      return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
   }

   ColorFilter filterCone = new ColorFilter();

   public InterestPointDetector<ImageFloat32> countConeBlobs(BufferedImage image)
   {
      filterCone.filter(image, image);
      ImageFloat32 input = ConvertBufferedImage.convertFromSingle(image, null, ImageFloat32.class);
      ImageUInt8 binary = new ImageUInt8(input.width, input.height);
      ImageSInt32 blobs = new ImageSInt32(input.width, input.height);

      // the mean pixel value is often a reasonable threshold when creating a binary image
      double mean = 10;    //

      // create a binary image
      ThresholdImageOps.threshold(input, binary, (float) mean, true);

      // remove small blobs through erosion and dilation
      // The null in the input indicates that it should internally declare the work image it needs
      // this is less efficient, but easier to code.
      binary = BinaryImageOps.erode8(binary, null);
      binary = BinaryImageOps.erode8(binary, null);

      // binary = BinaryImageOps.erode8(binary, null);

      binary = BinaryImageOps.dilate8(binary, null);
      binary = BinaryImageOps.dilate8(binary, null);

      // Detect blobs inside the binary image and assign labels to them
      int numBlobs = BinaryImageOps.labelBlobs4(binary, blobs);

      BufferedImage visualized = VisualizeBinaryData.renderLabeled(blobs, numBlobs, null);

      return detect(visualized);
   }

   int show = 0;




   private void process(BufferedImage input)
   {
      if (!PAUSE)
      {
         InterestPointDetector<ImageFloat32> cones = countConeBlobs(deepCopy(input));
         System.out.println(cones.getNumberOfFeatures());
         CropFilter cropFilter = new CropFilter(0, 0, input.getWidth(), input.getHeight() - input.getHeight() / 4);
         BufferedImage croppedImage = new BufferedImage(input.getWidth(), input.getHeight() - input.getHeight() / 4, BufferedImage.TYPE_INT_RGB);
         cropFilter.filter(input, croppedImage);
         filter.setHorizonYLocation(input.getHeight() / 2);
         filter.filter(croppedImage, croppedImage);

         // croppedImage = canny(croppedImage);


         // BoxBlurFilter boxBlur = new BoxBlurFilter(2, 2, 1);
         // boxBlur.filter(croppedImage, croppedImage);


         List<LineParametric2D_F32> list = detectLines(croppedImage, ImageUInt8.class, ImageSInt16.class);

         ArrayList<Line2d> lines = new ArrayList<Line2d>();

         ArrayList<Vector3d> circles = new ArrayList<Vector3d>();

         for (int i = 0; i < cones.getNumberOfFeatures(); i++)
         {
            Point2D_F64 pt = cones.getLocation(i);

            // note how it checks the capabilities of the detector
            double scale = cones.getScale(i);
            int radius = (int) (scale * BoofDefaults.SCALE_SPACE_CANONICAL_RADIUS);
            Vector3d tmp = new Vector3d(pt.x, pt.y, radius);
            circles.add(tmp);

//          circles.add(new Ci)render.addCircle((int)pt.x,(int)pt.y,radius);


         }

         circlePainter.setCircles(circles);

         for (LineParametric2D_F32 lineParametric2D_f32 : list)
         {
            Point2D_F32 pointOnLine1 = lineParametric2D_f32.getPointOnLine(0.0f);
            Point2d p1 = new Point2d(pointOnLine1.getX(), pointOnLine1.getY());
            Point2D_F32 pointOnLine2 = lineParametric2D_f32.getPointOnLine(1.0f);
            Point2d p2 = new Point2d(pointOnLine2.getX(), pointOnLine2.getY());

            Line2d line2d = new Line2d(p1, p2);
            lines.add(line2d);
         }

         lines = removeBadLines(lines);
         linePainter.setLines(lines);
         vanishingPointDetector.setLines(lines);

         repackIfImageSizeChanges(croppedImage.getWidth(), croppedImage.getHeight());
         analyzedImageViewer.updateImage(croppedImage);
         repackRawIfImageSizeChanges(input.getWidth(), input.getHeight());
         rawImageViewer.updateImage(input);
      }
   }

   private ArrayList<Line2d> removeBadLines(ArrayList<Line2d> lines)
   {
      ArrayList<Line2d> cleanedLines = new ArrayList<Line2d>();
      for (Line2d line : lines)
      {
         if ((line.getSlope() < 3.0) && (Math.abs(line.getSlope()) > 0.01))
         {
            cleanedLines.add(line);
         }
      }

      return cleanedLines;
   }

   private void repackIfImageSizeChanges(int width, int height)
   {
      if ((analyzedImageViewer.getWidth() < width) || (analyzedImageViewer.getHeight() < height))
      {
         Dimension dimension = new Dimension(width, height);
         analyzedImageViewer.setPreferredSize(dimension);
         linePainter.setImageHeight(height);
         circlePainter.setImageHeight(height);
         f.pack();
      }
   }

   private void repackRawIfImageSizeChanges(int width, int height)
   {
      if ((rawImageViewer.getWidth() < width) || (rawImageViewer.getHeight() < height))
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

               // process();

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

               // process();

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

               // process();

            }
         });

         tmp.getContentPane().add(slider);
      }

      {
         final JSlider slider = new JSlider(1, 200, new Double(lowThresh * 1000).intValue());
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               lowThresh = new Float(slider.getValue()) / 1000;


               System.out.println("lowThresh: " + lowThresh);

               // process();

            }
         });

         tmp.getContentPane().add(slider);
      }

      {
         final JSlider slider = new JSlider(1, 200, new Float(highThresh * 1000).intValue());
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               highThresh = new Float(slider.getValue()) / 1000;


               System.out.println("highThresh: " + highThresh);

               // process();

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

   public InterestPointDetector<ImageFloat32> detect(BufferedImage image)
   {
      ImageFloat32 input = ConvertBufferedImage.convertFromSingle(image, null, ImageFloat32.class);

      // Create a Fast Hessian detector from the SURF paper.
      // Other detectors can be used in this example too.
      InterestPointDetector<ImageFloat32> detector = FactoryInterestPoint.fastHessian(new ConfigFastHessian(10, 2, 100, 2, 9, 3, 4));

      // find interest points in the image
      detector.detect(input);

      // Show the features
      return detector;
   }

   private static <T extends ImageSingleBand> void displayResults(BufferedImage image, InterestPointDetector<T> detector)
   {
      Graphics2D g2 = image.createGraphics();
      FancyInterestPointRender render = new FancyInterestPointRender();


      for (int i = 0; i < detector.getNumberOfFeatures(); i++)
      {
         Point2D_F64 pt = detector.getLocation(i);

         // note how it checks the capabilities of the detector
         if (detector.hasScale())
         {
            double scale = detector.getScale(i);
            int radius = (int) (scale * BoofDefaults.SCALE_SPACE_CANONICAL_RADIUS);
            render.addCircle((int) pt.x, (int) pt.y, radius);
         }
         else
         {
            render.addPoint((int) pt.x, (int) pt.y);
         }
      }

      // make the circle's thicker
      g2.setStroke(new BasicStroke(3));

      // just draw the features onto the input image
      render.draw(g2);
      ShowImages.showWindow(image, "Detected Features");
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
