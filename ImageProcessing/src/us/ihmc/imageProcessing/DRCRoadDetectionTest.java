


package us.ihmc.imageProcessing;

import georegression.struct.line.LineParametric2D_F32;

import java.awt.GridLayout;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import jxl.format.RGB;
import us.ihmc.imageProcessing.ImageFilters.ColorFilter;
import us.ihmc.imageProcessing.utilities.VideoPlayer;
import us.ihmc.utilities.camera.VideoListener;
import boofcv.abst.feature.detect.line.DetectLineHoughPolar;
import boofcv.alg.filter.binary.ThresholdImageOps;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.feature.detect.line.FactoryDetectLineAlgs;
import boofcv.gui.binary.VisualizeBinaryData;
import boofcv.gui.feature.ImageLinePanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageSInt16;
import boofcv.struct.image.ImageSingleBand;
import boofcv.struct.image.ImageUInt8;

public class DRCRoadDetectionTest implements VideoListener
{
   // adjusts edge threshold for identifying pixels belonging to a line

   // adjust the maximum number of found lines in the image
   private int maxLines = 8;
   private int localMaxRadius = 5;
   private int minCounts = 22;
   private double resolutionRange = 1;
   private double resolutionAngle = Math.toRadians(1);
   private float edgeThreshold = 11;
   float mean = 58;    // (float)ImageStatistics.mean(input);
   ColorFilter filter;

   public DRCRoadDetectionTest()
   {
      filter = new ColorFilter();
      filter.addColorToLookFor(new RGB(48, 48, 46));
      filter.addColorToLookFor(new RGB(50, 50, 50));
      filter.addColorToLookFor(new RGB(55, 56, 50));

      filter.addColorToLookFor(new RGB(68, 68, 66));
      filter.addColorToLookFor(new RGB(71, 72, 67));


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
   public <T extends ImageSingleBand<?>, D extends ImageSingleBand<?>> ArrayList<LineParametric2D_F32> detectLines(BufferedImage image, Class<T> imageType,
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
      ArrayList<LineParametric2D_F32> finalList = new ArrayList<LineParametric2D_F32>();
      for (LineParametric2D_F32 checkLine : found)
      {
         if ((checkLine.getSlopeY() / checkLine.getSlopeX()) > 0.1 || (checkLine.getSlopeY() / checkLine.getSlopeX()) < -0.1)
         {
            finalList.add(checkLine);
         }

      }


      return finalList;
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

      // ShowImages.showWindow(visualBinary,"Binary Image");
      return visualBinary;
   }

   public void updateImage(BufferedImage bufferedImage)
   {
    //  BufferedImage input = UtilImageIO.loadImage(DRCRoadDetectionTest.class.getResource("exampleVideo/DrivingTaskRoad.jpg").getFile());
//
     // process(input);

    process(bufferedImage);


   }

   private void process(BufferedImage input)
   {
       filter.filter(input, input);

      // BoxBlurFilter boxBlur = new BoxBlurFilter(5, 5, 1);
      // boxBlur.filter(input, input);

      // input = filterbinaryExample(input);

      ArrayList<LineParametric2D_F32> list = detectLines(input, ImageUInt8.class, ImageSInt16.class);
      gui.setBackground(input);


      gui.setLines(list);
      gui.repaint();
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
         final JSlider slider = new JSlider(1, 200, new Double(edgeThreshold).intValue());
         slider.setOrientation(JSlider.VERTICAL);
         slider.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent arg0)
            {
               edgeThreshold = new Float(slider.getValue());


               System.out.println("edgeThreshold: " + edgeThreshold);

               // process();

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

               // process();

            }
         });

         tmp.getContentPane().add(slider);
      }


      tmp.pack();

      tmp.setVisible(true);
      tmp.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


      gui = new ImageLinePanel();


      ShowImages.showWindow(gui, "Found Line Segments");

   }

   ImageLinePanel gui;

   public static void main(String args[])
   {
      DRCRoadDetectionTest drcRoadDetectionTest = new DRCRoadDetectionTest();
      final VideoPlayer videoPlayer = new VideoPlayer("./media/videos/run1.mov", drcRoadDetectionTest, true);

//    JFrame jFrame = new JFrame("Video Player Test");
//    jFrame.addWindowListener(new WindowAdapter()
//    {
//       public void windowClosing(WindowEvent e)
//       {
//          videoPlayer.close();
//          System.exit(0);
//       }
//    });
//
//    jFrame.getContentPane().add(imageViewer, BorderLayout.CENTER);
//    jFrame.pack();
//    jFrame.setVisible(true);

      videoPlayer.start();
   }
}
