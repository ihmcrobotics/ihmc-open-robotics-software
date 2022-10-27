package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.DMatchVector;
import org.bytedeco.opencv.opencv_core.KeyPointVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_features2d.DescriptorMatcher;
import org.bytedeco.opencv.opencv_features2d.ORB;
import us.ihmc.bytedeco.mapsenseWrapper.VisualOdometry;
import us.ihmc.log.LogTools;

import static org.bytedeco.opencv.global.opencv_features2d.drawKeypoints;
import static org.bytedeco.opencv.global.opencv_features2d.drawMatches;
import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;
import static org.bytedeco.opencv.global.opencv_imgcodecs.IMREAD_GRAYSCALE;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;

public class VisualOdometryModule
{
   private final VisualOdometry.VisualOdometryExternal visualOdometryExternal;

   private static final String LEFT_CAMERA_NAME = "image_2";
   private static final String RIGHT_CAMERA_NAME = "image_2";

   private static final String DATASET_PATH = "/home/bmishra/Workspace/Data/Datasets/dataset/sequences/00/";

   private ImageMat previousImage;
   private ImageMat currentImage;
   private ImageMat displayImage;

   private String fileName = "000000.png";
   private int frameIndex = 0;
   private boolean initialized = false;

   public VisualOdometryModule()
   {
      BytedecoTools.loadMapsenseLibraries();
      visualOdometryExternal = new VisualOdometry.VisualOdometryExternal();
   }

   public void update()
   {
      if(!initialized)
      {
         previousImage = ImageTools.loadAsImageMat(DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName);
         initialized = true;
         return;
      }

      currentImage = ImageTools.loadAsImageMat(DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName);

      visualOdometryExternal.displayMat(currentImage.getData(), currentImage.getRows(), currentImage.getCols(), 1);

      LogTools.info("File Name: {}", DATASET_PATH + fileName);

      fileName = String.format("%1$6s", frameIndex).replace(' ', '0') + ".png";
      frameIndex++;
   }

   public void render()
   {

   }

   public static void main(String[] args)
   {
      VisualOdometryModule vo = new VisualOdometryModule();

      for (int i = 0; i < 4500; i++)
      {
         long start = System.nanoTime();
         vo.update();
         long end = System.nanoTime();
         System.out.println("Time Taken (Update): " + (end - start) / 1000 + "us");
      }
   }
}
