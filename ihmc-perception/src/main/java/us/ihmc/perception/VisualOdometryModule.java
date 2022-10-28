package us.ihmc.perception;

import us.ihmc.bytedeco.mapsenseWrapper.VisualOdometry;
import us.ihmc.log.LogTools;

public class VisualOdometryModule
{
   private final VisualOdometry.VisualOdometryExternal visualOdometryExternal;

   private static final String LEFT_CAMERA_NAME = "image_0";
   private static final String RIGHT_CAMERA_NAME = "image_1";

   private static final String DATASET_PATH = "/home/quantum/Workspace/Data/Datasets/sequences/00/";

   private ImageMat currentImageRight;
   private ImageMat currentImageLeft;

   private String leftImageName;
   private String rightImageName;

   private ImageMat displayImageLeft;

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
      fileName = String.format("%1$6s", frameIndex).replace(' ', '0') + ".png";
      leftImageName = DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName;
      rightImageName = DATASET_PATH + RIGHT_CAMERA_NAME + "/" + fileName;

      currentImageLeft = ImageTools.loadAsImageMat(leftImageName);
      currentImageRight = ImageTools.loadAsImageMat(rightImageName);

      visualOdometryExternal.displayMat(currentImageLeft.getData(), currentImageLeft.getRows(), currentImageLeft.getCols(), 1);

      visualOdometryExternal.updateStereo(currentImageLeft.getData(), currentImageRight.getData(), currentImageLeft.getRows(), currentImageLeft.getCols());


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
