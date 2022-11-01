package us.ihmc.perception;

import us.ihmc.bytedeco.mapsenseWrapper.VisualOdometry;
import us.ihmc.bytedeco.slamWrapper.SlamWrapper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.ArrayList;
import java.util.List;

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

      BytedecoTools.loadGTSAMLibraries();
      SlamWrapper.FactorGraphExternal factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      //      factorGraphExternal.helloWorldTest();

      factorGraphExternal.visualSLAMTest();

      //float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      //float[] odometry = new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
      //
      //factorGraphExternal.addPriorPoseFactor(1, poseInitial);
      //factorGraphExternal.addOdometryFactor(odometry, 2);
      //
      //factorGraphExternal.setPoseInitialValue(1, poseInitial);
      //factorGraphExternal.setPoseInitialValue(2, odometry);
      //
      //factorGraphExternal.optimize();
      //
      //factorGraphExternal.printResults();
   }



   public void visualSLAMUpdate()
   {
      float[] angles = new float[]{1.5708f, 2.35619f, 3.14159f, -2.35619f, -1.5708f, -0.785398f, -1.83697e-16f, 0.785398f};

      double radius = 30.0;
      int i = 0;
      double theta = 0.0;
      Point3D up = new Point3D(0, 0, 1);
      Point3D target = new Point3D(0, 0, 0);

      ArrayList<Point3D> points = new ArrayList<>();
      points.add(new Point3D(10.0,10.0,10.0));
      points.add(new Point3D(-10.0,10.0,10.0));
      points.add(new Point3D(-10.0,-10.0,10.0));
      points.add(new Point3D(10.0,-10.0,10.0));
      points.add(new Point3D(10.0,10.0,-10.0));
      points.add(new Point3D(-10.0,10.0,-10.0));
      points.add(new Point3D(-10.0,-10.0,-10.0));
      points.add(new Point3D(10.0,-10.0,-10.0));

      for (; i < 8; ++i, theta += 2 * Math.PI / 8)
      {
         Pose3D pose = new Pose3D();



         Point3D position = new Point3D(radius * Math.cos(theta), radius * Math.sin(theta), 0.0);



      }
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

      //for (int i = 0; i < 4500; i++)
      //{
      //   long start = System.nanoTime();
      //   vo.update();
      //   long end = System.nanoTime();
      //   System.out.println("Time Taken (Update): " + (end - start) / 1000 + "us");
      //}
   }
}
