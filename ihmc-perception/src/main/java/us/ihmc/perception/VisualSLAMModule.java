package us.ihmc.perception;

import us.ihmc.bytedeco.mapsenseWrapper.VisualOdometry;
import us.ihmc.bytedeco.slamWrapper.SlamWrapper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.elements.CameraModel;

import java.util.ArrayList;

public class VisualSLAMModule
{
   private final VisualOdometry.VisualOdometryExternal visualOdometryExternal;
   private final SlamWrapper.FactorGraphExternal factorGraphExternal;

   private int frameIndex = 0;
   private boolean initialized = false;

   public VisualSLAMModule()
   {
      BytedecoTools.loadMapsenseLibraries();
      visualOdometryExternal = new VisualOdometry.VisualOdometryExternal();

      BytedecoTools.loadGTSAMLibraries();
      factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      factorGraphExternal.addPriorPoseFactor(1, poseInitial);
      factorGraphExternal.setPoseInitialValue(1, poseInitial);
   }


   public void update(ImageMat leftImage, ImageMat rightImage)
   {
      //visualOdometryExternal.displayMat(leftImage.getData(), leftImage.getRows(), leftImage.getCols(), 1);

      visualOdometryExternal.updateStereo(leftImage.getData(), rightImage.getData(), leftImage.getRows(), leftImage.getCols());
      //visualOdometryExternal.getKeyframe();

      LogTools.info("Inserting: {}", frameIndex+1);

      VisualOdometry.KeyframeExternal[] keyframes;

      VisualOdometry.LandmarkExternal[] landmarks;

      float[] odometry = new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
      factorGraphExternal.addOdometryFactor(odometry, frameIndex + 2);

      float[] poseValue = new float[]{0.0f, 0.0f, 0.0f, 1.0f * frameIndex, 0.0f, 0.0f};
      factorGraphExternal.setPoseInitialValue(frameIndex + 2, poseValue);

      factorGraphExternal.optimize();

      factorGraphExternal.printResults();

      frameIndex++;
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

         CameraModel camera = new CameraModel(50.0f,50.0f,50.0f,50.0f, pose);

         Point3D position = new Point3D(radius * Math.cos(theta), radius * Math.sin(theta), 0.0);

         Point2D measurement = camera.project(points.get(i));

         LogTools.info("Projection: {}", measurement);


      }
   }

   public void render()
   {

   }

}
