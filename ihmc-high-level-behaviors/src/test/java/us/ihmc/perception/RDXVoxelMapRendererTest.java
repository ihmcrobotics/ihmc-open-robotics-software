package us.ihmc.perception;

import com.badlogic.gdx.math.Vector3;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuMapping.VoxelMapExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXVoxelMapRenderer;

import java.util.List;

public class RDXVoxelMapRendererTest
{
   /**
    * Create a fake transformation matrix just for the test
    */
   private static double[] getDoubles()
   {
      double[] transformArray = new double[12];
      // Identity rotation matrix
      transformArray[0] = 1.0;  // R00
      transformArray[1] = 0.0;  // R01
      transformArray[2] = 0.0;  // R02
      transformArray[3] = 1.0;  // Tx

      transformArray[4] = 0.0;  // R10
      transformArray[5] = 1.0;  // R11
      transformArray[6] = 0.0;  // R12
      transformArray[7] = 2.0;  // Ty

      transformArray[8] = 0.0; // R20
      transformArray[9] = 0.0; // R21
      transformArray[10] = 1.0; // R22
      transformArray[11] = 3.0; // Tz
      return transformArray;
   }

   @Test
   public void testRapidVoxelMapExtractor()
   {
      RDXVoxelMapRenderer voxelMapRenderer = new RDXVoxelMapRenderer();
      VoxelMapExtractor voxelMapExtractor = new VoxelMapExtractor();

      // Create fake data for the test
      GpuMat gpuMat = new GpuMat(1280, 720, opencv_core.CV_16UC1);
      gpuMat.setTo(new Scalar(2000));
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      // Need to set these to the size of the image
      cameraIntrinsics.setWidth(1280);
      cameraIntrinsics.setHeight(720);
      double[] transformArray = getDoubles();

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(transformArray);

      voxelMapExtractor.update(gpuMat, cameraIntrinsics, rigidBodyTransform, rigidBodyTransform, rigidBodyTransform, new Point3D(0.0, 0.0, 0.0));

      GpuMat voxelMap = voxelMapExtractor.getVoxelMap();

      List<Vector3> vectors = voxelMapRenderer.extractOccupiedVoxels(voxelMap,
                                                                     voxelMapExtractor.getCellsPerAxis(),
                                                                     voxelMapExtractor.getCellSize(),
                                                                     0.0f,
                                                                     0.0f);

      RDXBaseUI baseUI = new RDXBaseUI();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
                                  {
                                     @Override
                                     public void create()
                                     {
                                        voxelMapRenderer.create(1000);
                                        baseUI.getPrimaryScene().addRenderableProvider(voxelMapRenderer);
                                        baseUI.create();
                                     }

                                     @Override
                                     public void render()
                                     {
                                        voxelMapRenderer.update(vectors, voxelMapExtractor.getCellSize(), voxelMapExtractor.getCellsPerAxis());
                                        baseUI.renderBeforeOnScreenUI();
                                        baseUI.renderEnd();
                                     }

                                     @Override
                                     public void dispose()
                                     {
                                        voxelMapRenderer.dispose();
                                     }
                                  }

      );

      voxelMapExtractor.destroy();
      baseUI.dispose();
   }
}
