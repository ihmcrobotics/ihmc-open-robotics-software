package us.ihmc.rdx.simulation;

import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.rdx.DepthSensorDemoObjectsModel;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXSensorSimulator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

import java.util.ArrayList;
import java.util.List;

public class RDXSensorSimulatorDemo
{
   private static final int WIDTH = 1280;
   private static final int HEIGHT = 720;
   private static final float FOV = 70.0f;
   private static final float MIN_RANGE = 0.2f;
   private static final float MAX_RANGE = 20.0f;

   private final Throttler throttler = new Throttler().setFrequency(30.0);
   private final RDXSensorSimulator sensorSimulator;
   private RDXPose3DGizmo sensorPoseGizmo;

   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor();
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private List<Point3D32> pointCloud = new ArrayList<>();

   public RDXSensorSimulatorDemo()
   {
      sensorSimulator = new RDXSensorSimulator(WIDTH, HEIGHT, FOV, MIN_RANGE, MAX_RANGE);

      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new DepthSensorDemoObjectsModel().newInstance(), RDXSceneLevel.GROUND_TRUTH);

            sensorSimulator.create(baseUI.getPrimaryScene());
            sensorSimulator.enableColor(true);
            sensorSimulator.enableDepth(true);

            sensorPoseGizmo = new RDXPose3DGizmo();
            sensorPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);

            pointCloudRenderer.create(WIDTH * HEIGHT);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
         }

         @Override
         public void render()
         {
            if (throttler.run())
            {
               sensorSimulator.grab(sensorPoseGizmo.getTransformToParent());

               RawImage colorImage = sensorSimulator.getColorImage();
               RawImage depthImage = sensorSimulator.getDepthImage();

               pointCloud = pointCloudExtractor.extractPointCloud(depthImage);
               pointCloudRenderer.setPointsToRender(pointCloud);

               colorImage.release();
               depthImage.release();
            }

            pointCloudRenderer.updateMesh();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSensorSimulatorDemo();
   }
}
