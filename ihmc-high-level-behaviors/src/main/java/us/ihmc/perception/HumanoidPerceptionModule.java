package us.ihmc.perception;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class HumanoidPerceptionModule
{
   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private CollisionBoxProvider collisionBoxProvider;
   private FramePlanarRegionsList framePlanarRegionsList;
   private PlanarRegionsList regionsInSensorFrame;
   private PlanarRegionsList regionsInWorldFrame;
   private PlanarRegionMappingHandler mapHandler;

   public HumanoidPerceptionModule(OpenCLManager openCLManager)
   {
      this.mapHandler = new PlanarRegionMappingHandler();
      this.openCLManager = openCLManager;
      this.framePlanarRegionsList = new FramePlanarRegionsList();
   }

   public void update(Mat depthImage, ReferenceFrame cameraFrame, boolean rapidRegions)
   {
      if (rapidRegions)
      {
         BytedecoOpenCVTools.convertFloatToShort(depthImage, bytedecoDepthImage.getBytedecoOpenCVMat(),
                 1000.0,
                 0.0);

         extractFramePlanarRegionsList(cameraFrame);

         if (mapHandler.isEnabled())
         {
            mapHandler.updateMapWithNewRegions(this.framePlanarRegionsList);
         }
      }
   }

   public void initializeRapidPlanarRegionsExtractor(int depthHeight, int depthWidth, CameraIntrinsics cameraIntrinsics)
   {
      this.bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
      this.bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      this.rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
      this.rapidPlanarRegionsExtractor.create(openCLManager, depthHeight, depthWidth, cameraIntrinsics.getFx(), cameraIntrinsics.getFy(), cameraIntrinsics.getCx(),
                                              cameraIntrinsics.getCy());
   }

   public void initializeBodyCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      this.fullRobotModel = fullRobotModel;
      this.collisionBoxProvider = collisionBoxProvider;
      this.collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, collisionBoxProvider);
   }

   public void extractFramePlanarRegionsList(ReferenceFrame cameraFrame)
   {
      this.rapidPlanarRegionsExtractor.update(bytedecoDepthImage, cameraFrame, this.framePlanarRegionsList);
      this.rapidPlanarRegionsExtractor.setProcessing(false);
      this.regionsInSensorFrame = this.framePlanarRegionsList.getPlanarRegionsList();
      this.regionsInWorldFrame = this.regionsInSensorFrame.copy();
      this.regionsInWorldFrame.applyTransform(cameraFrame.getTransformToWorldFrame());
   }

   public void filterFramePlanarRegionsList()
   {
      PerceptionFilterTools.applyCollisionFilter(framePlanarRegionsList, this.collidingScanRegionFilter);
   }

   public FramePlanarRegionsList getFramePlanarRegionsResult()
   {
      return this.framePlanarRegionsList;
   }

   public PlanarRegionsList getRegionsInSensorFrame()
   {
      return this.regionsInSensorFrame;
   }

   public PlanarRegionsList getRegionsInWorldFrame()
   {
      return this.regionsInWorldFrame;
   }

   public BytedecoImage getBytedecoDepthImage()
   {
      return this.bytedecoDepthImage;
   }

   public RapidPlanarRegionsExtractor getRapidRegionsExtractor()
   {
      return this.rapidPlanarRegionsExtractor;
   }

   public PlanarRegionMappingHandler getMapHandler()
   {
      return this.mapHandler;
   }
}
