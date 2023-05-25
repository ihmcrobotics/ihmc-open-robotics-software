package us.ihmc.perception.modules;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;

public class TerrainPerceptionModule
{
   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private CollisionBoxProvider collisionBoxProvider;
   private FramePlanarRegionsList framePlanarRegionsList;

   public TerrainPerceptionModule(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
      this.framePlanarRegionsList = new FramePlanarRegionsList();
   }

   public void initializeRapidPlanarRegionsExtractor(int depthHeight, int depthWidth, int fx, int fy, int cx, int cy)
   {
      this.bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
      this.bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      this.rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
      this.rapidPlanarRegionsExtractor.create(openCLManager, depthHeight, depthWidth, fx, fy, cx, cy);
   }

   public void initializeBodyCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      this.fullRobotModel = fullRobotModel;
      this.collisionBoxProvider = collisionBoxProvider;
      this.collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, collisionBoxProvider);
   }

   public void extractFramePlanarRegionsList(BytedecoImage depthImage, ReferenceFrame cameraFrame)
   {
      this.rapidPlanarRegionsExtractor.update(depthImage, cameraFrame, this.framePlanarRegionsList);
      this.rapidPlanarRegionsExtractor.setProcessing(false);
   }

   public FramePlanarRegionsList getExtractedPlanarRegions()
   {
      return this.framePlanarRegionsList;
   }

}
