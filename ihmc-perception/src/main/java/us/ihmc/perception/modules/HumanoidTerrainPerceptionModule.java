package us.ihmc.perception.modules;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class HumanoidTerrainPerceptionModule
{
   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private CollisionBoxProvider collisionBoxProvider;

   public HumanoidTerrainPerceptionModule(OpenCLManager openCLManager, int depthHeight, int depthWidth, int fx, int fy, int cx, int cy)
   {
      this.openCLManager = openCLManager;

      bytedecoDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
      bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
      rapidPlanarRegionsExtractor.create(openCLManager,
                                         depthHeight, depthWidth, fx, fy, cx, cy);
   }

   public void initializeCollisionFilter()
   {
      collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, collisionBoxProvider);
   }

}
