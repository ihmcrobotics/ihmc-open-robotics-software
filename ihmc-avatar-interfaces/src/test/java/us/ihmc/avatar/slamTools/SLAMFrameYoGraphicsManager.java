package us.ihmc.avatar.slamTools;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePose3D;

public class SLAMFrameYoGraphicsManager
{
   private final SLAMFrame slamFrame;
   private final int sizeOfPointCloud;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePose3D yoFrameSensorPose;
   private final YoFramePoint3D[] yoFramePointCloud;

   private final YoGraphicCoordinateSystem yoGraphicSensorPose;
   private final YoGraphicPosition[] yoGraphicPointCloud;

   public SLAMFrameYoGraphicsManager(String prefix, SLAMFrame frame, AppearanceDefinition appearance, YoVariableRegistry registry,
                                     YoGraphicsListRegistry graphicsRegistry)
   {
      slamFrame = frame;
      sizeOfPointCloud = slamFrame.getPointCloud().length;

      yoFrameSensorPose = new YoFramePose3D(prefix + "_SensorPoseFrame", worldFrame, registry);
      yoFramePointCloud = new YoFramePoint3D[sizeOfPointCloud];
      for (int i = 0; i < sizeOfPointCloud; i++)
      {
         yoFramePointCloud[i] = new YoFramePoint3D(prefix + "_PointCloud_" + i, worldFrame, registry);
      }

      YoGraphicsList yoGraphicListRegistry = new YoGraphicsList(prefix + "_SLAM_Frame_Viz");
      yoGraphicSensorPose = new YoGraphicCoordinateSystem(prefix + "_SensorPoseViz", yoFrameSensorPose, 0.1, appearance);
      yoGraphicListRegistry.add(yoGraphicSensorPose);
      yoGraphicPointCloud = new YoGraphicPosition[sizeOfPointCloud];
      for (int i = 0; i < sizeOfPointCloud; i++)
      {
         yoGraphicPointCloud[i] = new YoGraphicPosition(prefix + "_PointCloudViz_" + i, yoFramePointCloud[i], 0.003, appearance);
         yoGraphicListRegistry.add(yoGraphicPointCloud[i]);
      }

      graphicsRegistry.registerYoGraphicsList(yoGraphicListRegistry);
   }

   public void updateGraphics()
   {
      yoFrameSensorPose.set(slamFrame.getSensorPose());
      Point3DReadOnly[] pointCloud = slamFrame.getPointCloud();
      for (int i = 0; i < sizeOfPointCloud; i++)
      {
         yoFramePointCloud[i].set(pointCloud[i]);
      }
   }
}
