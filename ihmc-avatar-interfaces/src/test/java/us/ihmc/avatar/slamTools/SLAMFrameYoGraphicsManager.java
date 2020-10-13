package us.ihmc.avatar.slamTools;

import java.util.List;
import java.util.Random;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SLAMFrameYoGraphicsManager
{
   private final SLAMFrame slamFrame;
   private final int sizeOfPointCloud;
   private final TIntArrayList indicesArray = new TIntArrayList();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePose3D yoFrameSensorPose;
   private final YoGraphicCoordinateSystem yoGraphicSensorPose;

   private final YoFramePoint3D[] yoFramePointCloud;
   private final YoGraphicPosition[] yoGraphicPointCloud;

   private final YoFramePoint3D[] yoFrameSurfelPoints;
   private final YoGraphicPosition[] yoGraphicSurfelPoints;

   private final boolean visualizeSurfel;
   private static final double NORMAL_VIZ_LENGTH = 0.03;

   private final YoFrameVector3D[] yoFrameSurfelNormals;
   private final YoGraphicVector[] yoGraphicSurfelNormals;

   public SLAMFrameYoGraphicsManager(String prefix, SLAMFrame frame, int size, AppearanceDefinition appearance, AppearanceDefinition surfelAppearance,
                                     YoRegistry registry, YoGraphicsListRegistry graphicsRegistry, boolean visualizeSurfel)
   {
      slamFrame = frame;
      this.visualizeSurfel = visualizeSurfel;

      yoFrameSensorPose = new YoFramePose3D(prefix + "_SensorPoseFrame", worldFrame, registry);
      YoGraphicsList yoGraphicListRegistry = new YoGraphicsList(prefix + "_SLAM_Frame_Viz");
      yoGraphicSensorPose = new YoGraphicCoordinateSystem(prefix + "_SensorPoseViz", yoFrameSensorPose, 0.1, appearance);
      yoGraphicListRegistry.add(yoGraphicSensorPose);

      if (size < 0 || slamFrame.getCorrectedPointCloudInWorld().size() < size)
      {
         sizeOfPointCloud = slamFrame.getCorrectedPointCloudInWorld().size();
         for (int i = 0; i < slamFrame.getCorrectedPointCloudInWorld().size(); i++)
         {
            indicesArray.add(i);
         }
      }
      else
      {
         Random selector = new Random(0612L);
         sizeOfPointCloud = size;
         while (indicesArray.size() != sizeOfPointCloud)
         {
            int selectedIndex = selector.nextInt(slamFrame.getCorrectedPointCloudInWorld().size());
            if (!indicesArray.contains(selectedIndex))
            {
               indicesArray.add(selectedIndex);
            }
         }
      }

      yoFramePointCloud = new YoFramePoint3D[sizeOfPointCloud];
      yoGraphicPointCloud = new YoGraphicPosition[sizeOfPointCloud];
      for (int i = 0; i < sizeOfPointCloud; i++)
      {
         yoFramePointCloud[i] = new YoFramePoint3D(prefix + "_PointCloud_" + i, worldFrame, registry);
         yoGraphicPointCloud[i] = new YoGraphicPosition(prefix + "_PointCloudViz_" + i, yoFramePointCloud[i], 0.003, appearance);
         yoGraphicListRegistry.add(yoGraphicPointCloud[i]);
      }

      List<Plane3D> surfaceElements = frame.getSurfaceElements();
      int numberOfSurfel = surfaceElements.size();

      yoFrameSurfelPoints = new YoFramePoint3D[numberOfSurfel];
      yoGraphicSurfelPoints = new YoGraphicPosition[numberOfSurfel];

      yoFrameSurfelNormals = new YoFrameVector3D[numberOfSurfel];
      yoGraphicSurfelNormals = new YoGraphicVector[numberOfSurfel];
      for (int i = 0; i < numberOfSurfel; i++)
      {
         yoFrameSurfelPoints[i] = new YoFramePoint3D(prefix + "_SurfelPoint_" + i, worldFrame, registry);
         yoGraphicSurfelPoints[i] = new YoGraphicPosition(prefix + "_SurfelPointViz_" + i, yoFrameSurfelPoints[i], 0.003, surfelAppearance);

         yoFrameSurfelNormals[i] = new YoFrameVector3D(prefix + "_SurfelNormal_" + i, worldFrame, registry);
         yoGraphicSurfelNormals[i] = new YoGraphicVector(prefix + "_SurfelNormalViz_"
               + i, yoFrameSurfelPoints[i], yoFrameSurfelNormals[i], NORMAL_VIZ_LENGTH, surfelAppearance, false);

         yoGraphicListRegistry.add(yoGraphicSurfelPoints[i]);
         yoGraphicListRegistry.add(yoGraphicSurfelNormals[i]);
      }

      graphicsRegistry.registerYoGraphicsList(yoGraphicListRegistry);
   }

   public SLAMFrameYoGraphicsManager(String prefix, SLAMFrame frame, int size, AppearanceDefinition appearance, YoRegistry registry,
                                     YoGraphicsListRegistry graphicsRegistry)
   {
      this(prefix, frame, size, appearance, appearance, registry, graphicsRegistry, false);
   }

   public void updateGraphics()
   {
      yoFrameSensorPose.set(slamFrame.getCorrectedSensorPoseInWorld());
      List<? extends Point3DReadOnly> pointCloud = slamFrame.getCorrectedPointCloudInWorld();
      for (int i = 0; i < sizeOfPointCloud; i++)
      {
         yoFramePointCloud[i].set(pointCloud.get(indicesArray.get(i)));
      }

      if (visualizeSurfel)
      {
         List<Plane3D> surfaceElements = slamFrame.getSurfaceElements();
         for (int i = 0; i < surfaceElements.size(); i++)
         {
            yoFrameSurfelPoints[i].set(surfaceElements.get(i).getPoint());
            yoFrameSurfelNormals[i].set(surfaceElements.get(i).getNormal());
         }
      }
   }

   public void hide()
   {
      yoFrameSensorPose.setToNaN();
      for (int i = 0; i < sizeOfPointCloud; i++)
      {
         yoFramePointCloud[i].setToNaN();
      }
      for (int i = 0; i < slamFrame.getSurfaceElements().size(); i++)
      {
         yoFrameSurfelPoints[i].setToNaN();
         yoFrameSurfelNormals[i].setToNaN();
      }
   }
}