package us.ihmc.avatar.reachabilityMap;

import java.util.List;

import javafx.application.Platform;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories;

public class ReachabilityMapTools
{
   public static List<VisualDefinition> createBoundingBoxVisuals(Voxel3DGrid voxel3DGrid)
   {
      return createBoundingBoxVisuals(voxel3DGrid.getMinPoint(), voxel3DGrid.getMaxPoint());
   }

   public static List<VisualDefinition> createBoundingBoxVisuals(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      double width = 0.01;
      ColorDefinition color = ColorDefinitions.LightBlue();
      VisualDefinitionFactory boundingBox = new VisualDefinitionFactory();
      FramePoint3D modifiableMin = new FramePoint3D(min);
      modifiableMin.changeFrame(ReferenceFrame.getWorldFrame());
      FramePoint3D modifiableMax = new FramePoint3D(max);
      modifiableMax.changeFrame(ReferenceFrame.getWorldFrame());
      double x0 = modifiableMin.getX();
      double y0 = modifiableMin.getY();
      double z0 = modifiableMin.getZ();
      double x1 = modifiableMax.getX();
      double y1 = modifiableMax.getY();
      double z1 = modifiableMax.getZ();
      // The three segments originating from min
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x1, y0, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x0, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x0, y0, z1, width), color);
      // The three segments originating from min
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x0, y1, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x1, y0, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x1, y1, z0, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y0, z0, x1, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y0, z0, x1, y0, z1, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y1, z0, x1, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y1, z0, x0, y1, z1, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z1, x1, y0, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z1, x0, y1, z1, width), color);

      return boundingBox.getVisualDefinitions();
   }

   public static List<VisualDefinition> createReachibilityColorScaleVisuals()
   {
      VisualDefinitionFactory voxelViz = new VisualDefinitionFactory();
      double maxReachability = 0.7;
      double resolution = 0.1;
      voxelViz.appendTranslation(-1.0, -1.0, 0.0);

      for (double z = 0; z <= maxReachability; z += maxReachability * resolution)
      {
         ColorDefinition color = ColorDefinitions.hsb(z * 360.0, 1.0, 1.0);
         voxelViz.appendTranslation(0.0, 0.0, resolution);
         voxelViz.addSphere(0.025, color);
      }

      return voxelViz.getVisualDefinitions();
   }

   public static void loadVisualizeReachabilityMap(ReachabilityMapRobotInformation robotInformation)
   {
      ReachabilityMapVisualizer visualizer = new ReachabilityMapVisualizer(robotInformation);
      if (visualizer.loadReachabilityMapFromFile())
         visualizer.visualize();
      else
         Platform.exit();
   }
}
