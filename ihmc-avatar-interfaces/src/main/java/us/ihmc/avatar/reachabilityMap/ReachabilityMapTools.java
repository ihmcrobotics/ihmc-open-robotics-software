package us.ihmc.avatar.reachabilityMap;

import java.util.List;

import javafx.application.Platform;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DBuilder;
import us.ihmc.scs2.definition.visual.TriangleMesh3DFactories;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

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

   public static VisualDefinition createPositionReachabilityVisual(Voxel3DData voxel, double scale, boolean reachable)
   {
      FramePoint3D voxelLocationLocal = new FramePoint3D(voxel.getPosition());
      voxelLocationLocal.changeFrame(ReferenceFrame.getWorldFrame());

      ColorDefinition color;
      if (reachable)
         color = ColorDefinitions.Chartreuse();
      else
         color = ColorDefinitions.DarkRed();
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      materialDefinition.setShininess(10);
      return new VisualDefinition(voxelLocationLocal, new Sphere3DDefinition(scale * voxel.getSize() / 2.0, 16), materialDefinition);
   }

   public static VisualDefinition createRReachabilityVisual(Voxel3DData voxel, double scale, double reachabilityValue)
   {
      FramePoint3D voxelLocationLocal = new FramePoint3D(voxel.getPosition());
      voxelLocationLocal.changeFrame(ReferenceFrame.getWorldFrame());

      ColorDefinition color;
      if (reachabilityValue == -1.0)
         color = ColorDefinitions.Black();
      else
         color = ColorDefinitions.hsb(0.7 * reachabilityValue * 360.0, 1, 1);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      materialDefinition.setShininess(10);
      return new VisualDefinition(voxelLocationLocal, new Sphere3DDefinition(scale * voxel.getSize() / 2.0, 16), materialDefinition);
   }

   public static void createRReachabilityVisualStyle2(Voxel3DData voxel,
                                                      double scale,
                                                      TriangleMesh3DBuilder reachableMeshBuilder,
                                                      TriangleMesh3DBuilder unreachableMeshBuilder)
   {
      RigidBodyTransform transform = voxel.getPosition().getReferenceFrame().getTransformToRoot();

      Point3D rayStart = new Point3D(voxel.getPosition());
      transform.transform(rayStart);
      Point3D rayEnd = new Point3D();
      Point3D[] pointsOnSphere = voxel.getSphereVoxelShape().getPointsOnSphere();
      double lineWidth = scale * voxel.getSize() / 3.0;

      for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
      {
         rayEnd.set(pointsOnSphere[rayIndex]);
         rayEnd.scale(scale);
         rayEnd.add(voxel.getPosition());
         transform.transform(rayEnd);

         if (voxel.isRayReachable(rayIndex))
            reachableMeshBuilder.addLine(rayStart, rayEnd, lineWidth);
         else
            unreachableMeshBuilder.addLine(rayStart, rayEnd, lineWidth);
      }
   }
}
