package us.ihmc.avatar.reachabilityMap;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.List;
import java.util.function.IntToDoubleFunction;

import javafx.application.Platform;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
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

   public static VisualDefinition createMetricVisual(Voxel3DData voxel, double scale, double qualityValue)
   {
      FramePoint3D voxelLocationLocal = new FramePoint3D(voxel.getPosition());
      voxelLocationLocal.changeFrame(ReferenceFrame.getWorldFrame());

      ColorDefinition color;
      if (qualityValue == -1.0)
         color = ColorDefinitions.Black();
      else
         color = ColorDefinitions.hsb(0.7 * qualityValue * 360.0, 1, 1);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      materialDefinition.setShininess(10);
      return new VisualDefinition(voxelLocationLocal, new Sphere3DDefinition(scale * voxel.getSize() / 2.0, 16), materialDefinition);
   }

   public static void createVoxelRayHeatmap(Voxel3DData voxel,
                                            double scale,
                                            Point2DReadOnly reachableTextureCoord,
                                            Point2DReadOnly unreachableTextureCoord,
                                            TriangleMesh3DBuilder vizMeshBuilder)
   {
      createVoxelRayHeatmap(voxel,
                            scale,
                            rayIndex -> voxel.isRayReachable(rayIndex) ? 1.0 : 0.0,
                            reachableTextureCoord,
                            unreachableTextureCoord,
                            vizMeshBuilder);
   }

   public static void createVoxelRayHeatmap(Voxel3DData voxel,
                                            double scale,
                                            IntToDoubleFunction rayIndexQualityCalculator,
                                            Point2DReadOnly highQualityTextureCoord,
                                            Point2DReadOnly lowQualityTextureCoord,
                                            TriangleMesh3DBuilder vizMeshBuilder)
   {
      TriangleMesh3DDefinition mesh = TriangleMesh3DFactories.Sphere(scale * voxel.getSize() / 2.0, 8, 8);

      for (int i = 0; i < mesh.getNormals().length; i++)
      {
         Point3D32 vertex = mesh.getVertices()[i];
         vertex.add(voxel.getPosition());
         Vector3D32 normal = mesh.getNormals()[i];
         Point2D32 texture = mesh.getTextures()[i];

         double sumOfWeights = 0.0;
         double reachabilityValue = 0.0;

         for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
         {
            Point3D pointOnVoxelSphere = voxel.getSphereVoxelShape().getPointsOnSphere()[rayIndex];
            double rayDirectionX = pointOnVoxelSphere.getX() / voxel.getSize();
            double rayDirectionY = pointOnVoxelSphere.getY() / voxel.getSize();
            double rayDirectionZ = pointOnVoxelSphere.getZ() / voxel.getSize();

            double weight = TupleTools.dot(rayDirectionX, rayDirectionY, rayDirectionZ, normal);

            if (weight <= 0.0)
            {
               continue;
            }
            else
            {
               double quality = rayIndexQualityCalculator.applyAsDouble(rayIndex);
               sumOfWeights += weight;
               reachabilityValue += weight * quality;
            }
         }

         reachabilityValue /= sumOfWeights;
         texture.interpolate(lowQualityTextureCoord, highQualityTextureCoord, reachabilityValue);
      }

      RigidBodyTransform pose = voxel.getPosition().getReferenceFrame().getTransformToRoot();
      vizMeshBuilder.addTriangleMesh3D(mesh, pose.getTranslation(), pose.getRotation());
   }

   public static TextureDefinition generateReachabilityGradient(double unreachableHue, double reachableHue)
   {
      return new TextureDefinition(createGradientImage(128, 4, unreachableHue, reachableHue));
   }

   private static BufferedImage createGradientImage(int width, int height, double hueStart, double hueEnd)
   {

      BufferedImage gradientImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
      Graphics2D graphics2D = gradientImage.createGraphics();

      for (int x = 0; x < width; x++)
      {
         double hue = EuclidCoreTools.interpolate(hueStart, hueEnd, x / (width - 1.0));
         graphics2D.setColor(Color.getHSBColor((float) hue, 1, 1));
         graphics2D.drawRect(x, 0, 1, height);
      }

      graphics2D.dispose();
      return gradientImage;
   }
}
