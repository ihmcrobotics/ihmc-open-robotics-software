package us.ihmc.humanoidBehaviors.ui.mapping.visualizer;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;

public class NormalOctreeGraphic extends Group
{
   private volatile List<Node> messageNodes;
   private List<Node> lastNodes = null;
   private volatile List<Node> updatePointCloudMeshViews;

   private static final float SCAN_POINT_SIZE = 0.01f;
   private static final int palleteSizeForMeshBuilder = 2048;
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(palleteSizeForMeshBuilder));

   private final RecyclingArrayList<Point3D> plane = new RecyclingArrayList<>(0, SupplierBuilder.createFromEmptyConstructor(Point3D.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   public void initialize()
   {
      updatePointCloudMeshViews = new ArrayList<>();
      meshBuilder.clear();
   }

   public void generateMeshes()
   {
      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());

      updatePointCloudMeshViews.add(scanMeshView);
      meshBuilder.clear();
      messageNodes = updatePointCloudMeshViews;
   }

   public void addMesh(List<Plane3D> planes, double octreeResolution, Color colorToViz)
   {
      for (Plane3D singlePlane : planes)
      {
         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();

         planeNormal.set(singlePlane.getNormal());
         pointOnPlane.set(singlePlane.getPoint());

         intersectionPlaneBoxCalculator.setCube(octreeResolution, pointOnPlane.getX(), pointOnPlane.getY(), pointOnPlane.getZ());
         intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
         intersectionPlaneBoxCalculator.computeIntersections(plane);

         if (plane.size() < 3)
            continue;

         int numberOfTriangles = plane.size() - 2;
         int[] triangleIndices = new int[3 * numberOfTriangles];
         int index = 0;
         for (int j = 2; j < plane.size(); j++)
         {
            triangleIndices[index++] = 0;
            triangleIndices[index++] = j - 1;
            triangleIndices[index++] = j;
         }

         Point3D32[] vertices = new Point3D32[plane.size()];
         TexCoord2f[] texCoords = new TexCoord2f[plane.size()];
         Vector3D32[] normals = new Vector3D32[plane.size()];

         for (int j = 0; j < plane.size(); j++)
         {
            vertices[j] = new Point3D32(plane.get(j));
            texCoords[j] = new TexCoord2f();
            normals[j] = new Vector3D32(planeNormal);
         }

         MeshDataHolder octreePlaneMeshDataHolder = new MeshDataHolder(vertices, texCoords, triangleIndices, normals);
         meshBuilder.addMesh(octreePlaneMeshDataHolder, colorToViz);
      }
   }

   public void addMesh(NormalOcTree octree, double octreeResolution, Color colorToViz, boolean vizWithDot)
   {
      NormalOcTreeMessage normalOctreeMessage = OcTreeMessageConverter.convertToMessage(octree);
      UIOcTree octreeForViz = new UIOcTree(normalOctreeMessage);

      for (UIOcTreeNode uiOcTreeNode : octreeForViz)
      {
         if (!uiOcTreeNode.isNormalSet() || !uiOcTreeNode.isHitLocationSet())
            continue;

         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();
         double size = uiOcTreeNode.getSize();

         uiOcTreeNode.getNormal(planeNormal);
         uiOcTreeNode.getHitLocation(pointOnPlane);

         intersectionPlaneBoxCalculator.setCube(size, uiOcTreeNode.getX(), uiOcTreeNode.getY(), uiOcTreeNode.getZ());
         intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
         intersectionPlaneBoxCalculator.computeIntersections(plane);

         if (plane.size() < 3)
            continue;

         int numberOfTriangles = plane.size() - 2;
         int[] triangleIndices = new int[3 * numberOfTriangles];
         int index = 0;
         for (int j = 2; j < plane.size(); j++)
         {
            triangleIndices[index++] = 0;
            triangleIndices[index++] = j - 1;
            triangleIndices[index++] = j;
         }

         Point3D32[] vertices = new Point3D32[plane.size()];
         TexCoord2f[] texCoords = new TexCoord2f[plane.size()];
         Vector3D32[] normals = new Vector3D32[plane.size()];

         for (int j = 0; j < plane.size(); j++)
         {
            vertices[j] = new Point3D32(plane.get(j));
            texCoords[j] = new TexCoord2f();
            normals[j] = new Vector3D32(planeNormal);
         }

         if (vizWithDot)
         {
            meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), pointOnPlane, colorToViz);
         }
         else
         {
            MeshDataHolder octreePlaneMeshDataHolder = new MeshDataHolder(vertices, texCoords, triangleIndices, normals);
            meshBuilder.addMesh(octreePlaneMeshDataHolder, colorToViz);
         }
      }
   }

   public void update()
   {
      List<Node> meshViews = messageNodes;
      if (lastNodes != meshViews)
      {
         getChildren().clear();
         getChildren().addAll(meshViews);
         lastNodes = meshViews;
      }
   }
}
