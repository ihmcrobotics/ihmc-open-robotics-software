package us.ihmc.robotEnvironmentAwareness.slam.viewer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;

/**
 * this will hold IhmcSLAMFrame and very original point cloud.
 * @author inhol
 *
 */
public class IhmcSLAMFrameViewer implements Runnable
{
   private final Group root = new Group();
   private final ObservableList<Node> children = root.getChildren();

   private volatile List<Node> updateMeshViews;

   private final AtomicReference<NormalOcTreeMessage> ocTreeState;

   private static final float OCTREE_POINT_SIZE = 0.05f;
   private static final int palleteSizeForMeshBuilder = 2048;
   private final JavaFXMultiColorMeshBuilder meshBuilder;

   public IhmcSLAMFrameViewer(REAUIMessager uiMessager)
   {
      ocTreeState = uiMessager.createInput(REAModuleAPI.SLAMOctreeMapState);

      updateMeshViews = new ArrayList<>();

      meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(palleteSizeForMeshBuilder));
   }

   public Node getRoot()
   {
      return root;
   }

   public void render()
   {
      if (updateMeshViews.size() == 0)
         return;

      children.clear();
      children.addAll(updateMeshViews);
   }

   @Override
   public void run()
   {
      NormalOcTreeMessage newMessage = ocTreeState.getAndSet(null);

      if (newMessage == null)
         return;

      System.out.println("Octree is here!!!");

      UIOcTree octreeForViz = new UIOcTree(newMessage);

      Random random = new Random(0612L);

      for (UIOcTreeNode uiOcTreeNode : octreeForViz)
      {
         if (!uiOcTreeNode.isNormalSet() || !uiOcTreeNode.isHitLocationSet())
            continue;

         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();

         uiOcTreeNode.getNormal(planeNormal);
         uiOcTreeNode.getHitLocation(pointOnPlane);

         if(random.nextBoolean() && random.nextBoolean() && random.nextBoolean())
            meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(OCTREE_POINT_SIZE), pointOnPlane, Color.BLUE);
      }

      MeshView scanMeshView = new MeshView(meshBuilder.generateMesh());
      scanMeshView.setMaterial(meshBuilder.generateMaterial());
      updateMeshViews.add(scanMeshView);
   }

}
