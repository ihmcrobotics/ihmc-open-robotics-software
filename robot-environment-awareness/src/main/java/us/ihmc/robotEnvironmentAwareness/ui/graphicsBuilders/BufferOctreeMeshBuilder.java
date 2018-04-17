package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;

/**
 * Created by adrien on 11/20/16.
 */
public class BufferOctreeMeshBuilder implements Runnable
{
   private static final Color DEFAULT_BUFFER_COLOR = Color.DARKRED;
   private static final double NODE_SCALE = 0.5;

   private final JavaFXMeshBuilder bufferMeshBuilder = new JavaFXMeshBuilder();
   private final Material bufferMaterial = new PhongMaterial(DEFAULT_BUFFER_COLOR);

   private final AtomicReference<Boolean> showBuffer;
   private final AtomicReference<NormalOcTreeMessage> bufferState;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private boolean hasClearedBufferGraphics = false;
   private final REAUIMessager uiMessager;

   public BufferOctreeMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      showBuffer = uiMessager.createInput(REAModuleAPI.UIOcTreeShowBuffer, false);
      bufferState = uiMessager.createInput(REAModuleAPI.OcTreeBufferState);
   }

   @Override
   public void run()
   {
      NormalOcTreeMessage newBufferState = bufferState.getAndSet(null);

      if (!showBuffer.get())
      {
         if (hasClearedBufferGraphics)
            return;
         bufferMeshBuilder.clear();
         meshAndMaterialToRender.set(new Pair<>(null, null));
         hasClearedBufferGraphics = true;
         return;
      }

      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestBuffer);

      bufferMeshBuilder.clear();

      if (newBufferState == null)
         return;

      UIOcTree uiOcTree = new UIOcTree(newBufferState);

      for (UIOcTreeNode uiOcTreeNode : uiOcTree)
      {
         Point3D nodeCenter = new Point3D(uiOcTreeNode.getX(), uiOcTreeNode.getY(), uiOcTreeNode.getZ());
         bufferMeshBuilder.addTetrahedron(NODE_SCALE * uiOcTreeNode.getSize(), nodeCenter);
      }

      meshAndMaterialToRender.set(new Pair<>(bufferMeshBuilder.generateMesh(), bufferMaterial));
      hasClearedBufferGraphics = false;
   }

   public boolean hasNewMeshAndMaterial()
   {
      return meshAndMaterialToRender.get() != null;
   }

   public Pair<Mesh, Material> pollMeshAndMaterial()
   {
      return meshAndMaterialToRender.getAndSet(null);
   }
}
