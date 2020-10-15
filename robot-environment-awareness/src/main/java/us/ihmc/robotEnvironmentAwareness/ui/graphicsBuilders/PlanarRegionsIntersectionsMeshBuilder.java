package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3DMessage;

public class PlanarRegionsIntersectionsMeshBuilder implements Runnable
{
   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;
   private final AtomicReference<Boolean> clearOcTree;

   private final AtomicReference<LineSegment3DMessage[]> intersectionsMessage;

   private final JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
   private final Material material = new PhongMaterial(Color.RED);
   private final REAUIMessager uiMessager;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private final Topic<Boolean> requestPlanarRegionsIntersectionsTopic;

   public PlanarRegionsIntersectionsMeshBuilder(REAUIMessager uiMessager)
   {
      this(uiMessager, REAModuleAPI.OcTreeEnable, REAModuleAPI.OcTreeClear, REAModuleAPI.RequestPlanarRegionsIntersections, REAModuleAPI.PlanarRegionsIntersectionState);
   }

   public PlanarRegionsIntersectionsMeshBuilder(REAUIMessager uiMessager,
                                                Topic<Boolean> ocTreeEnableTopic,
                                                Topic<Boolean> ocTreeClearTopic,
                                                Topic<Boolean> requestPlanarRegionsIntersectionsTopic,
                                                Topic<LineSegment3DMessage[]> planarRegionsIntersectionStateTopic)
   {
      this.uiMessager = uiMessager;
      this.requestPlanarRegionsIntersectionsTopic = requestPlanarRegionsIntersectionsTopic;
      enable = uiMessager.createInput(ocTreeEnableTopic, false);
      clear = uiMessager.createInput(ocTreeClearTopic, false);
      clearOcTree = uiMessager.createInput(ocTreeClearTopic, false);

      intersectionsMessage = uiMessager.createInput(planarRegionsIntersectionStateTopic);
   }

   @Override
   public void run()
   {
      LineSegment3DMessage[] newMessage = intersectionsMessage.getAndSet(null);

      // Reset both clears by using only one pipe
      if (clearOcTree.getAndSet(false) | clear.getAndSet(false))
      {
         meshAndMaterialToRender.set(new Pair<>(null, null));
         return;
      }

      if (!enable.get())
         return;

      uiMessager.submitStateRequestToModule(requestPlanarRegionsIntersectionsTopic);

      if (newMessage == null)
         return;

      meshBuilder.clear();

      for (LineSegment3DMessage intersection : newMessage)
      {
         Point3D32 start = intersection.getStart();
         Point3D32 end = intersection.getEnd();
         float lineWidth = 0.01f;
         meshBuilder.addLine(start, end, lineWidth);
      }

      meshAndMaterialToRender.set(new Pair<Mesh, Material>(meshBuilder.generateMesh(), material));
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
