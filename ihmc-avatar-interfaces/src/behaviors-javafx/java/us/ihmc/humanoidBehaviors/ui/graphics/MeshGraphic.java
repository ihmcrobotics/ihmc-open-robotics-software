package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class MeshGraphic extends Group
{
   private final Consumer<JavaFXMeshBuilder> meshBuilder;

   private volatile List<Node> nodes;
   private List<Node> lastNodes = null; // optimization

   private Color color = Color.LIGHTGRAY;

   public MeshGraphic(Consumer<JavaFXMeshBuilder> meshBuilder)
   {
      this.meshBuilder = meshBuilder;
   }

   public void generateMeshesAsync()
   {
      ThreadTools.startAThread(() -> generateMeshes(), "MeshGeneration");
   }

   public synchronized void generateMeshes()
   {
      List<Node> updateMeshViews = new ArrayList<>();

      JavaFXMeshBuilder meshHelper = new JavaFXMeshBuilder();

      meshBuilder.accept(meshHelper); // user creates mesh

      MeshView regionMeshView = new MeshView(meshHelper.generateMesh());
      regionMeshView.setMaterial(new PhongMaterial(color));

      updateMeshViews.add(regionMeshView);

      nodes = updateMeshViews; // volatile set
   }

   public void update()
   {
      List<Node> meshViews = nodes;  // volatile get
      if (lastNodes != meshViews) // optimization
      {
         getChildren().clear();
         getChildren().addAll(meshViews);
         lastNodes = meshViews;
      }
   }

   public void setColor(Color color)
   {
      this.color = color;
   }
}
