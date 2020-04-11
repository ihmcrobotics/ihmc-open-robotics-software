package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.shape.MeshView;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraph;

import java.util.HashMap;
import java.util.Map;

public class VisibilityGraphsGraphic extends Group
{
   private PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);

   // navigable extrusions

   private void build(VisibilityGraph visibilityGraph)
   {
//      Map<Integer, JavaFXMeshBuilder> navigableExtrusionsMeshBuilders = new HashMap<>();
//
//      JavaFXMeshBuilder navigableExtrusionsMeshBuilder = getOrCreate(navigableExtrusionsMeshBuilders, regionId);
//
//      HashMap<Integer, MeshView> navigableExtrusionsMapToRender = new HashMap<>();
//
//      for (Integer id : rawPointsMeshBuilders.keySet())
//      {
//         MeshView navigableExtrusionsMeshView = new MeshView(navigableExtrusionsMeshBuilders.get(id).generateMesh());
//         navigableExtrusionsMeshView.setMaterial(navigableMaterials.get(id));
//         navigableExtrusionsMapToRender.put(id, navigableExtrusionsMeshView);
//      }
   }

   private JavaFXMeshBuilder getOrCreate(Map<Integer, JavaFXMeshBuilder> meshBuilders, int regionId)
   {
      JavaFXMeshBuilder meshBuilder = meshBuilders.get(regionId);
      if (meshBuilder == null)
      {
         meshBuilder = new JavaFXMeshBuilder();
         meshBuilders.put(regionId, meshBuilder);
      }
      return meshBuilder;
   }

   private void handle(long now)
   {

   }
}
