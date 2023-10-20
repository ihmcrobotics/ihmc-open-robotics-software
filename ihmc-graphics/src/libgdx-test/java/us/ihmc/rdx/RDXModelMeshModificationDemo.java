package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.rdx.mesh.MeshDataBuilderMissingTools;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXModelMeshModificationDemo
{
   public RDXModelMeshModificationDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private static final double SIZE_MEDIAN = 0.2;

         private final Point3D[] vertices3D = new Point3D[8];
         private Model boxModel;
         private ModelInstance boxModelInstance;
         private final MeshDataBuilder meshDataBuilder = new MeshDataBuilder();
         private final Stopwatch stopwatch = new Stopwatch().start();
         private double size = SIZE_MEDIAN;

         @Override
         public void create()
         {
            baseUI.create();

            updateVertices();

            boxModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices3D, 0.05, Color.GRAY));
            boxModelInstance = new ModelInstance(boxModel);
            Mesh mesh = boxModelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;

            baseUI.getImGuiPanelManager().addPanel("Mesh", () ->
            {
               ImGui.text("Number of vertices: %d".formatted(mesh.getNumVertices()));
               ImGui.text("Number of indices: %d".formatted(mesh.getNumIndices()));
            });

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(boxModelInstance);
         }

         @Override
         public void render()
         {
            size = SIZE_MEDIAN + SIZE_MEDIAN * 0.1 * Math.sin(stopwatch.totalElapsed());
            updateVertices();

            Mesh mesh = boxModelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;

            meshDataBuilder.clear();
            MeshDataBuilderMissingTools.addMultiLineBox(vertices3D, 0.05, meshDataBuilder);
            MeshDataHolder meshDataHolder = meshDataBuilder.generateMeshDataHolder();

            RDXMeshDataInterpreter.repositionMeshVertices(meshDataHolder, mesh, Color.GRAY);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void updateVertices()
         {
            vertices3D[0] = new Point3D(size, size, size);
            vertices3D[1] = new Point3D(size, size, -size);
            vertices3D[2] = new Point3D(size, -size, size);
            vertices3D[3] = new Point3D(size, -size, -size);
            vertices3D[4] = new Point3D(-size, size, size);
            vertices3D[5] = new Point3D(-size, size, -size);
            vertices3D[6] = new Point3D(-size, -size, size);
            vertices3D[7] = new Point3D(-size, -size, -size);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXModelMeshModificationDemo();
   }
}
