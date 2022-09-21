package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImFloat;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXModelScalingDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-graphics/src/libgdx-test/resources",
                                                              "Scaling Demo");
   private ModelInstance couchModelInstance;
   private final ImFloat scale = new ImFloat(1.0f);
   private boolean changed = true;

   public GDXModelScalingDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            Model couchModel = GDXModelLoader.load("Couch/Couch.g3dj");
            couchModelInstance = new GDXModelInstance(couchModel);
            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-5.0, 5.0, 3.0);

            baseUI.getImGuiPanelManager().addPanel("Scaling", this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            // call update() methods here

            if (changed)
            {
               ModelData couchModelData = GDXModelLoader.loadModelData("Couch/Couch.g3dj");
               for (int i = 0; i < couchModelData.meshes.size; i++)
               {
                  ModelMesh modelMesh = couchModelData.meshes.get(i);

                  int floatsPerVertex = GDXTools.calculateFloatsPerVertex(modelMesh);
                  int numberOfVertices = modelMesh.vertices.length / floatsPerVertex;
                  // Each vertex is 8 floats: x,y,z,nx,ny,nz,u,v

                  for (int j = 0; j < numberOfVertices; j++)
                  {
                     float x = modelMesh.vertices[floatsPerVertex * j];
                     float y = modelMesh.vertices[floatsPerVertex * j + 1];
                     float z = modelMesh.vertices[floatsPerVertex * j + 2];

                     modelMesh.vertices[floatsPerVertex * j] *= scale.get();
                     modelMesh.vertices[floatsPerVertex * j + 1] *= scale.get();
                     modelMesh.vertices[floatsPerVertex * j + 2] *= scale.get();
                  }
               }
               Model scaledCouchModel = new Model(couchModelData);
               couchModelInstance = new ModelInstance(scaledCouchModel);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            changed = ImGuiTools.volatileInputFloat("scale", scale);
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            couchModelInstance.getRenderables(renderables, pool);
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
      new GDXModelScalingDemo();
   }
}
