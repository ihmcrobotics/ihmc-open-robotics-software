package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.MeshDataGeneratorMissing;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXSphericalImageProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXIconTexture imageTexture;
   private ModelInstance modelInstance;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble sphereRadius = new ImDouble(20.0);
   private final ImInt sphereLatitudeVertices = new ImInt(100);
   private final ImInt sphereLongitudeVertices = new ImInt(100);
   private final ImDouble projectionScaleX = new ImDouble(0.02);
   private final ImDouble projectionScaleY = new ImDouble(0.02);
   private Model model;

   public RDXSphericalImageProjectionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            imageTexture = new RDXIconTexture("/images/blackflytest.jpg");

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection", () ->
            {
               ImGuiTools.volatileInputDouble(labels.get("Sphere radius"), sphereRadius);
               ImGuiTools.volatileInputInt(labels.get("Sphere latitude vertices"), sphereLatitudeVertices);
               ImGuiTools.volatileInputInt(labels.get("Sphere longitude vertices"), sphereLongitudeVertices);
               ImGuiTools.sliderDouble(labels.get("Projection scale X"), projectionScaleX, 0.005, 0.05);
               ImGuiTools.sliderDouble(labels.get("Projection scale Y"), projectionScaleY, 0.005, 0.05);
            });
         }

         @Override
         public void render()
         {
            MeshDataHolder sphereMeshDataHolder = MeshDataGeneratorMissing.InvertedSphere(sphereRadius.get(),
                                                                                          sphereLatitudeVertices.get(),
                                                                                          sphereLongitudeVertices.get());

            for (int i = 0; i < sphereMeshDataHolder.getVertices().length; i++)
            {
               Point3D32 vertex = sphereMeshDataHolder.getVertices()[i];
               TexCoord2f texturePoint = sphereMeshDataHolder.getTexturePoints()[i];

               // TODO: Magical function
               double wrappedY = vertex.getY();
               if (vertex.getX() < 0.0)
               {
                  if (vertex.getY() > 0.0)
                     wrappedY += vertex.getY() - sphereRadius.get();
                  else
                     wrappedY -= vertex.getY() - sphereRadius.get();
               }
               texturePoint.setX((projectionScaleX.get() * wrappedY) + 0.5);
               texturePoint.setY((-projectionScaleY.get() * vertex.getZ()) + 0.5);
            }

            ModelBuilder modelBuilder = new ModelBuilder();

            modelBuilder.begin();
            Mesh mesh = RDXMeshDataInterpreter.interpretMeshData(sphereMeshDataHolder);


            MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            Material material = new Material();
            material.set(TextureAttribute.createDiffuse(imageTexture.getTexture()));
            material.set(ColorAttribute.createDiffuse(Color.WHITE));
            modelBuilder.part(meshPart, material);

            if (model != null)
               model.dispose();

            model = modelBuilder.end();

            modelInstance = new ModelInstance(model);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            modelInstance.getRenderables(renderables, pool);
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
      new RDXSphericalImageProjectionDemo();
   }
}
