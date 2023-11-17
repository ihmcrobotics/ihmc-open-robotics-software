package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXSphericalImageProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   public RDXSphericalImageProjectionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            RDXIconTexture imageTexture = new RDXIconTexture("/images/blackflytest.jpg");

            MeshDataHolder sphereMeshDataHolder = MeshDataGenerator.Sphere(20.0, 100, 100);

            TexCoord2f[] texturePoints = sphereMeshDataHolder.getTexturePoints();

            for (int i = 0; i < sphereMeshDataHolder.getVertices().length; i++)
            {
               Point3D32 vertex = sphereMeshDataHolder.getVertices()[i];
               TexCoord2f texturePoint = sphereMeshDataHolder.getTexturePoints()[i];

               // TODO: Magical function

            }

            ModelBuilder modelBuilder = new ModelBuilder();

            modelBuilder.begin();
            Mesh mesh = RDXMeshDataInterpreter.interpretMeshData(sphereMeshDataHolder);


            MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            Material material = new Material();
            material.set(TextureAttribute.createDiffuse(imageTexture.getTexture()));
            material.set(ColorAttribute.createDiffuse(Color.WHITE));
            modelBuilder.part(meshPart, material);

            Model model = modelBuilder.end();

            RDXModelInstance modelInstance = new RDXModelInstance(model);
            baseUI.getPrimaryScene().addRenderableProvider(modelInstance);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
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
