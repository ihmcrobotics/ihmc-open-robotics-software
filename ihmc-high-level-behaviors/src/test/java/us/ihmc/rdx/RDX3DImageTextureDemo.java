package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.g3d.utils.TextureProvider;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;

public class RDX3DImageTextureDemo
{
   private final RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();

   public RDX3DImageTextureDemo()
   {
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private int width = 800;
         private int height = 600;

         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);

            ModelBuilder modelBuilder = new ModelBuilder();
            modelBuilder.begin();

            MeshBuilder meshBuilder = new MeshBuilder();
            meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

            // Counter clockwise order
            // Draw so thumb faces away and index right
            Vector3 topLeftPosition = new Vector3(0.0f, 0.0f, 0.0f);
            Vector3 bottomLeftPosition = new Vector3(0.0f, (float) height, 0.0f);
            Vector3 bottomRightPosition = new Vector3((float) width, (float) height, 0.0f);
            Vector3 topRightPosition = new Vector3((float) width, 0.0f, 0.0f);
            Vector3 topLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector3 bottomLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector3 bottomRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector3 topRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector2 topLeftUV = new Vector2(0.0f, 0.0f);
            Vector2 bottomLeftUV = new Vector2(0.0f, 1.0f);
            Vector2 bottomRightUV = new Vector2(1.0f, 1.0f);
            Vector2 topRightUV = new Vector2(1.0f, 0.0f);
            meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topLeftUV);
            meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomLeftUV);
            meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomRightUV);
            meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topRightUV);
            meshBuilder.triangle((short) 3, (short) 0, (short) 1);
            meshBuilder.triangle((short) 1, (short) 2, (short) 3);

            Mesh mesh = meshBuilder.end();

            MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            Material material = new Material();

            TextureProvider.FileTextureProvider fileTextureProvider = new TextureProvider.FileTextureProvider();
            Texture debugImageTexture = fileTextureProvider.load("debugImageTexture.jpg");

            material.set(TextureAttribute.createDiffuse(debugImageTexture));
            material.set(ColorAttribute.createDiffuse(Color.WHITE));
            modelBuilder.part(meshPart, material);

            Model model = modelBuilder.end();
            ModelInstance modelInstance = new ModelInstance(model);
            modelInstance.transform.scale(0.01f, 0.01f, 0.01f);
            sceneManager.addRenderableProvider(modelInstance);
         }

         @Override
         public void render()
         {
            RDX3DSceneTools.glClearGray();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }

         @Override
         public void dispose()
         {
            sceneManager.dispose();
         }
      }, "RDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDX3DImageTextureDemo();
   }
}
