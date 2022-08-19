package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;
import static com.badlogic.gdx.graphics.VertexAttributes.Usage.TextureCoordinates;

public class GDX3DSituatedImagePanel
{
   private ModelInstance modelInstance;

   public void create(Texture texture, Vector3[] points)
   {
      create(texture, Double.NaN, Double.NaN, points, true);
   }

   public void create(Texture texture, double panelWidth, double panelHeight)
   {
      create(texture, panelWidth, panelHeight, null, false);
   }

   private void create(Texture texture, double panelWidth, double panelHeight, Vector3[] points, boolean usePoints)
   {
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      float halfPanelHeight = (float) panelHeight / 2.0f;
      float halfPanelWidth = (float) panelWidth / 2.0f;

      // Counter clockwise order
      // Draw so thumb faces away and index right
      Vector3 topLeftPosition;
      Vector3 bottomLeftPosition;
      Vector3 bottomRightPosition;
      Vector3 topRightPosition;
      if (usePoints)
      {
         topLeftPosition = points[0];
         bottomLeftPosition = points[1];
         bottomRightPosition = points[2];
         topRightPosition = points[3];
      }
      else
      {
         topLeftPosition = new Vector3(halfPanelHeight, halfPanelWidth, 0.0f);
         bottomLeftPosition = new Vector3(-halfPanelHeight, halfPanelWidth, 0.0f);
         bottomRightPosition = new Vector3(-halfPanelHeight, -halfPanelWidth, 0.0f);
         topRightPosition = new Vector3(halfPanelHeight, -halfPanelWidth, 0.0f);
      }
      Vector3 topLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 bottomLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 bottomRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 topRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector2 topLeftUV = new Vector2(0.0f, 1.0f);
      Vector2 bottomLeftUV = new Vector2(0.0f, 0.0f);
      Vector2 bottomRightUV = new Vector2(1.0f, 0.0f);
      Vector2 topRightUV = new Vector2(1.0f, 1.0f);
      meshBuilder.vertex(topLeftPosition, topLeftNormal, com.badlogic.gdx.graphics.Color.WHITE, topLeftUV);
      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, com.badlogic.gdx.graphics.Color.WHITE, bottomLeftUV);
      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, com.badlogic.gdx.graphics.Color.WHITE, bottomRightUV);
      meshBuilder.vertex(topRightPosition, topRightNormal, com.badlogic.gdx.graphics.Color.WHITE, topRightUV);
      meshBuilder.triangle((short) 3, (short) 0, (short) 1);
      meshBuilder.triangle((short) 1, (short) 2, (short) 3);
      Mesh mesh = meshBuilder.end();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();

      material.set(TextureAttribute.createDiffuse(texture));
      material.set(ColorAttribute.createDiffuse(new Color(0.68235f, 0.688235f, 0.688235f, 1.0f)));
      modelBuilder.part(meshPart, material);

      // TODO: Rebuild the model if the camera parameters change.
      Model model = modelBuilder.end();
      modelInstance = new ModelInstance(model);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
