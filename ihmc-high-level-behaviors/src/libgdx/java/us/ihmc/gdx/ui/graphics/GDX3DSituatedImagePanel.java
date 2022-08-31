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
import com.badlogic.gdx.math.Frustum;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXTools;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;
import static com.badlogic.gdx.graphics.VertexAttributes.Usage.TextureCoordinates;

public class GDX3DSituatedImagePanel
{
   private ModelInstance modelInstance;
   private Texture texture;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final Vector3 topLeftPosition = new Vector3();
   private final Vector3 bottomLeftPosition = new Vector3();
   private final Vector3 bottomRightPosition = new Vector3();
   private final Vector3 topRightPosition = new Vector3();
   private final Vector3 topLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
   private final Vector3 bottomLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
   private final Vector3 bottomRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
   private final Vector3 topRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
   private final Vector2 topLeftUV = new Vector2();
   private final Vector2 bottomLeftUV = new Vector2();
   private final Vector2 bottomRightUV = new Vector2();
   private final Vector2 topRightUV = new Vector2();

   public void create(Texture texture, Frustum frustum, ReferenceFrame referenceFrame, boolean flipY)
   {
      // Counter clockwise order
      // Draw so thumb faces away and index right
      Vector3[] planePoints = frustum.planePoints;
      topLeftPosition.set(planePoints[7]);
      bottomLeftPosition.set(planePoints[4]);
      bottomRightPosition.set(planePoints[5]);
      topRightPosition.set(planePoints[6]);
      create(texture, referenceFrame, flipY);
   }

   public void create(Texture texture, Vector3[] points, ReferenceFrame referenceFrame, boolean flipY)
   {
      topLeftPosition.set(points[0]);
      bottomLeftPosition.set(points[1]);
      bottomRightPosition.set(points[2]);
      topRightPosition.set(points[3]);
      create(texture, referenceFrame, flipY);
   }

   public void create(Texture texture, double panelWidth, double panelHeight, ReferenceFrame referenceFrame, boolean flipY)
   {
      float halfPanelHeight = (float) panelHeight / 2.0f;
      float halfPanelWidth = (float) panelWidth / 2.0f;
      topLeftPosition.set(halfPanelHeight, halfPanelWidth, 0.0f);
      bottomLeftPosition.set(-halfPanelHeight, halfPanelWidth, 0.0f);
      bottomRightPosition.set(-halfPanelHeight, -halfPanelWidth, 0.0f);
      topRightPosition.set(halfPanelHeight, -halfPanelWidth, 0.0f);
      create(texture, referenceFrame, flipY);
   }

   private void create(Texture texture, ReferenceFrame referenceFrame, boolean flipY)
   {
      this.texture = texture;
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Counter clockwise order
      // Draw so thumb faces away and index right
      topLeftUV.set(0.0f, flipY ? 1.0f : 0.0f);
      bottomLeftUV.set(0.0f, flipY ? 0.0f : 1.0f);
      bottomRightUV.set(1.0f, flipY ? 0.0f : 1.0f);
      topRightUV.set(1.0f, flipY ? 1.0f : 0.0f);
      tempFramePoint.setToZero(ReferenceFrame.getWorldFrame());
      GDXTools.toEuclid(topLeftPosition, tempFramePoint);
      tempFramePoint.changeFrame(referenceFrame);
      GDXTools.toGDX(tempFramePoint, topLeftPosition);
      meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topLeftUV);
      tempFramePoint.setToZero(ReferenceFrame.getWorldFrame());
      GDXTools.toEuclid(bottomLeftPosition, tempFramePoint);
      tempFramePoint.changeFrame(referenceFrame);
      GDXTools.toGDX(tempFramePoint, bottomLeftPosition);
      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomLeftUV);
      tempFramePoint.setToZero(ReferenceFrame.getWorldFrame());
      GDXTools.toEuclid(bottomRightPosition, tempFramePoint);
      tempFramePoint.changeFrame(referenceFrame);
      GDXTools.toGDX(tempFramePoint, bottomRightPosition);
      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomRightUV);
      tempFramePoint.setToZero(ReferenceFrame.getWorldFrame());
      GDXTools.toEuclid(topRightPosition, tempFramePoint);
      tempFramePoint.changeFrame(referenceFrame);
      GDXTools.toGDX(tempFramePoint, topRightPosition);
      meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topRightUV);
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
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }

   public void setPoseToReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (modelInstance != null)
      {
         referenceFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
         GDXTools.toGDX(tempTransform, modelInstance.transform);
      }
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public Texture getTexture()
   {
      return texture;
   }
}
