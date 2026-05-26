package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.util.Set;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;

public class RDX3DSituatedImagePanel
{
   protected float panelDistance = 0.77f;

   private ModelInstance modelInstance;
   protected ModelInstance hoverFrustumMesh;
   protected Texture texture;
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

   protected final RigidBodyTransform floatingPanelTransformToWorld = new RigidBodyTransform();
   protected final ReferenceFrame floatingPanelFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                            floatingPanelTransformToWorld);
   protected boolean isShowing = false;

   protected static final boolean RENDER_FRUSTUM = false;
   protected final boolean checkCollison;
   protected CameraPanelFrustumCollision frustumCollidable;
   protected boolean frustumInitialized = false;
   protected final RigidBodyTransform frustumTransformOriginInWorld = new RigidBodyTransform();

   public RDX3DSituatedImagePanel(boolean checkCollision)
   {
      this.checkCollison = checkCollision;
   }

   public void create(Texture texture, Vector3[] points, ReferenceFrame centerOfPanelFrame, boolean flipY, float verticalFOV)
   {
      topLeftPosition.set(points[0]);
      bottomLeftPosition.set(points[1]);
      bottomRightPosition.set(points[2]);
      topRightPosition.set(points[3]);
      create(texture, centerOfPanelFrame, flipY, verticalFOV);
   }

   private void create(Texture texture, ReferenceFrame centerOfPanelFrame, boolean flipY, float verticalFOV)
   {
      this.texture = texture;
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Draw so thumb faces away and index right
      topLeftUV.set(0.0f, flipY ? 1.0f : 0.0f);
      bottomLeftUV.set(0.0f, flipY ? 0.0f : 1.0f);
      bottomRightUV.set(1.0f, flipY ? 0.0f : 1.0f);
      topRightUV.set(1.0f, flipY ? 1.0f : 0.0f);

      // Transform the corners into world frame
      transformFromLocalToWorld(topLeftPosition, centerOfPanelFrame);
      transformFromLocalToWorld(bottomLeftPosition, centerOfPanelFrame);
      transformFromLocalToWorld(bottomRightPosition, centerOfPanelFrame);
      transformFromLocalToWorld(topRightPosition, centerOfPanelFrame);

      // Add vertices for the front of the panel
      meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topLeftUV);
      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomLeftUV);
      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomRightUV);
      meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topRightUV);

      // Add a mirrored image on the back so it's always visible.
      meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topRightUV);
      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomRightUV);
      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomLeftUV);
      meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topLeftUV);

      // Front
      meshBuilder.triangle((short) 3, (short) 0, (short) 1);
      meshBuilder.triangle((short) 1, (short) 2, (short) 3);
      // Back
      meshBuilder.triangle((short) 5, (short) 4, (short) 7);
      meshBuilder.triangle((short) 7, (short) 6, (short) 5);

      Mesh mesh = meshBuilder.end();
      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();

      material.set(PBRTextureAttribute.createBaseColorTexture(texture));
      material.set(PBRColorAttribute.createDiffuse(new Color(0.68235f, 0.688235f, 0.688235f, 1.0f)));
      modelBuilder.part(meshPart, material);

      Model model = modelBuilder.end();
      modelInstance = new ModelInstance(model);

      if (!frustumInitialized && checkCollison)
      {
         buildFrustum(verticalFOV);
         frustumInitialized = true;
      }
   }

   private void buildFrustum(float verticalFOV)
   {
      float closeToCamera = 0.01f;
      float farFromCamera = 1.0f;
      float aspect = texture.getWidth() / (float) texture.getHeight();

      float minImageHeight = 2.0f * closeToCamera * (float) Math.tan(Math.toRadians(verticalFOV) / 2.0f);
      float minImageWidth = minImageHeight * aspect;
      float maxImageHeight = 2.0f * farFromCamera * (float) Math.tan(Math.toRadians(verticalFOV) / 2.0f);
      float maxImageWidth = maxImageHeight * aspect;

      Vector3[] nearCorners = {
            new Vector3(closeToCamera, minImageWidth / 2f, minImageHeight / 2f),
            new Vector3(closeToCamera, minImageWidth / 2f, -minImageHeight / 2f),
            new Vector3(closeToCamera, -minImageWidth / 2f, -minImageHeight / 2f),
            new Vector3(closeToCamera, -minImageWidth / 2f, minImageHeight / 2f)
      };
      Vector3[] farCorners = {
            new Vector3(farFromCamera, maxImageWidth / 2f, maxImageHeight / 2f),
            new Vector3(farFromCamera, maxImageWidth / 2f, -maxImageHeight / 2f),
            new Vector3(farFromCamera, -maxImageWidth / 2f, -maxImageHeight / 2f),
            new Vector3(farFromCamera, -maxImageWidth / 2f, maxImageHeight / 2f)
      };

      frustumCollidable = new CameraPanelFrustumCollision(nearCorners, farCorners);
      if (RENDER_FRUSTUM)
      {
         hoverFrustumMesh = new ModelInstance(getFrustumModel(nearCorners, farCorners));
         LibGDXTools.toEuclid(hoverFrustumMesh.transform, frustumTransformOriginInWorld);
      }
   }

   private Model getFrustumModel(Vector3[] nearCorners, Vector3[] farCorners)
   {
      RDXMultiColorMeshBuilder multiColorMeshBuilder = new RDXMultiColorMeshBuilder();
      Color color = Color.WHITE;
      double lineWidth = 0.002;
      // Near plane rectangle
      for (int i = 0; i < 4; i++)
         multiColorMeshBuilder.addLine(nearCorners[i].x,
                                       nearCorners[i].y,
                                       nearCorners[i].z,
                                       nearCorners[(i + 1) % 4].x,
                                       nearCorners[(i + 1) % 4].y,
                                       nearCorners[(i + 1) % 4].z,
                                       lineWidth,
                                       color);

      // Far plane rectangle
      for (int i = 0; i < 4; i++)
         multiColorMeshBuilder.addLine(farCorners[i].x,
                                       farCorners[i].y,
                                       farCorners[i].z,
                                       farCorners[(i + 1) % 4].x,
                                       farCorners[(i + 1) % 4].y,
                                       farCorners[(i + 1) % 4].z,
                                       lineWidth,
                                       color);

      // Connect each near-far corner (vertical frustum edges)
      for (int i = 0; i < 4; i++)
         multiColorMeshBuilder.addLine(nearCorners[i].x,
                                       nearCorners[i].y,
                                       nearCorners[i].z,
                                       farCorners[i].x,
                                       farCorners[i].y,
                                       farCorners[i].z,
                                       lineWidth,
                                       color);

      Mesh frustumMesh = multiColorMeshBuilder.generateMesh();
      MeshPart frustumMeshPart = new MeshPart("frustum", frustumMesh, 0, frustumMesh.getNumIndices(), GL41.GL_LINES);
      Material lineMaterial = new Material();
      ModelBuilder frustumModelBuilder = new ModelBuilder();
      frustumModelBuilder.begin();
      frustumModelBuilder.part(frustumMeshPart, lineMaterial);
      return frustumModelBuilder.end();
   }

   private void transformFromLocalToWorld(Vector3 positionToTransform, ReferenceFrame localFrame)
   {
      tempFramePoint.setToZero(ReferenceFrame.getWorldFrame());
      LibGDXTools.toEuclid(positionToTransform, tempFramePoint);
      tempFramePoint.changeFrame(localFrame);
      LibGDXTools.toLibGDX(tempFramePoint, positionToTransform);
   }

   public void update(Texture imageTexture, ReferenceFrame cameraFrame, float verticalFOV)
   {
      // Prevent ever having an invisible panel out there, which is very confusing
      // to the VR user when the controllers are colliding with and invisible box.
      boolean somethingToShow = imageTexture != null || texture != null;
      isShowing = somethingToShow;

      // Update the texture if necessary
      if (isShowing && imageTexture != null)
      {
         boolean flipY = false;
         // Calculate panel size in meters from FOV
         float panelHeightMeters = 2.0f * panelDistance * (float) Math.tan(Math.toRadians(verticalFOV) / 2.0f);
         float aspectRatio = imageTexture.getWidth() / (float) imageTexture.getHeight();
         float panelWidthMeters = panelHeightMeters * aspectRatio;
         float halfWidth = panelWidthMeters * 0.5f;
         float halfHeight = panelHeightMeters * 0.5f;

         // Build the transform: start with identity, translate by +distanceMeters along X, then apply cameraFrame transform
         floatingPanelTransformToWorld.set(cameraFrame.getTransformToRoot());
         RigidBodyTransform camToPanel = new RigidBodyTransform();
         camToPanel.setIdentity();
         camToPanel.getTranslation().set(panelDistance, 0.0, 0.0); // move distanceMeters along X in camera frame
         floatingPanelTransformToWorld.multiply(camToPanel); // floatingPanel now at (distanceMeters,0,0) in camera frame
         floatingPanelTransformToWorld.invert();
         floatingPanelFrame.update();

         create(imageTexture,
                new Vector3[] {new Vector3(0.0f, halfWidth, halfHeight),
                               new Vector3(0.0f, halfWidth, -halfHeight),
                               new Vector3(0.0f, -halfWidth, -halfHeight),
                               new Vector3(0.0f, -halfWidth, halfHeight)},
                floatingPanelFrame,
                flipY,
                verticalFOV);

         RigidBodyTransform newFrustumTransform = new RigidBodyTransform(cameraFrame.getTransformToRoot());
         newFrustumTransform.multiplyInvertOther(frustumTransformOriginInWorld);
         if (hoverFrustumMesh != null)
         {
            LibGDXTools.toLibGDX(newFrustumTransform, hoverFrustumMesh.transform);
         }
         if (frustumCollidable != null)
         {
            frustumCollidable.updateCorners(newFrustumTransform);
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isShowing && sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         if (modelInstance != null)
            modelInstance.getRenderables(renderables, pool);
         if (hoverFrustumMesh != null)
            hoverFrustumMesh.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      // Dispose the texture if we own it
      if (texture != null)
      {
         texture.dispose();
         texture = null;
      }

      // Dispose the main model
      if (modelInstance != null)
      {
         if (modelInstance.model != null)
            modelInstance.model.dispose();
         modelInstance = null;
      }

      // Dispose the hover box mesh model
      if (hoverFrustumMesh != null)
      {
         if (hoverFrustumMesh.model != null)
            hoverFrustumMesh.model.dispose();
         hoverFrustumMesh = null;
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