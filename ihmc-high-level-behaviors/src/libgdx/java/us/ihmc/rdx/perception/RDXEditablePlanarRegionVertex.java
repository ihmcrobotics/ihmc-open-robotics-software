package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.gizmo.DynamicLibGDXModel;
import us.ihmc.rdx.ui.gizmo.RDXGizmoTools;
import us.ihmc.rdx.ui.gizmo.SphereRayIntersection;

import java.util.Set;
import java.util.function.Consumer;

public class RDXEditablePlanarRegionVertex
{
   private float radius = 0.03f;
   private Material normalMaterial;
   private Material highlightMaterial;
   private DynamicLibGDXModel sphereGraphic;
   private final SphereRayIntersection sphereRayIntersection = new SphereRayIntersection();
   private Point3D positionInWorld = new Point3D();
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean hovered = false;

   public RDXEditablePlanarRegionVertex()
   {
      normalMaterial = new Material();
      normalMaterial.set(TextureAttribute.createDiffuse(RDXMultiColorMeshBuilder.loadPaletteTexture()));
      normalMaterial.set(new BlendingAttribute(true, RDXGizmoTools.CENTER_DEFAULT_COLOR.a));

      highlightMaterial = new Material();
      highlightMaterial.set(TextureAttribute.createDiffuse(RDXMultiColorMeshBuilder.loadPaletteTexture()));
      highlightMaterial.set(new BlendingAttribute(true, RDXGizmoTools.CENTER_SELECTED_DEFAULT_COLOR.a));

      sphereGraphic = new DynamicLibGDXModel();
      sphereGraphic.setMesh(meshBuilder -> meshBuilder.addSphere(radius, RDXGizmoTools.CENTER_DEFAULT_COLOR));
      sphereGraphic.setMaterial(normalMaterial);
      sphereGraphic.getOrCreateModelInstance();
   }

   public void update()
   {
      LibGDXTools.toEuclid(sphereGraphic.getModelInstance().transform, positionInWorld);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      sphereRayIntersection.update(radius, positionInWorld);
      boolean intersects = sphereRayIntersection.intersect(input.getPickRayInWorld());

      if (intersects)
      {
         pickResult.setDistanceToCamera(positionInWorld.distance(input.getPickRayInWorld().getPoint()));
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      boolean hovered = input.getClosestPick() == pickResult;
      if (this.hovered != hovered)
      {
         this.hovered = hovered;

         if (hovered)
         {
            sphereGraphic.setMaterial(highlightMaterial);
         }
         else
         {
            sphereGraphic.setMaterial(normalMaterial);
         }
         sphereGraphic.getOrCreateModelInstance();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         sphereGraphic.getModelInstance().getRenderables(renderables, pool);
      }
   }

   public void setPositionInWorld(Point3DReadOnly position)
   {
      LibGDXTools.toLibGDX(position, sphereGraphic.getModelInstance().transform);
   }

   public void setPositionInWorld(Consumer<Point3DBasics> positionSetter)
   {
      positionSetter.accept(positionInWorld);
      setPositionInWorld(positionInWorld);
   }

   public boolean isHovered()
   {
      return hovered;
   }
}
