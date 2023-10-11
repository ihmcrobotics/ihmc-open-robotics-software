package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

public class RDXPoseModifiableObject
{
   private final RDXPose3DGizmo pose3DGizmo = new RDXPose3DGizmo();
   private boolean isSelected = false;
   private boolean showCollisionMesh = false;
   private RDXEnvironmentObject object;
   private final Point3D tempIntersection = new Point3D();

   public void create(RDX3DPanel panel3D, RDXEnvironmentObject object)
   {
      this.object = object;
      pose3DGizmo.create(panel3D);
      panel3D.addImGui3DViewInputProcessor(this, this::process3DViewInput);
      object.setTransformToWorld(pose3DGizmo.getTransformToParent());
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (isSelected)
      {
         pose3DGizmo.calculate3DViewPick(input);
      }
   }

   private void process3DViewInput(ImGui3DViewInput viewInput)
   {
      showCollisionMesh = false;
      if (isSelected)
      {
         pose3DGizmo.process3DViewInput(viewInput);
         object.setTransformToWorld(pose3DGizmo.getTransformToParent());

         if (viewInput.isWindowHovered()
          && viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left)
          && !object.intersect(viewInput.getPickRayInWorld(), tempIntersection))
         {
            isSelected = false;
         }
      }
      else
      {
         if (viewInput.isWindowHovered())
         {
            boolean intersects = object.intersect(viewInput.getPickRayInWorld(), tempIntersection);
            showCollisionMesh = intersects;

            if (viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left) && intersects)
            {
               isSelected = true;
               pose3DGizmo.getTransformToParent().set(object.getObjectTransform());
            }
         }
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showCollisionMesh)
         object.getCollisionMeshRenderables(renderables, pool);
      if (isSelected)
         pose3DGizmo.getRenderables(renderables, pool);
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      object.getRealRenderables(renderables, pool);
   }

   public RDXEnvironmentObject getObject()
   {
      return object;
   }
}
