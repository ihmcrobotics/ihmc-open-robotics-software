package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.GDXPose3DWidget;

import java.util.concurrent.atomic.AtomicInteger;

public class GDXPoseModifiableObject
{
   private final static AtomicInteger INDEX = new AtomicInteger();

   private final GDXPose3DWidget pose3DWidget = new GDXPose3DWidget(getClass().getSimpleName() + INDEX.getAndIncrement());
   private boolean isSelected = false;
   private boolean showCollisionMesh = false;
   private GDXEnvironmentObject object;
   private final Point3D tempIntersection = new Point3D();

   public void create(GDXImGuiBasedUI baseUI, GDXEnvironmentObject object)
   {
      this.object = object;
      pose3DWidget.create();
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
      object.set(pose3DWidget.getTransform());
   }

   private void process3DViewInput(ImGui3DViewInput viewInput)
   {
      showCollisionMesh = false;
      if (isSelected)
      {
         pose3DWidget.process3DViewInput(viewInput);
         object.set(pose3DWidget.getTransform());

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
            boolean intesects = object.intersect(viewInput.getPickRayInWorld(), tempIntersection);
            showCollisionMesh = intesects;

            if (viewInput.mouseReleasedWithoutDrag(ImGuiMouseButton.Left) && intesects)
            {
               isSelected = true;
               pose3DWidget.getTransform().set(object.getObjectTransform());
            }
         }
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showCollisionMesh)
         object.getCollisionModelInstance().getRenderables(renderables, pool);
      if (isSelected)
         pose3DWidget.getRenderables(renderables, pool);
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      object.getRealisticModelInstance().getRenderables(renderables, pool);
   }

   public GDXEnvironmentObject getObject()
   {
      return object;
   }
}
