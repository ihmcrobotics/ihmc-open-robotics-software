package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXPose3DWidget;

public class GDXEnvironment implements RenderableProvider
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironment.class, "Environment");
   private GDXImGuiBasedUI baseUI;
   private GDXMediumCinderBlockRoughed cinderBlock;
   private final GDXPose3DWidget pose3DWidget = new GDXPose3DWidget();

   private boolean selected = false;
   private boolean intersecting = false;
   private final Point3D intersection = new Point3D();

   private final ImFloat r = new ImFloat(1.0f);
   private final ImFloat g = new ImFloat(1.0f);
   private final ImFloat b = new ImFloat(1.0f);
   private final ImFloat a = new ImFloat(1.0f);

   public void create(GDXImGuiBasedUI baseUI)
   {
      this.baseUI = baseUI;
      cinderBlock = new GDXMediumCinderBlockRoughed();
      baseUI.getSceneManager().addRenderableProvider(this);

      pose3DWidget.create(baseUI);
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   private void process3DViewInput(ImGui3DViewInput viewInput)
   {
      if (selected)
      {
         pose3DWidget.process3DViewInput(viewInput);
         GDXTools.toGDX(pose3DWidget.getTransform(), cinderBlock.getRealisticModelInstance().transform);
         GDXTools.toGDX(pose3DWidget.getTransform(), cinderBlock.getCollisionModelInstance().transform);
         cinderBlock.getCollisionGeometryObject().getPose().set(pose3DWidget.getTransform());

         if (ImGui.getMouseDragDeltaX() == 0.0f
          && ImGui.getMouseDragDeltaX() == 0.0f
          && ImGui.isMouseReleased(ImGuiMouseButton.Left)
          && viewInput.isWindowHovered())
         {
            selected = intersecting = cinderBlock.intersect(viewInput.getPickRayInWorld(baseUI), intersection);
         }
      }
      else
      {
         if (viewInput.isWindowHovered())
         {
            Line3DReadOnly pickRay = viewInput.getPickRayInWorld(baseUI);
            intersecting = cinderBlock.intersect(pickRay, intersection);

            if (ImGui.getMouseDragDeltaX() == 0.0f && ImGui.getMouseDragDeltaX() == 0.0f && ImGui.isMouseReleased(ImGuiMouseButton.Left))
            {
               selected = intersecting;

               if (intersecting)
               {
                  GDXTools.toEuclid(cinderBlock.getRealisticModelInstance().transform, pose3DWidget.getTransform());
               }
            }
         }
      }
   }

   public void render()
   {
      ImGui.begin(WINDOW_NAME);

      ImGui.text("Selected: " + selected);
      ImGui.text("Intersecting: " + intersecting);

      ImGui.dragFloat("R", r.getData(), 0.01f, 0.0f, 1.0f);
      ImGui.dragFloat("G", g.getData(), 0.01f, 0.0f, 1.0f);
      ImGui.dragFloat("B", b.getData(), 0.01f, 0.0f, 1.0f);
      ImGui.dragFloat("A", a.getData(), 0.01f, 0.0f, 1.0f);

      pose3DWidget.render();

      ImGui.end();

      cinderBlock.getCollisionModelInstance().materials.get(0).set(ColorAttribute.createDiffuse(r.get(), g.get(), b.get(), a.get()));
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      cinderBlock.getRealisticModelInstance().getRenderables(renderables, pool);

      if (selected || intersecting)
      {
         cinderBlock.getCollisionModelInstance().getRenderables(renderables, pool);
      }

      if (selected)
      {
         pose3DWidget.getRenderables(renderables, pool);
      }
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
