package us.ihmc.gdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;

public class GDXFootstepPlannerGoalGizmo implements RenderableProvider
{
   private final Pose3D pose = new Pose3D();
   /** The main, source, true, base transform that this thing represents. */
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private boolean dragging = false;
   private final String imGuiWindowName;
   private FocusBasedGDXCamera camera3D;
   private final Point3D cameraPosition = new Point3D();
   private double lastDistanceToCamera = -1.0;

   public GDXFootstepPlannerGoalGizmo(String name)
   {
      imGuiWindowName = ImGuiTools.uniqueLabel("3D Widget (" + name + ")");
   }

   public void create(FocusBasedGDXCamera camera3D)
   {
      this.camera3D = camera3D;

      recreateGraphics();
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      updateFromSourceTransform();

      boolean rightMouseDown = ImGui.getIO().getMouseDown(ImGuiMouseButton.Right);
      boolean isWindowHovered = ImGui.isWindowHovered();

      if (!rightMouseDown)
      {
         dragging = false;
      }
      if (isWindowHovered && !dragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineCurrentSelectionFromPickRay(pickRay);

         if (rightMouseDown)
         {
            dragging = true;
         }
      }
      if (dragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();

      }

      // after things have been modified, update the derivative stuff
      updateFromSourceTransform();

      GDXTools.toEuclid(camera3D.position, cameraPosition);
      double distanceToCamera = cameraPosition.distance(pose.getPosition());
      if (lastDistanceToCamera != distanceToCamera)
      {
         lastDistanceToCamera = distanceToCamera;
         recreateGraphics();
         updateFromSourceTransform();
      }
   }

   private void updateFromSourceTransform()
   {
      pose.set(transform);
   }

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {

   }

   public void renderImGuiTuner()
   {
      ImGui.begin(imGuiWindowName);
      ImGui.text("Use the right mouse button to manipulate the widget.");

      if (ImGui.button("Reset"))
      {
         transform.setToZero();
      }

      ImGui.pushItemWidth(100.00f);
      boolean proportionsChanged = false;
      ImGui.popItemWidth();

      if (proportionsChanged)
         recreateGraphics();

      ImGui.end();

      updateFromSourceTransform();
   }

   private void recreateGraphics()
   {
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
   }

   public Pose3DReadOnly getPose()
   {
      return pose;
   }

   // TODO: Make this transform the ground truth and give the pose as needed only
   public RigidBodyTransform getTransform()
   {
      return transform;
   }
}
