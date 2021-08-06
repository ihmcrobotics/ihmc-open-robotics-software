package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXFootstepPlannerGoalGizmo;

public class GDXWalkPathControlRing
{
   private final GDXFootstepPlannerGoalGizmo footstepPlannerGoalGizmo = new GDXFootstepPlannerGoalGizmo();
   private boolean selected = false;
   private boolean modified = false;
   private boolean mouseIntersects;
   private ROS2SyncedRobotModel syncedRobot;

   public void create(GDXImGuiBasedUI baseUI, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      footstepPlannerGoalGizmo.create(baseUI.get3DSceneManager().getCamera3D());
      baseUI.addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void update()
   {
      footstepPlannerGoalGizmo.getTransform().set(syncedRobot.getReferenceFrames().getMidFeetZUpFrame().getTransformToWorldFrame());
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      footstepPlannerGoalGizmo.process3DViewInput(input);
      mouseIntersects = footstepPlannerGoalGizmo.getHollowCylinderIntersects();
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modified || mouseIntersects)
      {
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
      }
   }
}
