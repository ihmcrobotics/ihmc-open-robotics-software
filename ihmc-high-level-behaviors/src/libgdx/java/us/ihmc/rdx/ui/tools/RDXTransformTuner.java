package us.ihmc.rdx.ui.tools;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiRigidBodyTransformTuner;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

public class RDXTransformTuner
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RigidBodyTransform transformToTune;
   private final ImBoolean showGizmo = new ImBoolean(false);
   private final RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo();
   private final ImGuiRigidBodyTransformTuner transformTuner;

   public RDXTransformTuner(RigidBodyTransform transformToTune)
   {
      this.transformToTune = transformToTune;

      transformTuner = new ImGuiRigidBodyTransformTuner(transformToTune);
   }

   public void create(RDX3DPanel panel3D)
   {
      poseGizmo.create(panel3D);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (showGizmo.get())
      {
         poseGizmo.calculate3DViewPick(input);
      }
   }

   // happens before renderImGuiWidgets
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (showGizmo.get())
      {
         poseGizmo.getTransformToParent().set(transformToTune);
         poseGizmo.process3DViewInput(input);
         transformToTune.set(poseGizmo.getTransformToParent());
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Show gizmo"), showGizmo))
      {
         if (showGizmo.get())
         {
            poseGizmo.getTransformToParent().set(transformToTune);
         }
      }

      transformTuner.renderTunerWithYawPitchRoll();

      if (ImGui.button(labels.get("Print transform")))
      {
         System.out.println(transformToTune);
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showGizmo.get())
      {
         poseGizmo.getRenderables(renderables, pool);
      }
   }
}
