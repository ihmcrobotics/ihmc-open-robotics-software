package us.ihmc.gdx.ui.tools;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXTransformTuner
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RigidBodyTransform transformToTune;
   private final ImBoolean showGizmo = new ImBoolean(false);
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private final ImDouble x = new ImDouble();
   private final ImDouble y = new ImDouble();
   private final ImDouble z = new ImDouble();
   private final ImDouble yaw = new ImDouble();
   private final ImDouble pitch = new ImDouble();
   private final ImDouble roll = new ImDouble();

   public GDXTransformTuner(RigidBodyTransform transformToTune)
   {
      this.transformToTune = transformToTune;
   }

   public void create(FocusBasedGDXCamera camera3D)
   {
      poseGizmo.create(camera3D);
   }

   // happens before renderImGuiWidgets
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (showGizmo.get())
      {
         poseGizmo.getTransform().set(transformToTune);
         poseGizmo.process3DViewInput(input);
         transformToTune.set(poseGizmo.getTransform());
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Show gizmo"), showGizmo))
      {
         if (showGizmo.get())
         {
            poseGizmo.getTransform().set(transformToTune);
         }
      }

      x.set(transformToTune.getTranslation().getX());
      y.set(transformToTune.getTranslation().getY());
      z.set(transformToTune.getTranslation().getZ());
      yaw.set(transformToTune.getRotation().getYaw());
      pitch.set(transformToTune.getRotation().getPitch());
      roll.set(transformToTune.getRotation().getRoll());
      ImGui.inputDouble(labels.get("x"), x, 0.01);
      ImGui.inputDouble(labels.get("y"), y, 0.01);
      ImGui.inputDouble(labels.get("z"), z, 0.01);
      ImGui.inputDouble(labels.get("yaw"), yaw, 0.01);
      ImGui.inputDouble(labels.get("pitch"), pitch, 0.01);
      ImGui.inputDouble(labels.get("roll"), roll, 0.01);
      transformToTune.getTranslation().set(x.get(), y.get(), z.get());
      transformToTune.getRotation().setYawPitchRoll(yaw.get(), pitch.get(), roll.get());

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
