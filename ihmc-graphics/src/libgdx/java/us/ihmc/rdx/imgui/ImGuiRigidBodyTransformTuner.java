package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ImGuiRigidBodyTransformTuner
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RigidBodyTransform transformToTune;
   private final ImFloat x = new ImFloat();
   private final ImFloat y = new ImFloat();
   private final ImFloat z = new ImFloat();
   private final ImFloat yaw = new ImFloat();
   private final ImFloat pitch = new ImFloat();
   private final ImFloat roll = new ImFloat();
   private final ImFloat m00 = new ImFloat();
   private final ImFloat m01 = new ImFloat();
   private final ImFloat m02 = new ImFloat();
   private final ImFloat m10 = new ImFloat();
   private final ImFloat m11 = new ImFloat();
   private final ImFloat m12 = new ImFloat();
   private final ImFloat m20 = new ImFloat();
   private final ImFloat m21 = new ImFloat();
   private final ImFloat m22 = new ImFloat();

   public ImGuiRigidBodyTransformTuner()
   {
      this(new RigidBodyTransform());
   }

   public ImGuiRigidBodyTransformTuner(RigidBodyTransform transformToTune)
   {
      this.transformToTune = transformToTune;
   }

   public void renderTunerWithRotationMatrix()
   {
      renderTranslationTuner(0.01);

      m00.set((float) transformToTune.getRotation().getM00());
      m01.set((float) transformToTune.getRotation().getM01());
      m02.set((float) transformToTune.getRotation().getM02());
      m10.set((float) transformToTune.getRotation().getM10());
      m11.set((float) transformToTune.getRotation().getM11());
      m12.set((float) transformToTune.getRotation().getM12());
      m20.set((float) transformToTune.getRotation().getM20());
      m21.set((float) transformToTune.getRotation().getM21());
      m22.set((float) transformToTune.getRotation().getM22());
      ImGui.pushItemWidth(50.0f);
      ImGui.dragFloat(labels.get("m00"), m00.getData(), 0.01f);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("m01"), m01.getData(), 0.01f);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("m02"), m02.getData(), 0.01f);

      ImGui.dragFloat(labels.get("m10"), m10.getData(), 0.01f);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("m11"), m11.getData(), 0.01f);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("m12"), m12.getData(), 0.01f);

      ImGui.dragFloat(labels.get("m20"), m20.getData(), 0.01f);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("m21"), m21.getData(), 0.01f);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("m22"), m22.getData(), 0.01f);
      ImGui.popItemWidth();
      transformToTune.getRotation().set(m00.get(), m01.get(), m02.get(), m10.get(), m11.get(), m12.get(), m20.get(), m21.get(), m22.get());
   }

   public void renderTunerWithYawPitchRoll()
   {
      renderTunerWithYawPitchRoll(0.01);
   }

   public void renderTunerWithYawPitchRoll(double fineAdjustmentStepSize)
   {
      renderTranslationTuner(fineAdjustmentStepSize);

      yaw.set((float) transformToTune.getRotation().getYaw());
      pitch.set((float) transformToTune.getRotation().getPitch());
      roll.set((float) transformToTune.getRotation().getRoll());
      ImGui.pushItemWidth(50.0f);
      ImGui.dragFloat(labels.get("yaw"), yaw.getData(), (float) fineAdjustmentStepSize);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("pitch"), pitch.getData(), (float) fineAdjustmentStepSize);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("roll"), roll.getData(), (float) fineAdjustmentStepSize);
      ImGui.popItemWidth();
      transformToTune.getRotation().setYawPitchRoll(yaw.get(), pitch.get(), roll.get());
   }

   public void renderTranslationTuner(double fineAdjustmentStepSize)
   {
      x.set((float) transformToTune.getTranslation().getX());
      y.set((float) transformToTune.getTranslation().getY());
      z.set((float) transformToTune.getTranslation().getZ());
      ImGui.pushItemWidth(50.0f);
      ImGui.dragFloat(labels.get("x"), x.getData(), (float) fineAdjustmentStepSize);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("y"), y.getData(), (float) fineAdjustmentStepSize);
      ImGui.sameLine();
      ImGui.dragFloat(labels.get("z"), z.getData(), (float) fineAdjustmentStepSize);
      ImGui.popItemWidth();
      transformToTune.getTranslation().set(x.get(), y.get(), z.get());
   }
}
