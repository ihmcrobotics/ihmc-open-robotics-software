package us.ihmc.rdx.ui.graphics.ros2.yolo;

import imgui.ImGui;
import imgui.ImGuiStyle;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.flag.ImGuiTableRowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.perception.detections.yolo.CRDTYOLOv8ModelParameters;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.util.Arrays;
import java.util.function.Consumer;

public class RDXROS2YOLOv8ModelSettings
{
   private static final int TABLE_COLUMN_COUNT = 6;
   private static final float MAX_TABLE_HEIGHT = 200.0f;

   private final CRDTYOLOv8ModelParameters syncedParameters;

   private final String modelName;
   private final String[] detectableObjectClasses;
   private final int detectableObjectClassCount;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   // Adjustors for all object classes
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImBoolean universalEnable = new ImBoolean(true);
   private final ImFloat universalConfidenceThreshold = new ImFloat(0.7f);
   private final ImFloat universalMaskThreshold = new ImFloat(0.0f);
   private final ImInt universalErosionKernelRadius = new ImInt(1);
   private final ImFloat universalOutlierThreshold = new ImFloat(1.0f);

   // Adjustors for individual object classes
   private final ImBoolean[] enables;
   private final ImFloat[] confidenceThresholds;
   private final ImFloat[] maskThresholds;
   private final ImInt[] erosionKernelRadii;
   private final ImFloat[] outlierThresholds;

   public RDXROS2YOLOv8ModelSettings(CRDTYOLOv8ModelParameters syncedParameters)
   {
      this.syncedParameters = syncedParameters;
      this.modelName = syncedParameters.getModelName();
      this.detectableObjectClasses = syncedParameters.getDetectableObjectClasses();
      detectableObjectClassCount = detectableObjectClasses.length;

      // Initialize settings
      enables = new ImBoolean[detectableObjectClassCount];
      confidenceThresholds = new ImFloat[detectableObjectClassCount];
      maskThresholds = new ImFloat[detectableObjectClassCount];
      erosionKernelRadii = new ImInt[detectableObjectClassCount];
      outlierThresholds = new ImFloat[detectableObjectClassCount];

      for (int i = 0; i < detectableObjectClassCount; ++i)
      {
         enables[i] = new ImBoolean(universalEnable);
         confidenceThresholds[i] = new ImFloat(universalConfidenceThreshold);
         maskThresholds[i] = new ImFloat(universalMaskThreshold);
         erosionKernelRadii[i] = new ImInt(universalErosionKernelRadius);
         outlierThresholds[i] = new ImFloat(universalOutlierThreshold);
      }

      updateSyncedParameters();
   }

   public String getModelName()
   {
      return modelName;
   }

   public void update()
   {
      if (syncedParameters.isModified())
      {
         nmsThreshold.set(syncedParameters.getNMSThreshold().getValue());
         for (int i = 0; i < detectableObjectClassCount; ++i)
         {
            enables[i].set(!syncedParameters.getIgnoredObjectClasses().getValueReadOnly(i));
            confidenceThresholds[i].set(syncedParameters.getConfidenceThresholds().getValueReadOnly(i));
            maskThresholds[i].set(syncedParameters.getMaskThresholds().getValueReadOnly(i));
            erosionKernelRadii[i].set(syncedParameters.getErosionKernelRadii().getValueReadOnly(i));
            outlierThresholds[i].set(syncedParameters.getOutlierThresholds().getValueReadOnly(i));
         }
      }
   }

   public void renderSettings()
   {
      boolean changed = false;

      changed |= ImGui.sliderFloat(labels.get("NMS Threshold"), nmsThreshold.getData(), 0.0f, 1.0f);

      final ImGuiStyle style = new ImGuiStyle();
      final int noScrollTableFlags = ImGuiTableFlags.BordersV
                                     | ImGuiTableFlags.BordersOuterH
                                     | ImGuiTableFlags.NoKeepColumnsVisible
                                     | ImGuiTableFlags.RowBg;
      float noScrollTableHeight = ImGuiTools.calcRenderSize(() -> renderSettingsTable(style, noScrollTableFlags, 0.0f)).y;

      int tableFlags = noScrollTableFlags;
      float tableHeight = 0.0f;
      if (noScrollTableHeight > MAX_TABLE_HEIGHT)
      {
         tableFlags |= ImGuiTableFlags.ScrollY;
         tableHeight = MAX_TABLE_HEIGHT;
      }

      changed |= renderSettingsTable(style, tableFlags, tableHeight);

      if (changed)
         updateSyncedParameters();
   }

   private boolean renderSettingsTable(ImGuiStyle style, int tableFlags, float height)
   {
      boolean changed = false;
      if (ImGui.beginTable(labels.getHidden("Object Class Settings"), TABLE_COLUMN_COUNT, tableFlags, 0.0f, height))
      {
         // Always show first two rows (header + universal adjusters)
         ImGui.tableSetupScrollFreeze(0, 2);

         float widgetWidth = ImGui.calcTextSize("0.00").x + (2.0f * style.getItemInnerSpacingX()) + 1.0f;
         ImGui.tableSetupColumn(labels.get("Enable"), ImGuiTableColumnFlags.WidthFixed | ImGuiTableColumnFlags.NoHeaderWidth);
         ImGui.tableSetupColumn(labels.get("Object Class"), ImGuiTableColumnFlags.WidthStretch);
         ImGui.tableSetupColumn(labels.get("Confidence Threshold"), ImGuiTableColumnFlags.WidthFixed, widgetWidth);
         ImGui.tableSetupColumn(labels.get("Mask Threshold"), ImGuiTableColumnFlags.WidthFixed, widgetWidth);
         ImGui.tableSetupColumn(labels.get("Erosion Kernel Radius"), ImGuiTableColumnFlags.WidthFixed, widgetWidth);
         ImGui.tableSetupColumn(labels.get("Outlier Threshold"), ImGuiTableColumnFlags.WidthFixed, widgetWidth);

         // Render header
         ImGui.tableHeadersRow();
         ImGui.setItemAllowOverlap();

         changed |= renderUniversalAdjustors();
         changed |= renderIndividualAdjustors();

         ImGui.endTable();
      }

      return changed;
   }

   private boolean renderUniversalAdjustors()
   {
      boolean changed = false;

      // Render universal adjusters
      ImGui.tableNextRow(ImGuiTableRowFlags.Headers);
      if (ImGui.tableNextColumn()) // Enable
      {
         ImGui.setNextItemWidth(-1.0f);
         if (ImGui.checkbox(labels.getHidden("enable universal"), universalEnable))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               enables[i].set(universalEnable);
            changed = true;
         }
         ImGui.setItemAllowOverlap();
      }

      if (ImGui.tableNextColumn()) // Object Class
      {
         ImGui.setNextItemWidth(-1.0f);
         ImGui.text("UNIVERSAL ADJUSTER");
      }

      if (ImGui.tableNextColumn()) // Confidence threshold
      {
         if (tableInputFloat(universalConfidenceThreshold, 0.0f, 1.0f))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               confidenceThresholds[i].set(universalConfidenceThreshold);
            changed = true;
         }
      }

      if (ImGui.tableNextColumn()) // Mask Threshold
      {
         if (tableInputFloat(universalMaskThreshold, -10.0f, 10.0f))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               maskThresholds[i].set(universalMaskThreshold);
            changed = true;
         }
      }

      if (ImGui.tableNextColumn()) // Erosion kernel radius
      {
         if (tableInputInt(universalErosionKernelRadius, 0, 10))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               erosionKernelRadii[i].set(universalErosionKernelRadius);
            changed = true;
         }
      }

      if (ImGui.tableNextColumn()) // Outlier threshold
      {
         if (tableInputFloat(universalOutlierThreshold, 0.0f, 1.0f))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               outlierThresholds[i].set(universalOutlierThreshold);
            changed = true;
         }
      }

      return changed;
   }

   private boolean renderIndividualAdjustors()
   {
      boolean changed = false;

      for (int i = 0; i < detectableObjectClassCount; ++i)
      {
         ImGui.tableNextRow();

         if (ImGui.tableNextColumn()) // Enable
         {
            ImGui.setNextItemWidth(-1.0f);
            changed |= ImGui.checkbox(labels.getHidden("enable" + i), enables[i]);
            ImGui.setItemAllowOverlap();
         }

         if (ImGui.tableNextColumn()) // Object class
         {
            ImGui.setNextItemWidth(-1.0f);
            ImGui.text(detectableObjectClasses[i]);
         }

         if (ImGui.tableNextColumn()) // Confidence threshold
            changed |= tableInputFloat(confidenceThresholds[i], 0.0f, 1.0f);

         if (ImGui.tableNextColumn()) // Mask threshold
            changed |= tableInputFloat(maskThresholds[i], -10.0f, 10.0f);

         if (ImGui.tableNextColumn()) // Erosion kernel radius
            changed |= tableInputInt(erosionKernelRadii[i], 0, 10);

         if (ImGui.tableNextColumn()) // Outlier threshold
            changed |= tableInputFloat(outlierThresholds[i], 0.0f, 1.0f);
      }

      return changed;
   }

   private boolean tableInputFloat(ImFloat value, float min, float max)
   {
      String label = ImGui.tableGetColumnName() + ImGui.tableGetColumnIndex() + ImGui.tableGetRowIndex();

      ImGui.setNextItemWidth(-1.0f);
      boolean changed = ImGui.inputFloat(labels.getHidden(label), value, 0.0f, 0.0f, "%.2f", ImGuiInputTextFlags.EnterReturnsTrue);
      ImGui.setItemAllowOverlap();

      if (changed && min < max)
         value.set((float) MathTools.clamp(value.get(), min, max));

      return changed;
   }

   private boolean tableInputInt(ImInt value, int min, int max)
   {
      String label = ImGui.tableGetColumnName() + ":" + ImGui.tableGetColumnIndex() + ":" + ImGui.tableGetRowIndex();

      ImGui.setNextItemWidth(-1.0f);
      boolean changed = ImGui.inputInt(labels.getHidden(label), value, 0, 0, ImGuiInputTextFlags.EnterReturnsTrue);
      ImGui.setItemAllowOverlap();

      if (changed && min < max)
         value.set(MathTools.clamp(value.get(), min, max));

      return changed;
   }

   private void updateSyncedParameters()
   {
      syncedParameters.getNMSThreshold().setValue(nmsThreshold.get());
      for (int i = 0; i < detectableObjectClassCount; ++i)
      {
         syncedParameters.getIgnoredObjectClasses().setValue(i, !enables[i].get());
         syncedParameters.getConfidenceThresholds().setValue(i, confidenceThresholds[i].get());
         syncedParameters.getMaskThresholds().setValue(i, maskThresholds[i].get());
         syncedParameters.getErosionKernelRadii().setValue(i, erosionKernelRadii[i].get());
         syncedParameters.getOutlierThresholds().setValue(i, outlierThresholds[i].get());
      }
   }
}
