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
import imgui.type.ImString;
import us.ihmc.commons.MathTools;
import us.ihmc.perception.detections.yolo.SyncedYOLOv8ModelParameters;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.util.Arrays;
import java.util.Comparator;

public class RDXROS2YOLOv8ModelSettings
{
   private static final int TABLE_COLUMN_COUNT = 6;
   private static final float MAX_TABLE_HEIGHT = 200.0f;

   private final SyncedYOLOv8ModelParameters syncedParameters;

   private final String modelName;
   private final int detectableObjectClassCount;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   // Adjustors for all object classes
   private final ImString tableFilter = new ImString();
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImBoolean universalEnable = new ImBoolean(true);
   private final ImFloat universalConfidenceThreshold = new ImFloat(0.7f);
   private final ImFloat universalMaskThreshold = new ImFloat(0.0f);
   private final ImInt universalErosionKernelRadius = new ImInt(1);
   private final ImFloat universalOutlierThreshold = new ImFloat(2.0f);

   // Adjustors for individual object classes
   private final RDXYOLOv8ModelClassSettings[] classSettings;

   public RDXROS2YOLOv8ModelSettings(SyncedYOLOv8ModelParameters syncedParameters)
   {
      this.syncedParameters = syncedParameters;
      this.modelName = syncedParameters.getModelName();

      String[] detectableClasses = syncedParameters.getDetectableObjectClasses();
      detectableObjectClassCount = detectableClasses.length;

      // Initialize parameters
      classSettings = new RDXYOLOv8ModelClassSettings[detectableObjectClassCount];
      for (int i = 0; i < detectableObjectClassCount; ++i)
         classSettings[i] = new RDXYOLOv8ModelClassSettings(i, detectableClasses[i], syncedParameters);

      Arrays.sort(classSettings, Comparator.comparing(settings -> settings.className));
   }

   public String getModelName()
   {
      return modelName;
   }

   public void update()
   {
      syncedParameters.checkModified();
      for (RDXYOLOv8ModelClassSettings classSetting : classSettings)
         classSetting.update();
   }

   public void renderSettings()
   {
      ImGui.sliderFloat(labels.get("NMS Threshold"), nmsThreshold.getData(), 0.0f, 1.0f);
      ImGui.inputTextWithHint(labels.getHidden("Search"), "Search", tableFilter);
      ImGui.sameLine();
      if (ImGui.button("Clear"))
         tableFilter.set("");

      final ImGuiStyle style = new ImGuiStyle();
      final int noScrollTableFlags = ImGuiTableFlags.BordersV | ImGuiTableFlags.BordersOuterH | ImGuiTableFlags.NoKeepColumnsVisible | ImGuiTableFlags.RowBg;
      float noScrollTableHeight = ImGuiTools.calcRenderSize(() -> renderSettingsTable(style, noScrollTableFlags, 0.0f)).y;

      int tableFlags = noScrollTableFlags;
      float tableHeight = 0.0f;
      if (noScrollTableHeight > MAX_TABLE_HEIGHT)
      {
         tableFlags |= ImGuiTableFlags.ScrollY;
         tableHeight = MAX_TABLE_HEIGHT;
      }

      renderSettingsTable(style, tableFlags, tableHeight);
   }

   private void renderSettingsTable(ImGuiStyle style, int tableFlags, float height)
   {
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

         renderUniversalAdjustors();
         renderIndividualAdjustors();

         ImGui.endTable();
      }
   }

   private void renderUniversalAdjustors()
   {
      // Render universal adjusters
      ImGui.tableNextRow(ImGuiTableRowFlags.Headers);
      if (ImGui.tableNextColumn()) // Enable
      {
         ImGui.setNextItemWidth(-1.0f);
         if (ImGui.checkbox(labels.getHidden("enable universal"), universalEnable))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               classSettings[i].enable.set(universalEnable);
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
               classSettings[i].confidenceThreshold.set(universalConfidenceThreshold);
         }
      }

      if (ImGui.tableNextColumn()) // Mask Threshold
      {
         if (tableInputFloat(universalMaskThreshold, -10.0f, 10.0f))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               classSettings[i].maskThreshold.set(universalMaskThreshold);
         }
      }

      if (ImGui.tableNextColumn()) // Erosion kernel radius
      {
         if (tableInputInt(universalErosionKernelRadius, 0, 10))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               classSettings[i].erosionKernelRadius.set(universalErosionKernelRadius);
         }
      }

      if (ImGui.tableNextColumn()) // Outlier threshold
      {
         if (tableInputFloat(universalOutlierThreshold, 0.0f, 5.0f))
         {
            for (int i = 0; i < detectableObjectClassCount; ++i)
               classSettings[i].outlierThreshold.set(universalOutlierThreshold);
         }
      }
   }

   private void renderIndividualAdjustors()
   {
      for (int i = 0; i < detectableObjectClassCount; ++i)
      {
         if (classSettings[i].className.toLowerCase().contains(tableFilter.get().toLowerCase()))
         {
            ImGui.tableNextRow();
            classSettings[i].renderTableRow();
         }
      }
   }

   private static class RDXYOLOv8ModelClassSettings
   {
      private final SyncedYOLOv8ModelParameters modelParameters;

      final int classIndex;
      final String className;

      private final ImGuiUniqueLabelMap labels;
      final ImBoolean enable;
      final ImFloat confidenceThreshold;
      final ImFloat maskThreshold;
      final ImInt erosionKernelRadius;
      final ImFloat outlierThreshold;

      public RDXYOLOv8ModelClassSettings(int classIndex, String className, SyncedYOLOv8ModelParameters modelParameters)
      {
         this.classIndex = classIndex;
         this.className = className;
         this.modelParameters = modelParameters;

         labels = new ImGuiUniqueLabelMap(getClass());
         enable = new ImBoolean(true);
         confidenceThreshold = new ImFloat(0.7f);
         maskThreshold = new ImFloat(0.0f);
         erosionKernelRadius = new ImInt(1);
         outlierThreshold = new ImFloat(2.0f);
      }

      public void update()
      {
         if (modelParameters.isModified())
         {
            enable.set(!modelParameters.getIgnoredObjectClasses().getValueReadOnly(classIndex));
            confidenceThreshold.set(modelParameters.getConfidenceThresholds().getValueReadOnly(classIndex));
            maskThreshold.set(modelParameters.getMaskThresholds().getValueReadOnly(classIndex));
            erosionKernelRadius.set(modelParameters.getErosionKernelRadii().getValueReadOnly(classIndex));
            outlierThreshold.set(modelParameters.getOutlierThresholds().getValueReadOnly(classIndex));
         }

         modelParameters.getIgnoredObjectClasses().setValue(classIndex, !enable.get());
         modelParameters.getConfidenceThresholds().setValue(classIndex, confidenceThreshold.get());
         modelParameters.getMaskThresholds().setValue(classIndex, maskThreshold.get());
         modelParameters.getErosionKernelRadii().setValue(classIndex, erosionKernelRadius.get());
         modelParameters.getOutlierThresholds().setValue(classIndex, outlierThreshold.get());
      }

      public void renderTableRow()
      {
         if (ImGui.tableNextColumn()) // Enable
         {
            ImGui.setNextItemWidth(-1.0f);
            ImGui.checkbox(labels.getHidden("enable"), enable);
            ImGui.setItemAllowOverlap();
         }

         if (ImGui.tableNextColumn()) // Object class
         {
            ImGui.setNextItemWidth(-1.0f);
            ImGui.text(className);
         }

         if (ImGui.tableNextColumn()) // Confidence threshold
            tableInputFloat(confidenceThreshold, 0.0f, 1.0f);

         if (ImGui.tableNextColumn()) // Mask threshold
            tableInputFloat(maskThreshold, -10.0f, 10.0f);

         if (ImGui.tableNextColumn()) // Erosion kernel radius
            tableInputInt(erosionKernelRadius, 0, 10);

         if (ImGui.tableNextColumn()) // Outlier threshold
            tableInputFloat(outlierThreshold, 0.0f, 5.0f);
      }
   }

   private static boolean tableInputFloat(ImFloat value, float min, float max)
   {
      String label = "###" + ImGui.tableGetColumnName() + ":" + ImGui.tableGetColumnIndex() + ":" + ImGui.tableGetRowIndex();

      ImGui.setNextItemWidth(-1.0f);
      boolean changed = ImGui.inputFloat(label, value, 0.0f, 0.0f, "%.2f", ImGuiInputTextFlags.EnterReturnsTrue);
      ImGui.setItemAllowOverlap();

      if (changed && min < max)
         value.set((float) MathTools.clamp(value.get(), min, max));

      return changed;
   }

   private static boolean tableInputInt(ImInt value, int min, int max)
   {
      String label = "###" + ImGui.tableGetColumnName() + ":" + ImGui.tableGetColumnIndex() + ":" + ImGui.tableGetRowIndex();

      ImGui.setNextItemWidth(-1.0f);
      boolean changed = ImGui.inputInt(label, value, 0, 0, ImGuiInputTextFlags.EnterReturnsTrue);
      ImGui.setItemAllowOverlap();

      if (changed && min < max)
         value.set(MathTools.clamp(value.get(), min, max));

      return changed;
   }
}
