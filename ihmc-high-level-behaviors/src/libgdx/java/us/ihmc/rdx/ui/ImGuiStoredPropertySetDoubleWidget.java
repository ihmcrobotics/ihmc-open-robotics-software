package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiDataType;
import imgui.type.ImDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.Consumer;

/**
 * TODO: This class would benefit from a different structure.
 *   Instead of the slightly different constructors, maybe it should have a series
 *   of methods that can be called to configure it after construction.
 */
public class ImGuiStoredPropertySetDoubleWidget implements ImGuiStoredPropertySetWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String label;
   private final double min;
   private final double max;
   private String unitString;
   private final Runnable onParametersUpdatedCallback;
   private final ImDoubleWrapper imDoubleWrapper;
   private double step;
   private double stepFast;
   private DoubleStoredPropertyKey key;
   private String format;
   private String fancyPrefixLabel;
   private float unitStringWidth;
   private float fancyPrefixWidth;
   private boolean widthsCalculated = false;

   public ImGuiStoredPropertySetDoubleWidget(StoredPropertySetBasics storedPropertySet,
                                             DoubleStoredPropertyKey key,
                                             double min,
                                             double max,
                                             Runnable onParametersUpdatedCallback)
   {
      this.min = min;
      this.max = max;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      label = labels.get(key.getTitleCasedName());
      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key, this::renderSliderWithMinMax);
   }

   public ImGuiStoredPropertySetDoubleWidget(StoredPropertySetBasics storedPropertySet,
                                             DoubleStoredPropertyKey key,
                                             double step,
                                             double stepFast,
                                             String format,
                                             Runnable onParametersUpdatedCallback)
   {
      this.key = key;
      this.min = key.getLowerBound();
      this.max = key.getUpperBound();
      this.format = format;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      label = labels.getHidden(key.getTitleCasedName());
      fancyPrefixLabel = key.getTitleCasedName() + ":";

      Consumer<ImDouble> widgetRenderer;
      if (key.hasLowerBound() && key.hasUpperBound())
      {
         widgetRenderer = this::renderSliderWithMinMaxAndFormatFancy;
      }
      else
      {
         this.step = step;
         this.stepFast = stepFast;
         widgetRenderer = this::renderInputWithStepAndStepFastFancy;
      }

      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key, widgetRenderer);
   }

   public ImGuiStoredPropertySetDoubleWidget(StoredPropertySetBasics storedPropertySet,
                                             DoubleStoredPropertyKey key,
                                             double step,
                                             double stepFast,
                                             String format,
                                             String unitString,
                                             Runnable onParametersUpdatedCallback)
   {
      this(storedPropertySet, key, step, stepFast, format, unitString, onParametersUpdatedCallback, false);
   }

   public ImGuiStoredPropertySetDoubleWidget(StoredPropertySetBasics storedPropertySet,
                                             DoubleStoredPropertyKey key,
                                             double step,
                                             double stepFast,
                                             String format,
                                             String unitString,
                                             Runnable onParametersUpdatedCallback,
                                             boolean enforceInputWidget)
   {
      this.key = key;
      this.min = key.getLowerBound();
      this.max = key.getUpperBound();
      this.format = format;
      this.unitString = unitString;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      label = labels.get(unitString, key.getTitleCasedName());
      fancyPrefixLabel = key.getTitleCasedName() + ":";

      Consumer<ImDouble> widgetRenderer;
      if (!enforceInputWidget && key.hasLowerBound() && key.hasUpperBound())
      {
         widgetRenderer = this::renderSliderWithMinMaxAndFormatFancy;
      }
      else
      {
         this.step = step;
         this.stepFast = stepFast;
         widgetRenderer = this::renderInputWithStepAndStepFastFancy;
      }

      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key, widgetRenderer);
   }

   @Override
   public void renderImGuiWidget()
   {
      imDoubleWrapper.renderImGuiWidget();
   }

   @Override
   public boolean changed()
   {
      return imDoubleWrapper.changed();
   }

   private void renderInputWithStepAndStepFastFancy(ImDouble imDouble)
   {
      if (!widthsCalculated)
      {
         widthsCalculated = true;
         ImVec2 size = new ImVec2();
         ImGui.calcTextSize(size, fancyPrefixLabel);
         fancyPrefixWidth = size.x;
         if (unitString != null)
         {
            ImGui.calcTextSize(size, unitString);
            unitStringWidth = size.x;
         }
      }

      ImGui.text(fancyPrefixLabel);
      ImGui.sameLine();
      float columnWidth = ImGuiTools.getUsableWindowWidth();
      ImGui.pushItemWidth(columnWidth - fancyPrefixWidth - unitStringWidth);
      renderInputWithStepAndStepFast(imDouble);
      ImGui.popItemWidth();
   }

   private void renderInputWithStepAndStepFast(ImDouble imDouble)
   {
      if (ImGuiTools.volatileInputDouble(label, imDouble, step, stepFast, format))
      {
         clamp(imDouble);
         onParametersUpdatedCallback.run();
      }
   }

   private void renderSliderWithMinMaxAndFormatFancy(ImDouble imDouble)
   {
      fancyBefore();
      renderSliderWithMinMaxAndFormat(imDouble);
      fancyAfter();
   }

   private void renderSliderWithMinMaxAndFormat(ImDouble imDouble)
   {
      if (ImGui.sliderScalar(label, ImGuiDataType.Double, imDouble, min, max, format))
      {
         onParametersUpdatedCallback.run();
      }
   }

   private void renderSliderWithMinMax(ImDouble imDouble)
   {
      if (ImGui.sliderScalar(label, ImGuiDataType.Double, imDouble, min, max))
      {
         //         clamp(imDouble); // TODO: In what case is this necessary?
         onParametersUpdatedCallback.run();
      }
   }

   private void clamp(ImDouble imDouble)
   {
      if (!Double.isNaN(min) && !Double.isNaN(max))
         imDouble.set(MathTools.clamp(imDouble.get(), min, max));
   }

   private void fancyBefore()
   {
      if (!widthsCalculated)
      {
         widthsCalculated = true;
         ImVec2 size = new ImVec2();
         ImGui.calcTextSize(size, fancyPrefixLabel);
         fancyPrefixWidth = size.x;
         if (unitString != null)
         {
            ImGui.calcTextSize(size, unitString);
            unitStringWidth = size.x;
         }
      }

      ImGui.text(fancyPrefixLabel);
      ImGui.sameLine();
      float columnWidth = ImGuiTools.getUsableWindowWidth();
      ImGui.pushItemWidth(columnWidth - fancyPrefixWidth - unitStringWidth);
   }

   private void fancyAfter()
   {
      ImGui.popItemWidth();
   }
}
