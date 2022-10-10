package us.ihmc.gdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.gdx.imgui.ImDoubleWrapper;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.Consumer;

public class ImGuiStoredPropertySetDoubleWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String label;
   private final double min;
   private final double max;
   private final Runnable onParametersUpdatedCallback;
   private final ImDoubleWrapper imDoubleWrapper;
   private final Consumer<ImDouble> accessImDouble;
   private double step;
   private double stepFast;
   private DoubleStoredPropertyKey key;
   private String format;

   public ImGuiStoredPropertySetDoubleWidget(StoredPropertySetBasics storedPropertySet,
                                             DoubleStoredPropertyKey key,
                                             double min,
                                             double max,
                                             Runnable onParametersUpdatedCallback)
   {
      this.min = min;
      this.max = max;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key);
      label = labels.get(key.getTitleCasedName());
      accessImDouble = this::renderSliderWithMinMax;
   }

   public ImGuiStoredPropertySetDoubleWidget(StoredPropertySetBasics storedPropertySet,
                                             DoubleStoredPropertyKey key,
                                             double step,
                                             double stepFast,
                                             String format,
                                             Runnable onParametersUpdatedCallback)
   {
      this.format = format;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key);
      label = labels.get(key.getTitleCasedName());

      if (key.hasLowerBound() && key.hasUpperBound())
      {
         this.min = key.getLowerBound();
         this.max = key.getUpperBound();
         accessImDouble = this::renderSliderWithMinMaxAndFormat;
      }
      else
      {
         this.step = step;
         this.stepFast = stepFast;
         min = Double.NaN;
         max = Double.NaN;
         accessImDouble = this::renderInputWithStepAndStepFast;
      }
   }

   public ImGuiStoredPropertySetDoubleWidget(StoredPropertySetBasics storedPropertySet,
                                             DoubleStoredPropertyKey key,
                                             double step,
                                             double stepFast,
                                             String format,
                                             String unitString,
                                             Runnable onParametersUpdatedCallback)
   {
      this.key = key;
      this.format = format;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key);
      label = labels.get(unitString, key.getTitleCasedName());

      if (key.hasLowerBound() && key.hasUpperBound())
      {
         this.min = key.getLowerBound();
         this.max = key.getUpperBound();
         accessImDouble = this::renderSliderWithMinMaxAndFormatFancy;
      }
      else
      {
         this.step = step;
         this.stepFast = stepFast;
         min = Double.NaN;
         max = Double.NaN;
         accessImDouble = this::renderInputWithStepAndStepFastFancy;
      }
   }

   public void render()
   {
      imDoubleWrapper.accessImDouble(accessImDouble);
   }

   private void renderInputWithStepAndStepFastFancy(ImDouble imDouble)
   {
      fancyBefore();
      renderInputWithStepAndStepFast(imDouble);
      fancyAfter();
   }

   private void renderInputWithStepAndStepFast(ImDouble imDouble)
   {
      if (ImGuiTools.volatileInputDouble(label, imDouble, step, stepFast, format))
      {
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
         clamp(imDouble); // TODO: In what case is this necessary?
         onParametersUpdatedCallback.run();
      }
   }

   private void clamp(ImDouble imDouble)
   {
      imDouble.set(MathTools.clamp(imDouble.get(), min, max));
   }

   private void fancyBefore()
   {
      ImGui.text(key.getTitleCasedName() + ":");
      ImGui.sameLine();
      ImGui.pushItemWidth(100.0f);
   }

   private void fancyAfter()
   {
      ImGui.popItemWidth();
   }
}
