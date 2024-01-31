package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiDataType;
import imgui.type.ImInt;
import us.ihmc.commons.MathTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.Consumer;

public class ImGuiStoredPropertySetIntegerWidget implements ImGuiStoredPropertySetWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String label;
   private final int min;
   private final int max;
   private final Runnable onParametersUpdatedCallback;
   private final ImIntegerWrapper imIntegerWrapper;
   private int step;
   private String fancyPrefixLabel;
   private float fancyPrefixWidth;
   private boolean widthsCalculated = false;

   public ImGuiStoredPropertySetIntegerWidget(StoredPropertySetBasics storedPropertySet,
                                              IntegerStoredPropertyKey key,
                                              int min,
                                              int max,
                                              Runnable onParametersUpdatedCallback)
   {
      this.min = min;
      this.max = max;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      imIntegerWrapper = new ImIntegerWrapper(storedPropertySet, key, this::renderSliderWithMinMax);
      label = labels.get(key.getTitleCasedName());
   }

   public ImGuiStoredPropertySetIntegerWidget(StoredPropertySetBasics storedPropertySet,
                                              IntegerStoredPropertyKey key,
                                              int step,
                                              Runnable onParametersUpdatedCallback)
   {
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      label = labels.getHidden(key.getTitleCasedName());
      fancyPrefixLabel = key.getTitleCasedName() + ":";

      Consumer<ImInt> widgetRenderer;
      if (key.hasLowerBound() && key.hasUpperBound())
      {
         this.min = key.getLowerBound();
         this.max = key.getUpperBound();
         widgetRenderer = this::renderSliderWithMinMax;
      }
      else
      {
         this.step = step;
         min = Integer.MIN_VALUE;
         max = Integer.MAX_VALUE;
         widgetRenderer = this::renderInputWithStep;
      }

      imIntegerWrapper = new ImIntegerWrapper(storedPropertySet, key, widgetRenderer);
   }

   @Override
   public void renderImGuiWidget()
   {
      imIntegerWrapper.renderImGuiWidget();
   }

   @Override
   public boolean changed()
   {
      return imIntegerWrapper.changed();
   }

   private void renderInputWithStep(ImInt imInteger)
   {
      fancyBefore();
      if (ImGuiTools.volatileInputInt(label, imInteger, step))
      {
         onParametersUpdatedCallback.run();
      }
      fancyAfter();
   }

   private void renderSliderWithMinMax(ImInt imInteger)
   {
      fancyBefore();
      if (ImGui.sliderScalar(label, ImGuiDataType.S32, imInteger, min, max))
      {
         onParametersUpdatedCallback.run();
      }
      fancyAfter();
   }

   private void renderSliderWithMinMaxAndClamp(ImInt imInteger)
   {
      fancyBefore();
      if (ImGui.sliderScalar(label, ImGuiDataType.S32, imInteger, min, max))
      {
         clamp(imInteger); // TODO: In what case is this necessary?
         onParametersUpdatedCallback.run();
      }
      fancyAfter();
   }

   private void clamp(ImInt imInteger)
   {
      imInteger.set(MathTools.clamp(imInteger.get(), min, max));
   }

   private void fancyBefore()
   {
      if (!widthsCalculated)
      {
         widthsCalculated = true;
         ImVec2 size = new ImVec2();
         ImGui.calcTextSize(size, fancyPrefixLabel);
         fancyPrefixWidth = size.x;
      }

      ImGui.text(fancyPrefixLabel);
      ImGui.sameLine();
      ImGui.pushItemWidth(ImGuiTools.getUsableWindowWidth() - fancyPrefixWidth);
   }

   private void fancyAfter()
   {
      ImGui.popItemWidth();
   }
}
