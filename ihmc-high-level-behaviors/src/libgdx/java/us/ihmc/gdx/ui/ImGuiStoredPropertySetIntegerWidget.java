package us.ihmc.gdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImInt;
import us.ihmc.commons.MathTools;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.imgui.ImIntegerWrapper;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.Consumer;

public class ImGuiStoredPropertySetIntegerWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String label;
   private final int min;
   private final int max;
   private final Runnable onParametersUpdatedCallback;
   private final ImIntegerWrapper imIntegerWrapper;
   private final Consumer<ImInt> accessImInt;
   private int step;

   public ImGuiStoredPropertySetIntegerWidget(StoredPropertySetBasics storedPropertySet,
                                              IntegerStoredPropertyKey key,
                                              int min,
                                              int max,
                                              Runnable onParametersUpdatedCallback)
   {
      this.min = min;
      this.max = max;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      imIntegerWrapper = new ImIntegerWrapper(storedPropertySet, key);
      label = labels.get(key.getTitleCasedName());
      accessImInt = this::renderSliderWithMinMax;
   }

   public ImGuiStoredPropertySetIntegerWidget(StoredPropertySetBasics storedPropertySet,
                                              IntegerStoredPropertyKey key,
                                              int step,
                                              Runnable onParametersUpdatedCallback)
   {
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      imIntegerWrapper = new ImIntegerWrapper(storedPropertySet, key);
      label = labels.get(key.getTitleCasedName());

      if (key.hasLowerBound() && key.hasUpperBound())
      {
         this.min = key.getLowerBound();
         this.max = key.getUpperBound();
         accessImInt = this::renderSliderWithMinMax;
      }
      else
      {
         this.step = step;
         min = Integer.MIN_VALUE;
         max = Integer.MAX_VALUE;
         accessImInt = this::renderInputWithStep;
      }
   }

   public void render()
   {
      imIntegerWrapper.accessImInt(accessImInt);
   }

   private void renderInputWithStep(ImInt imInteger)
   {
      if (ImGuiTools.volatileInputInt(label, imInteger, step))
      {
         onParametersUpdatedCallback.run();
      }
   }

   private void renderSliderWithMinMax(ImInt imInteger)
   {
      if (ImGui.sliderScalar(label, ImGuiDataType.S32, imInteger, min, max))
      {
         onParametersUpdatedCallback.run();
      }
   }

   private void renderSliderWithMinMaxAndClamp(ImInt imInteger)
   {
      if (ImGui.sliderScalar(label, ImGuiDataType.S32, imInteger, min, max))
      {
         clamp(imInteger); // TODO: In what case is this necessary?
         onParametersUpdatedCallback.run();
      }
   }

   private void clamp(ImInt imInteger)
   {
      imInteger.set(MathTools.clamp(imInteger.get(), min, max));
   }
}
