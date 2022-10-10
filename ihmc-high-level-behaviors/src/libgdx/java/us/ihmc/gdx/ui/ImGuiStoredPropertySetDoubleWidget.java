package us.ihmc.gdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.gdx.imgui.ImDoubleWrapper;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.function.Consumer;

public class ImGuiStoredPropertySetDoubleWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String label;
   private final float min;
   private final float max;
   private final Runnable onParametersUpdatedCallback;
   private final ImDoubleWrapper imDoubleWrapper;
   private final Consumer<ImDouble> accessImDouble;

   public ImGuiStoredPropertySetDoubleWidget(DoubleStoredPropertyKey key,
                                             StoredPropertySetBasics storedPropertySet,
                                             double min,
                                             double max,
                                             Runnable onParametersUpdatedCallback)
   {
      this.min = (float) min;
      this.max = (float) max;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      imDoubleWrapper = new ImDoubleWrapper(storedPropertySet, key);
      label = labels.get(key.getTitleCasedName());
      accessImDouble = this::renderSliderWithMinMax;
   }

   public void build()
   {

   }

   public void render()
   {
      imDoubleWrapper.accessImDouble(accessImDouble);
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
}
