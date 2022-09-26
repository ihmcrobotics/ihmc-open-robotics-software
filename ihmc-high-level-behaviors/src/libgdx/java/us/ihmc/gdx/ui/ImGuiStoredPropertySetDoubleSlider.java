package us.ihmc.gdx.ui;

import imgui.ImGui;
import us.ihmc.commons.MathTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class ImGuiStoredPropertySetDoubleSlider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DoubleStoredPropertyKey key;
   private final StoredPropertySetBasics storedPropertySet;
   private final float min;
   private final float max;
   private final Runnable onParametersUpdatedCallback;
   private final float[] valueAsArray = new float[] { 0.0f };

   public ImGuiStoredPropertySetDoubleSlider(DoubleStoredPropertyKey key,
                                             StoredPropertySetBasics storedPropertySet,
                                             double min,
                                             double max,
                                             Runnable onParametersUpdatedCallback)
   {
      this.key = key;
      this.storedPropertySet = storedPropertySet;
      this.min = (float) min;
      this.max = (float) max;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
   }

   public void render()
   {
      valueAsArray[0] = (float) storedPropertySet.get(key);
      if (ImGui.sliderFloat(labels.get(key.getTitleCasedName()), valueAsArray, min, max))
      {
         if (!Double.isNaN(min))
            valueAsArray[0] = (float) MathTools.clamp(valueAsArray[0], min, max);
         storedPropertySet.set(key, valueAsArray[0]);
         onParametersUpdatedCallback.run();
      }
   }
}
