package us.ihmc.rdx.ui;

import imgui.ImGui;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class ImGuiStoredPropertySetEnumWidget implements ImGuiStoredPropertySetWidget
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final StoredPropertySetBasics storedPropertySet;
   private final IntegerStoredPropertyKey key;
   private final Enum<?>[] enumValues;
   private final Runnable onParametersUpdatedCallback;
   private final String keyNameText;
   private final Notification changed = new Notification();

   public ImGuiStoredPropertySetEnumWidget(StoredPropertySetBasics storedPropertySet,
                                           IntegerStoredPropertyKey key,
                                           Enum<?>[] enumValues,
                                           Runnable onParametersUpdatedCallback)
   {
      this.storedPropertySet = storedPropertySet;
      this.key = key;
      this.enumValues = enumValues;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;
      keyNameText = key.getTitleCasedName() + ":";
   }

   @Override
   public void renderImGuiWidget()
   {
      ImGui.text(keyNameText);

      int keyValue = storedPropertySet.get(key);
      for (int i = 0; i < enumValues.length; i++)
      {
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get(enumValues[i].name()), keyValue == enumValues[i].ordinal()))
         {
            if (enumValues[i].ordinal() != keyValue)
            {
               storedPropertySet.set(key, enumValues[i].ordinal());
               onParametersUpdatedCallback.run();
               changed.set();
            }
         }
      }
   }

   @Override
   public boolean changed()
   {
      return changed.poll();
   }
}
