package us.ihmc.gdx.ui;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.tools.property.*;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeSet;

public class ImGuiStoredPropertySetTuner extends ImGuiPanel
{
   private StoredPropertySetBasics storedPropertySet;
   private StoredPropertyKeyList keys;
   private Runnable onParametersUpdatedCallback;
   private final TreeSet<String> versions = new TreeSet<>();

   private final HashMap<DoubleStoredPropertyKey, ImDouble> doubleValues = new HashMap<>();
   private final HashMap<IntegerStoredPropertyKey, ImInt> integerValues = new HashMap<>();
   private final HashMap<BooleanStoredPropertyKey, ImBoolean> booleanValues = new HashMap<>();

   public ImGuiStoredPropertySetTuner(String name)
   {
      super(name);
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void create(StoredPropertySetBasics storedPropertySet, StoredPropertyKeyList keys, Runnable onParametersUpdatedCallback)
   {
      this.storedPropertySet = storedPropertySet;
      this.keys = keys;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;

      Path saveFileDirectory = storedPropertySet.findSaveFileDirectory();
      PathTools.walkFlat(saveFileDirectory, (path, pathType) ->
      {
         String fileName = path.getFileName().toString();
         String prefix = storedPropertySet.getUncapitalizedClassName();
         String extension = ".ini";
         if (pathType == BasicPathVisitor.PathType.FILE && fileName.startsWith(prefix) && fileName.endsWith(extension))
         {
            versions.add(fileName.replaceAll(extension, "").substring(prefix.length()));
         }
         return FileVisitResult.CONTINUE;
      });

      for (StoredPropertyKey<?> propertyKey : keys.keys())
      {
         // Add supported types here
         if (propertyKey.getType().equals(Double.class))
         {
            DoubleStoredPropertyKey key = (DoubleStoredPropertyKey) propertyKey;
            doubleValues.put(key, new ImDouble(storedPropertySet.get(key)));
         }
         else if (propertyKey.getType().equals(Integer.class))
         {
            IntegerStoredPropertyKey key = (IntegerStoredPropertyKey) propertyKey;
            integerValues.put(key, new ImInt(storedPropertySet.get(key)));
         }
         else if (propertyKey.getType().equals(Boolean.class))
         {
            BooleanStoredPropertyKey key = (BooleanStoredPropertyKey) propertyKey;
            booleanValues.put(key, new ImBoolean(storedPropertySet.get(key)));
         }
         else
         {
            throw new RuntimeException("Please implement spinner for type: " + propertyKey.getType());
         }
      }
   }

   public void renderImGuiWidgets()
   {
      for (String version : versions)
      {
         if (ImGui.radioButton(version.isEmpty() ? "Primary" : version, storedPropertySet.getCurrentVersionSuffix().equals(version)))
         {
            storedPropertySet.updateBackingSaveFile(version);
            load();
         }
      }

      //      ImGuiInputTextFlags. // TODO: Mess with various flags
      ImGui.pushItemWidth(150.0f);
      for (StoredPropertyKey<?> propertyKey : keys.keys())
      {
         if (propertyKey.getType().equals(Double.class))
         {
            if (ImGui.inputDouble(propertyKey.getTitleCasedName(), doubleValues.get(propertyKey), 0.01, 0.5))
            {
               DoubleStoredPropertyKey key = (DoubleStoredPropertyKey) propertyKey;
               storedPropertySet.set(key, doubleValues.get(key).get());
               onParametersUpdatedCallback.run();
            }
         }
         else if (propertyKey.getType().equals(Integer.class))
         {
            if (ImGui.inputInt(propertyKey.getTitleCasedName(), integerValues.get(propertyKey), 1))
            {
               IntegerStoredPropertyKey key = (IntegerStoredPropertyKey) propertyKey;
               storedPropertySet.set(key, integerValues.get(key).get());
               onParametersUpdatedCallback.run();
            }
         }
         else if (propertyKey.getType().equals(Boolean.class))
         {
            if (ImGui.checkbox(propertyKey.getTitleCasedName(), booleanValues.get(propertyKey)))
            {
               BooleanStoredPropertyKey key = (BooleanStoredPropertyKey) propertyKey;
               storedPropertySet.set(key, booleanValues.get(key).get());
               onParametersUpdatedCallback.run();
            }
         }
      }
      if (ImGui.button("Load"))
      {
         load();
      }
      ImGui.sameLine();
      if (ImGui.button("Save"))
      {
         storedPropertySet.save();
      }
   }

   private void load()
   {
      storedPropertySet.load();
      for (Map.Entry<DoubleStoredPropertyKey, ImDouble> entry : doubleValues.entrySet())
      {
         entry.getValue().set(storedPropertySet.get(entry.getKey()));
      }
      for (Map.Entry<IntegerStoredPropertyKey, ImInt> entry : integerValues.entrySet())
      {
         entry.getValue().set(storedPropertySet.get(entry.getKey()));
      }
      for (Map.Entry<BooleanStoredPropertyKey, ImBoolean> entry : booleanValues.entrySet())
      {
         entry.getValue().set(storedPropertySet.get(entry.getKey()));
      }

      onParametersUpdatedCallback.run();
   }

   public <T> void changeParameter(StoredPropertyKey<T> key, T value)
   {
      storedPropertySet.set(key, value);
      onParametersUpdatedCallback.run();
   }

   public StoredPropertySetReadOnly getParameters()
   {
      return storedPropertySet;
   }
}
