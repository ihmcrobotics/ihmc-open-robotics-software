package us.ihmc.rdx.ui;

import imgui.ImGui;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.rdx.imgui.*;
import us.ihmc.tools.property.*;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeSet;

public class ImGuiStoredPropertySetTuner extends ImGuiPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private StoredPropertySetBasics storedPropertySet;
   private Runnable onParametersUpdatedCallback;
   private final Runnable onParametersUpdatedCallbackAndMore = this::onParametersUpdatedCallbackAndMore;
   private boolean anyParameterChanged = false;
   private final TreeSet<String> versions = new TreeSet<>();

   private final ArrayList<ImGuiStoredPropertySetWidget> imGuiWidgetRenderers = new ArrayList<>();
   private final HashMap<StoredPropertyKey<?>, ImGuiStoredPropertySetWidget> imGuiWidgetRendererMap = new HashMap<>();

   public ImGuiStoredPropertySetTuner(String name)
   {
      super(name);
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void create(StoredPropertySetBasics storedPropertySet)
   {
      create(storedPropertySet, () -> { });
   }

   public void create(StoredPropertySetBasics storedPropertySet, Runnable onParametersUpdatedCallback)
   {
      this.storedPropertySet = storedPropertySet;
      this.onParametersUpdatedCallback = onParametersUpdatedCallback;

      Path saveFileDirectory = storedPropertySet.findSaveFileDirectory();
      PathTools.walkFlat(saveFileDirectory, (path, pathType) ->
      {
         String fileName = path.getFileName().toString();
         String prefix = storedPropertySet.getCapitalizedClassName();
         String extension = ".json";
         if (pathType == BasicPathVisitor.PathType.FILE && fileName.startsWith(prefix) && fileName.endsWith(extension))
         {
            versions.add(fileName.replaceAll(extension, "").substring(prefix.length()));
         }
         return FileVisitResult.CONTINUE;
      });
      String currentWorkingVersion = versions.first();
      if (!storedPropertySet.getCurrentVersionSuffix().equals(currentWorkingVersion))
      {
         storedPropertySet.updateBackingSaveFile(currentWorkingVersion);
         storedPropertySet.load();
      }

      for (StoredPropertyKey<?> propertyKey : storedPropertySet.getKeyList().keys())
      {
         ImGuiStoredPropertySetWidget widget;
         if (propertyKey instanceof DoubleStoredPropertyKey doubleKey)
         {
            String format = "%.6f";
            double step = 0.01;
            double stepFast = 0.5;
            widget = new ImGuiStoredPropertySetDoubleWidget(storedPropertySet, doubleKey, step, stepFast, format, onParametersUpdatedCallbackAndMore);
         }
         else if (propertyKey instanceof IntegerStoredPropertyKey integerKey)
         {
            int step = 1;
            widget = new ImGuiStoredPropertySetIntegerWidget(storedPropertySet, integerKey, step, onParametersUpdatedCallbackAndMore);
         }
         else if (propertyKey instanceof BooleanStoredPropertyKey booleanKey)
         {
            widget = new ImGuiStoredPropertySetBooleanWidget(storedPropertySet, booleanKey, onParametersUpdatedCallbackAndMore);
         }
         else
         {
            throw new RuntimeException("Please implement spinner for type: " + propertyKey.getType());
         }
         imGuiWidgetRenderers.add(widget);
         imGuiWidgetRendererMap.put(propertyKey, widget);
      }
   }

   /**
    * @return if any parameter changed by the ImGui user
    */
   public boolean renderImGuiWidgets()
   {
      ImGui.text(storedPropertySet.getTitle());
      ImGui.text("Version:");
      if (versions.size() > 1)
      {
         for (String version : versions)
         {
            if (ImGui.radioButton(version.isEmpty() ? "Primary" : version, storedPropertySet.getCurrentVersionSuffix().equals(version)))
            {
               storedPropertySet.updateBackingSaveFile(version);
               load();
            }
         }
      }
      else
      {
         ImGui.sameLine();
         ImGui.text(storedPropertySet.getCurrentVersionSuffix());
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

      ImGui.text("(Ctrl + click sliders to set exact and unbounded value.)");

      for (ImGuiStoredPropertySetWidget widget : imGuiWidgetRenderers)
      {
         widget.render();
      }

      boolean returnAnyChanged = anyParameterChanged;
      anyParameterChanged = false;
      return returnAnyChanged;
   }

   public boolean changed(StoredPropertyKey<?> key)
   {
      return imGuiWidgetRendererMap.get(key).changed();
   }

   public ImGuiStoredPropertySetDoubleWidget createDoubleSlider(DoubleStoredPropertyKey doubleKey,
                                                                double step,
                                                                double stepFast,
                                                                String unitString,
                                                                String format)
   {
      return new ImGuiStoredPropertySetDoubleWidget(storedPropertySet, doubleKey, step, stepFast, format, unitString, onParametersUpdatedCallback);
   }

   public ImGuiStoredPropertySetDoubleWidget createDoubleSlider(DoubleStoredPropertyKey key, double min, double max)
   {
      return new ImGuiStoredPropertySetDoubleWidget(storedPropertySet, key, min, max, onParametersUpdatedCallback);
   }

   private void onParametersUpdatedCallbackAndMore()
   {
      anyParameterChanged = true;
      onParametersUpdatedCallback.run();
   }

   private void load()
   {
      storedPropertySet.load();

      onParametersUpdatedCallback.run();
   }

   public <T> void changeParameter(StoredPropertyKey<T> key, T value)
   {
      storedPropertySet.set(key, value);
      onParametersUpdatedCallback.run();
   }
}
