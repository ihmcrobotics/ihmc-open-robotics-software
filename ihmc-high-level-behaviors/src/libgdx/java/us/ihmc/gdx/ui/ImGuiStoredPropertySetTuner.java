package us.ihmc.gdx.ui;

import imgui.ImGui;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.gdx.imgui.*;
import us.ihmc.tools.property.*;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.TreeSet;

public class ImGuiStoredPropertySetTuner extends ImGuiPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private StoredPropertySetBasics storedPropertySet;
   private Runnable onParametersUpdatedCallback;
   private final TreeSet<String> versions = new TreeSet<>();

   private final ArrayList<Runnable> imGuiWidgetRenderers = new ArrayList<>();

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
         if (propertyKey instanceof DoubleStoredPropertyKey doubleKey)
         {
            String format = "%.6f";
            double step = 0.01;
            double stepFast = 0.5;
            ImGuiStoredPropertySetDoubleWidget widget = new ImGuiStoredPropertySetDoubleWidget(storedPropertySet,
                                                                                               doubleKey,
                                                                                               step,
                                                                                               stepFast,
                                                                                               format,
                                                                                               onParametersUpdatedCallback);
            imGuiWidgetRenderers.add(widget::render);
         }
         else if (propertyKey instanceof IntegerStoredPropertyKey integerKey)
         {
            int step = 1;
            ImGuiStoredPropertySetIntegerWidget widget = new ImGuiStoredPropertySetIntegerWidget(storedPropertySet,
                                                                                                 integerKey,
                                                                                                 step,
                                                                                                 onParametersUpdatedCallback);
            imGuiWidgetRenderers.add(widget::render);
         }
         else if (propertyKey instanceof BooleanStoredPropertyKey booleanKey)
         {
            ImGuiStoredPropertySetBooleanWidget widget = new ImGuiStoredPropertySetBooleanWidget(storedPropertySet, booleanKey, onParametersUpdatedCallback);
            imGuiWidgetRenderers.add(widget::render);
         }
         else
         {
            throw new RuntimeException("Please implement spinner for type: " + propertyKey.getType());
         }
      }
   }

   public void renderImGuiWidgets()
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

      for (Runnable imGuiWidgetRenderer : imGuiWidgetRenderers)
      {
         imGuiWidgetRenderer.run();
      }
   }

   public Runnable createDoubleSlider(DoubleStoredPropertyKey doubleKey, double step, double stepFast, String unitString, String format)
   {
      ImGuiStoredPropertySetDoubleWidget widget;
      widget = new ImGuiStoredPropertySetDoubleWidget(storedPropertySet, doubleKey, step, stepFast, format, unitString, onParametersUpdatedCallback);
      return widget::render;
   }

   public ImGuiStoredPropertySetDoubleWidget createDoubleSlider(DoubleStoredPropertyKey key, double min, double max)
   {
      return new ImGuiStoredPropertySetDoubleWidget(storedPropertySet, key, min, max, onParametersUpdatedCallback);
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
