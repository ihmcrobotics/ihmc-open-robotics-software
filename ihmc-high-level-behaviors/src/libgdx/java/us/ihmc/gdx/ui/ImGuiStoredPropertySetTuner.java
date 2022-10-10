package us.ihmc.gdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import us.ihmc.commons.MathTools;
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
         String label = propertyKey.getTitleCasedName();

         if (propertyKey instanceof DoubleStoredPropertyKey doubleKey)
         {
            ImGuiDoubleWidget imGuiDoubleWidget = new ImGuiDoubleWidget(() -> storedPropertySet.get(doubleKey),
                                                                        doubleValue -> storedPropertySet.set(doubleKey, doubleValue),
                                                                        imDouble ->
            {
               boolean changed;
               String format = "%.6f";
               if (doubleKey.hasLowerBound() && doubleKey.hasUpperBound())
               {
                  changed = ImGui.sliderScalar(label,
                                               ImGuiDataType.Double,
                                               imDouble,
                                               doubleKey.getLowerBound(),
                                               doubleKey.getUpperBound(), format);
               }
               else
               {
                  double step = 0.01;
                  double stepFast = 0.5;
                  changed = ImGuiTools.volatileInputDouble(label, imDouble, step, stepFast, format);
               }

               if (changed)
                  onParametersUpdatedCallback.run();
            });
            imGuiWidgetRenderers.add(imGuiDoubleWidget::renderImGuiWidget);
         }
         else if (propertyKey instanceof IntegerStoredPropertyKey integerKey)
         {
            ImGuiIntegerWidget imGuiIntegerWidget = new ImGuiIntegerWidget(() -> storedPropertySet.get(integerKey),
                                                                           integerValue -> storedPropertySet.set(integerKey, integerValue),
                                                                           imInt ->
            {
               boolean changed;
               if (integerKey.hasLowerBound() && integerKey.hasUpperBound())
               {
                  changed = ImGui.sliderScalar(label,
                                               ImGuiDataType.S32,
                                               imInt,
                                               integerKey.getLowerBound(),
                                               integerKey.getUpperBound());
               }
               else
               {
                  int step = 1;
                  changed = ImGuiTools.volatileInputInt(label, imInt, step);
               }

               if (changed)
                  onParametersUpdatedCallback.run();
            });
            imGuiWidgetRenderers.add(imGuiIntegerWidget::renderImGuiWidget);
         }
         else if (propertyKey instanceof BooleanStoredPropertyKey booleanKey)
         {
            ImGuiBooleanWidget imGuiBooleanWidget = new ImGuiBooleanWidget(() -> storedPropertySet.get(booleanKey),
                                                                           booleanValue -> storedPropertySet.set(booleanKey, booleanValue),
                                                                           imBoolean ->
            {
               if (ImGui.checkbox(label, imBoolean))
               {
                  onParametersUpdatedCallback.run();
               }
            });
            imGuiWidgetRenderers.add(imGuiBooleanWidget::renderImGuiWidget);
         }
         else
         {
            throw new RuntimeException("Please implement spinner for type: " + propertyKey.getType());
         }
      }
   }

   public void renderImGuiWidgets()
   {
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

      //      ImGuiInputTextFlags. // TODO: Mess with various flags
      ImGui.pushItemWidth(150.0f);
      for (Runnable imGuiWidgetRenderer : imGuiWidgetRenderers)
      {
         imGuiWidgetRenderer.run();
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

   public Runnable createDoubleSlider(DoubleStoredPropertyKey doubleKey,
                                      double step,
                                      double stepFast,
                                      double min,
                                      double max,
                                      boolean fancyLabel,
                                      String unitString,
                                      String format)
   {
      String label = fancyLabel ? labels.get(unitString, doubleKey.getTitleCasedName()) : doubleKey.getTitleCasedName();
      ImGuiDoubleWidget imGuiDoubleWidget = new ImGuiDoubleWidget(storedPropertySet, doubleKey, imDouble ->
      {
         if (fancyLabel)
         {
            ImGui.text(doubleKey.getTitleCasedName() + ":");
            ImGui.sameLine();
            ImGui.pushItemWidth(100.0f);
         }

         boolean changed;
         if (doubleKey.hasLowerBound() && doubleKey.hasUpperBound())
         {
            changed = ImGui.sliderScalar(label,
                                         ImGuiDataType.Double,
                                         imDouble,
                                         doubleKey.getLowerBound(),
                                         doubleKey.getUpperBound(), format);
         }
         else
         {
            changed = ImGuiTools.volatileInputDouble(label, imDouble, step, stepFast, format);
         }

         if (changed)
         {
            imDouble.set(MathTools.clamp(imDouble.get(), min, max));
            onParametersUpdatedCallback.run();
         }

         if (fancyLabel)
         {
            ImGui.popItemWidth();
         }
      });
      return imGuiDoubleWidget::renderImGuiWidget;
   }

   public ImGuiStoredPropertySetDoubleWidget createDoubleSlider(DoubleStoredPropertyKey key, double min, double max)
   {
      return new ImGuiStoredPropertySetDoubleWidget(key, storedPropertySet, min, max, onParametersUpdatedCallback);
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
