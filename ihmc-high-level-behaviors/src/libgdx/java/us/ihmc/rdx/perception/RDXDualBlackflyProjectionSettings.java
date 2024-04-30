package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import java.util.concurrent.CompletableFuture;

public class RDXDualBlackflyProjectionSettings
{
   private static List<String> availablePresets = new ArrayList<>();

   static
   {
      // Every 2 seconds, refresh the available presets file list
      new Thread(() ->
      {
         while (RDXBaseUI.getInstance() != null)
         {
            List<String> newAvailablePresets = new ArrayList<>();
            File directory = IHMCCommonPaths.VR_DIRECTORY.toFile();
            directory.mkdirs();

            File[] files = directory.listFiles();

            if (files != null)
            {
               for (File file : files)
               {
                  if (file.isFile() && file.getName().endsWith(".properties"))
                  {
                     newAvailablePresets.add(file.getName());
                  }
               }
            }

            availablePresets = newAvailablePresets;

            ThreadTools.sleep(2000);
         }
      }, RDXDualBlackflyProjectionSettings.class.getSimpleName() + "FileRefresh").start();
   }

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private static int deleteRetries = 0;

   protected final ImBoolean projectionShapesLockOnRobot = new ImBoolean(false);
   protected final ImBoolean showLeftProjectionShape = new ImBoolean(true);
   protected final ImBoolean showRightProjectionShape = new ImBoolean(true);
   protected final ImDouble projectionXOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble projectionYOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble projectionZOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble projectionYawOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble projectionPitchOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble projectionRollOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble cameraXOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble cameraYOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble cameraZOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble cameraYawOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble cameraPitchOffsetCalibration = new ImDouble(0.0);
   protected final ImDouble cameraRollOffsetCalibration = new ImDouble(0.0);

   private final ImString presetFileName = new ImString();
   private final ImInt selectedPreset = new ImInt(0);
   private boolean unsaved = false;

   public RDXDualBlackflyProjectionSettings()
   {

   }

   public void save(RDXProjectionShape projectionShape, String presetFileName)
         throws IOException, NoSuchMethodException, InvocationTargetException, IllegalAccessException
   {
      if (!presetFileName.endsWith(".properties"))
         presetFileName += ".properties";

      RDXBaseUI.pushNotification("Saving projection settings preset: " + presetFileName);

      File vrSettingsPath = IHMCCommonPaths.VR_DIRECTORY.toFile();
      vrSettingsPath.mkdirs();

      File vrStereoVisionSettingsFile = IHMCCommonPaths.VR_DIRECTORY.resolve(presetFileName).toFile();

      if (!vrStereoVisionSettingsFile.exists())
         vrStereoVisionSettingsFile.createNewFile();

      Properties properties = new Properties();

      // Save fields in this class
      for (Field field : getClass().getDeclaredFields())
      {
         if (field.getType() == ImDouble.class || field.getType() == ImInt.class || field.getType() == ImBoolean.class)
         {
            field.setAccessible(true);
            Object fieldValue = field.get(this);
            properties.put(field.getName(), fieldValue.toString());
         }
      }

      // Save fields in the projection shape class
      for (Field field : projectionShape.getClass().getDeclaredFields())
      {
         if (field.getType() == ImDouble.class || field.getType() == ImInt.class || field.getType() == ImBoolean.class)
         {
            field.setAccessible(true);
            Object fieldValue = field.get(projectionShape);
            properties.put(field.getName(), fieldValue.toString());
         }
      }

      try (FileOutputStream output = new FileOutputStream(vrStereoVisionSettingsFile))
      {
         properties.store(output, null);
      }
   }

   public void saveAsync(RDXProjectionShape projectionShape, String presetFileName)
   {
      CompletableFuture.runAsync(() ->
      {
         try
         {
            save(projectionShape, presetFileName);
         }
         catch (IOException | NoSuchMethodException | InvocationTargetException | IllegalAccessException e)
         {
            LogTools.info(e);
         }
      });
   }

   public void load(RDXProjectionShape projectionShape, String presetFileName)
         throws IOException, InvocationTargetException, NoSuchMethodException, IllegalAccessException
   {
      File file = IHMCCommonPaths.VR_DIRECTORY.resolve(presetFileName).toFile();

      if (!file.exists())
      {
         save(projectionShape, presetFileName);
      }

      Properties properties = new Properties();

      try (FileInputStream input = new FileInputStream(file))
      {
         properties.load(input);
      }

      try
      {
         // Load fields in this class
         for (Field field : getClass().getDeclaredFields())
         {
            Method setMethod = null;
            Object setParam = null;

            if (field.getType() == ImDouble.class)
            {
               setMethod = field.getType().getDeclaredMethod("set", double.class);
               setParam = Double.parseDouble(properties.getProperty(field.getName()));
            }
            else if (field.getType() == ImInt.class)
            {
               setMethod = field.getType().getDeclaredMethod("set", int.class);
               setParam = Integer.parseInt(properties.getProperty(field.getName()));
            }
            else if (field.getType() == ImBoolean.class)
            {
               setMethod = field.getType().getDeclaredMethod("set", boolean.class);
               setParam = Boolean.parseBoolean(properties.getProperty(field.getName()));
            }

            if (setMethod != null)
            {
               try
               {
                  setMethod.invoke(field.get(this), setParam);
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
            }
         }

         // Load fields in projection shape class
         for (Field field : projectionShape.getClass().getDeclaredFields())
         {
            Method setMethod = null;
            Object setParam = null;

            if (field.getType() == ImDouble.class)
            {
               setMethod = field.getType().getDeclaredMethod("set", double.class);
               setParam = Double.parseDouble(properties.getProperty(field.getName()));
            }
            else if (field.getType() == ImInt.class)
            {
               setMethod = field.getType().getDeclaredMethod("set", int.class);
               setParam = Integer.parseInt(properties.getProperty(field.getName()));
            }
            else if (field.getType() == ImBoolean.class)
            {
               setMethod = field.getType().getDeclaredMethod("set", boolean.class);
               setParam = Boolean.parseBoolean(properties.getProperty(field.getName()));
            }

            if (setMethod != null)
            {
               try
               {
                  field.setAccessible(true);
                  setMethod.setAccessible(true);
                  setMethod.invoke(field.get(projectionShape), setParam);
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
            }
         }
      }
      catch (Exception e)
      {
         LogTools.error(e);
         // Delete and re-create the file if there was a parsing error
         if (deleteRetries++ > 20)
            return; // Stop gap
         file.delete();
         save(projectionShape, presetFileName);
      }
   }

   public boolean renderControls()
   {
      boolean updated = false;

      ImGui.checkbox(labels.get("Lock projection shapes on robot"), projectionShapesLockOnRobot);
      ImGui.checkbox(labels.get("Show left projection"), showLeftProjectionShape);
      ImGui.checkbox(labels.get("Show right projection"), showRightProjectionShape);
      updated |= ImGuiTools.sliderDouble("Projection X offset", projectionXOffsetCalibration, -4, 4);
      updated |= ImGuiTools.sliderDouble("Projection Y offset", projectionYOffsetCalibration, -0.2, 0.2);
      updated |= ImGuiTools.sliderDouble("Projection Z offset", projectionZOffsetCalibration, -4, 4);
      updated |= ImGuiTools.sliderDouble("Projection yaw offset", projectionYawOffsetCalibration, -Math.toRadians(15.0), Math.toRadians(15.0));
      updated |= ImGuiTools.sliderDouble("Projection pitch offset", projectionPitchOffsetCalibration, -Math.toRadians(15.0), Math.toRadians(15.0));
      updated |= ImGuiTools.sliderDouble("Projection roll offset", projectionRollOffsetCalibration, -Math.toRadians(15.0), Math.toRadians(15.0));
      updated |= ImGuiTools.sliderDouble("Camera X offset", cameraXOffsetCalibration, -0.05, 0.05);
      updated |= ImGuiTools.sliderDouble("Camera Y offset", cameraYOffsetCalibration, 0.00, 0.12);
      updated |= ImGuiTools.sliderDouble("Camera Z offset", cameraZOffsetCalibration, -0.05, 0.05);
      updated |= ImGuiTools.sliderDouble("Camera Yaw offset", cameraYawOffsetCalibration, -0.05, 0.05);
      updated |= ImGuiTools.sliderDouble("Camera Pitch offset", cameraPitchOffsetCalibration, -0.05, 0.05);
      updated |= ImGuiTools.sliderDouble("Camera Roll offset", cameraRollOffsetCalibration, -0.05, 0.05);

      if (updated)
         unsaved = true;

      if (unsaved)
      {
         // TODO: visually show it's unsaved
      }

      return updated;
   }

   public boolean renderIOControls(RDXProjectionShape projectionShape)
   {
      ImGuiTools.inputText(labels.get("Preset name"), presetFileName);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Save")))
      {
         unsaved = false;
         saveAsync(projectionShape, presetFileName.get());
      }

      String[] presetFileNames = new String[availablePresets.size()];
      for (int i = 0; i < availablePresets.size(); i++)
         presetFileNames[i] = availablePresets.get(i);
      ImGui.combo(labels.get("Presets"), selectedPreset, presetFileNames);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Load")))
      {
         String presetFileNameToLoad = availablePresets.get(selectedPreset.get());
         presetFileName.set(presetFileNameToLoad);
         RDXBaseUI.pushNotification("Loading projection settings preset: " + presetFileNameToLoad);
         try
         {
            load(projectionShape, presetFileNameToLoad);
            return true;
         }
         catch (IOException | InvocationTargetException | NoSuchMethodException | IllegalAccessException e)
         {
            RDXBaseUI.pushNotification("Could not load projection settings preset: " + presetFileNameToLoad);
            LogTools.error(e);
         }
      }

      return false;
   }
}
