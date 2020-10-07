package us.ihmc.tools.saveableModule;

import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;

import javax.swing.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;

public class SaveableModuleStateTools
{
   private static final Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
   private static final String testFormatPostfix = "_Log";

   public static void registerYoTuple3DToSave(YoTuple3D framePoint3D, SaveableModuleState state)
   {
      state.registerDoubleToSave(framePoint3D.getYoX());
      state.registerDoubleToSave(framePoint3D.getYoY());
      state.registerDoubleToSave(framePoint3D.getYoZ());
   }

   public static void registerYoTuple2DToSave(YoTuple2D framePoint2D, SaveableModuleState state)
   {
      state.registerDoubleToSave(framePoint2D.getYoX());
      state.registerDoubleToSave(framePoint2D.getYoY());
   }

   public static void registerYoFrameQuaternionToSave(YoFrameQuaternion frameQuaternion, SaveableModuleState state)
   {
      state.registerDoubleToSave(frameQuaternion.getYoQs());
      state.registerDoubleToSave(frameQuaternion.getYoQx());
      state.registerDoubleToSave(frameQuaternion.getYoQy());
      state.registerDoubleToSave(frameQuaternion.getYoQz());
   }

   public static void registerYoFramePose3DToSave(YoFramePose3D framePose3D, SaveableModuleState state)
   {
      registerYoTuple3DToSave(framePose3D.getPosition(), state);
      registerYoFrameQuaternionToSave(framePose3D.getOrientation(), state);
   }

   public static void save(String moduleName, SaveableModuleState state)
   {
      JFileChooser fileChooser = new JFileChooser();
      Path directory = rootPath;
      File logDirectory = new File(directory.toString());
      SaveableModuleTools.ensureFileExists(logDirectory);
      File fileToSave = new File(moduleName + dateFormat.format(new Date()) + testFormatPostfix + File.separator);

      fileChooser.setCurrentDirectory(logDirectory);
      fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
      fileChooser.setSelectedFile(fileToSave);
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState != JFileChooser.APPROVE_OPTION)
         return;

      File file = fileChooser.getSelectedFile();

      save(file, state);
   }

   public static void save(File fileToSaveTo, SaveableModuleState stateToSave)
   {
      if (fileToSaveTo == null)
         throw new IllegalArgumentException("File has not been set.");
      if (stateToSave == null)
         throw new IllegalArgumentException("State has not been set.");

      SaveableModuleTools.ensureFileExists(fileToSaveTo);

      try
      {
         FileWriter fileWriter = new FileWriter(fileToSaveTo);
         fileWriter.write(stateToSave.toString());
         fileWriter.close();
      }
      catch (IOException ex)
      {
         throw new RuntimeException("Problem when saving module.");
      }
   }
}
