package us.ihmc.robotDataLogger.logger.converters;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.PathMatcher;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Properties;

import javax.swing.JFileChooser;
import javax.swing.JOptionPane;

import us.ihmc.robotDataLogger.logger.LogProperties;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;

public class ModelAttacher extends SimpleFileVisitor<Path>
{
   public enum LogModels
   {
      ATLAS("RobotModels/Atlas"),
      STEPPR("RobotModels/Steppr"),
      VALKYRIE("RobotModels/Valkyrie");

      private final boolean valid;
      private final String loader;
      private final String modelName;
      private final String[] resourceDirectories;
      private final File model;
      private final File resources;

      LogModels(String modelDirectory)
      {
         File dir = new File(modelDirectory);
         Properties logModelProperties;
         try
         {
            File log = new File(dir, "description.properties");
            FileReader reader = new FileReader(log);
            logModelProperties = new Properties();
            logModelProperties.load(reader);
            reader.close();

         }
         catch (IOException e)
         {
            System.err.println(e.getMessage());
            model = null;
            resources = null;
            loader = null;
            modelName = null;
            resourceDirectories = null;
            valid = false;
            return;
         }

         model = new File(dir, modelFilename);
         resources = new File(dir, modelResourceBundle);
         loader = logModelProperties.getProperty("loader");
         modelName = logModelProperties.getProperty("modelName");
         resourceDirectories = logModelProperties.getProperty("resourceDirectories").split(",");
         valid = true;
      }

      public boolean isValid()
      {
         return valid;
      }

      public String getLoader()
      {
         return loader;
      }

      public String getModelName()
      {
         return modelName;
      }

      public String[] getResourceDirectories()
      {
         return resourceDirectories;
      }

      public File getModel()
      {
         return model;
      }

      public File getResources()
      {
         return resources;
      }

   }

   private static final String modelFilename = "model.sdf";
   private static final String modelResourceBundle = "resources.zip";

   private final PathMatcher matcher = FileSystems.getDefault().getPathMatcher("glob:robotData.log");

   private final LogModels model;

   public ModelAttacher() throws IOException
   {

      JFileChooser chooser = new JFileChooser();
      chooser.setDialogTitle("Select directory");
      chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      if (chooser.showOpenDialog(null) == JFileChooser.APPROVE_OPTION)
      {
         File mainDirectory = chooser.getSelectedFile();

         model = chooseModel(mainDirectory);
         if (model != null)
         {
            System.out.println("Attaching model " + model);
            Files.walkFileTree(mainDirectory.toPath(), this);
         }
      }
      else
      {
         model = null;
      }

   }

   @Override
   public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
   {
      if (matcher.matches(file.getFileName()))
      {
         try
         {
            File directory = file.getParent().toFile();
            File log = new File(directory, YoVariableLoggerListener.propertyFile);
            LogProperties properties = new LogPropertiesReader(log);
            addModel(directory, properties, model);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
      return FileVisitResult.CONTINUE;
   }

   @Override
   public FileVisitResult visitFileFailed(Path file, IOException exc) throws IOException
   {

      System.err.println(exc.getMessage());
      return FileVisitResult.CONTINUE;
   }

   public static LogModels chooseModel(File mainDirectory)
   {
      return (LogModels) JOptionPane.showInputDialog(null, "Please choose a model to attach to " + mainDirectory, "Model chooser",
            JOptionPane.QUESTION_MESSAGE, null, LogModels.values(), null);
   }

   public static void addModel(File modelDirectory, LogProperties properties, LogModels model) throws IOException
   {



      if (properties.getModelLoaderClass() == null)
      {
         if (model == null || !model.isValid())
         {
            throw new RuntimeException("Cannot load model files");
         }

         System.out.println("Adding model to " + modelDirectory);
         properties.setModelLoaderClass(model.getLoader());
         properties.setModelName(model.getModelName());
         properties.setModelResourceDirectories(model.getResourceDirectories());
         properties.setModelPath(modelFilename);
         properties.setModelResourceBundlePath(modelResourceBundle);

         File modelFile = new File(modelDirectory, modelFilename);
         Files.copy(model.getModel().toPath(), modelFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
         File resourceFile = new File(modelDirectory, modelResourceBundle);
         Files.copy(model.getResources().toPath(), resourceFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
         
         File log = new File(modelDirectory, YoVariableLoggerListener.propertyFile);
         FileWriter writer = new FileWriter(log);
         properties.store(writer, "Model added by ModelAttacher");
         writer.close();
         System.out.println("Attached model to " + modelDirectory);
      }
      else
      {
         System.out.println(modelDirectory + " already contains a robot model");
      }

   }

   public static void main(String[] args) throws IOException
   {
      new ModelAttacher();
   }
}
