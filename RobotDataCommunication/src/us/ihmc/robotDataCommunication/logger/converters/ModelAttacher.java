package us.ihmc.robotDataCommunication.logger.converters;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.PathMatcher;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;

import javax.swing.JFileChooser;

import org.apache.bcel.classfile.Field;

import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataCommunication.logger.LogProperties;
import us.ihmc.robotDataCommunication.logger.LogPropertiesReader;

public class ModelAttacher extends SimpleFileVisitor<Path>
{
   private static final String modelFilename = "model.sdf";
   private static final String modelResourceBundle = "resources.zip";

   private final PathMatcher matcher = FileSystems.getDefault().getPathMatcher("glob:robotData.log");
   private final LogModelProvider modelProvider;
   
   public ModelAttacher(LogModelProvider modelProvider) throws IOException
   {
      this.modelProvider = modelProvider;
      JFileChooser chooser = new JFileChooser();
      chooser.setDialogTitle("Select directory");
      chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      if (chooser.showOpenDialog(null) == JFileChooser.APPROVE_OPTION)
      {
         File mainDirectory = chooser.getSelectedFile();
         System.out.println("Attaching model " + modelProvider.getModelName());
         Files.walkFileTree(mainDirectory.toPath(), this);
      }
      

   }

   @Override
   public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
   {
      if (matcher.matches(file.getFileName()))
      {
         try
         {
            converter(file);
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

   private void converter(Path file) throws IOException
   {
      

      File log = file.toFile();
      File directory = file.getParent().toFile();
      LogProperties properties = new LogPropertiesReader(log);

      if (properties.getModelLoaderClass() == null)
      {
         System.out.println("Adding model to " + file);
         properties.setModelLoaderClass(modelProvider.getLoader().getCanonicalName());
         properties.setModelName(modelProvider.getModelName());
         properties.setModelResourceDirectories(modelProvider.getResourceDirectories());
         properties.setModelPath(modelFilename);
         properties.setModelResourceBundlePath(modelResourceBundle);
         
         File modelFile = new File(directory, modelFilename);
         FileOutputStream modelStream = new FileOutputStream(modelFile);
         modelStream.write(modelProvider.getModel());
         modelStream.close();
         File resourceFile = new File(directory, modelResourceBundle);
         FileOutputStream resourceStream = new FileOutputStream(resourceFile);
         resourceStream.write(modelProvider.getResourceZip());
         resourceStream.close();
         
         FileWriter writer = new FileWriter(log);
         properties.store(writer, "Model added by ModelAttacher");
         writer.close();

      }
      else
      {
         System.out.println(file + " already contains a robot model");
      }

   }


}
