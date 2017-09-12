package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.filechooser.FileFilter;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/14/13
 * Time: 12:27 PM
 * To change this template use File | Settings | File Templates.
 */
public class ScriptFileSelector
{
   public static File getScriptFileFromUserSelection(String fileExtension)
   {
      JFileChooser chooser = getFileChooser(fileExtension);

      if (chooser != null)
      {
         File fileName = chooser.getSelectedFile();
         return fileName;
      }

      return null;
   }

   private static JFileChooser getFileChooser(String fileExtension)
   {
      JFileChooser chooser = new JFileChooser(new File(ScriptEngineSettings.scriptLoadingDirectory));
      chooser.setFileFilter(getIsoDataPathFilter(fileExtension));

      int returnVal = chooser.showOpenDialog(new JFrame());
      if (returnVal != JFileChooser.APPROVE_OPTION)
      {
         System.err.println("Can not load selected file: " + chooser.getName());

         return null;
      }

      return chooser;
   }

   private static FileFilter getIsoDataPathFilter(String fileExtension)
   {
      return new ProperEndingFileOrDirectoryFilter(fileExtension, true);
   }

   private static class ProperEndingFileOrDirectoryFilter extends FileFilter
   {
      private final String ending;
      private final boolean includeDirectories;

      public ProperEndingFileOrDirectoryFilter(String ending, boolean includeDirectories)
      {
         this.ending = ending;
         this.includeDirectories = includeDirectories;
      }

      public boolean accept(File file)
      {
         if (file.getAbsolutePath().endsWith(ending) || (includeDirectories && file.isDirectory()))
            return true;

         return false;
      }

      public String getDescription()
      {
         return "*" + ending;
      }
   }

   public static void main(String[] args)
   {
      System.out.println(ScriptFileSelector.getScriptFileFromUserSelection(ScriptEngineSettings.extension));

      System.exit(0);
   }

}
