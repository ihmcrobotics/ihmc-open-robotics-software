package us.ihmc.simulationconstructionset.videos;

import java.io.File;
import java.io.FilenameFilter;

public class VideoFileFilter extends javax.swing.filechooser.FileFilter implements FilenameFilter
{
   private final String[] extensions = { "mov", "mp4", "mpeg", "avi", "flv" };
   
   
   @Override
   public boolean accept(File file)
   {
      if(file.isDirectory())
      {
         return true;
      }
      
      String name = file.getName();
      String ext = name.substring(name.lastIndexOf(".") + 1);
      for(String extToTest : extensions)
      {
         if(extToTest.equals(ext))
            return true;
      }
      
      return false;
   }

   @Override
   public String getDescription()
   {
      return "Video: .mov, .mp4, mpeg, .avi, .flv";
   }

   @Override
   public boolean accept(File dir, String name)
   {
      return accept(new File(dir, name));
   }
}