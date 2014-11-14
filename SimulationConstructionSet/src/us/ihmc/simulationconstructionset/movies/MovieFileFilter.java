package us.ihmc.simulationconstructionset.movies;

import java.io.File;
import java.io.FilenameFilter;

public class MovieFileFilter extends javax.swing.filechooser.FileFilter implements FilenameFilter
{
   private final String[] extensions = { "mov", "mp4", "mpeg", "avi", "flv" };
   
   
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

   public String getDescription()
   {
      return "Movie: .mov, .mp4, mpeg, .avi, .flv";
   }

   public boolean accept(File dir, String name)
   {
      return accept(new File(dir, name));
   }
}