package us.ihmc.tools.gui;

import java.io.File;

public class MyFileFilter extends javax.swing.filechooser.FileFilter
{
   private String[] extensions;
   private String description;

   public MyFileFilter(String extension, String description)
   {
      this(new String[] {extension}, description);
   }

   public MyFileFilter(String[] extensions, String description)
   {
      super();

      this.extensions = extensions;
      this.description = description;
   }

   @Override
   public boolean accept(File f)
   {
      if (f.isDirectory())
         return true;

      for (int i = 0; i < extensions.length; i++)
      {
         if (f.getName().endsWith(extensions[i]))
            return true;
      }

      return false;
   }

   @Override
   public String getDescription()
   {
      return description;
   }
}
