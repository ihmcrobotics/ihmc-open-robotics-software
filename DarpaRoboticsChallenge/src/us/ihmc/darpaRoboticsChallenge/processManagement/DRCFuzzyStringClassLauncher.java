package us.ihmc.darpaRoboticsChallenge.processManagement;

import us.ihmc.utilities.processManagement.FuzzyStringClassLauncher;

public class DRCFuzzyStringClassLauncher extends FuzzyStringClassLauncher
{
   public DRCFuzzyStringClassLauncher(String name)
   {
      super(name);
   }

   public static void main(String[] args)
   {
      if(System.getProperty("os.name").toLowerCase().contains("mac"))
         System.setProperty("apple.awt.UIElement", "true");
      DRCFuzzyStringClassLauncher launcher = new DRCFuzzyStringClassLauncher("Class Launcher");      
      launcher.start();
   }
}
