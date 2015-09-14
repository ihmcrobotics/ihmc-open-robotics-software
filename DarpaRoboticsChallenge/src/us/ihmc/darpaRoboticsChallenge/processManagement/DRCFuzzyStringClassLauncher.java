package us.ihmc.darpaRoboticsChallenge.processManagement;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.tools.processManagement.FuzzyStringClassLauncher;

public class DRCFuzzyStringClassLauncher extends FuzzyStringClassLauncher
{
   public DRCFuzzyStringClassLauncher(String name)
   {
      super(name);
   }

   public static void main(String[] args)
   {
      if(SystemUtils.IS_OS_MAC)
         System.setProperty("apple.awt.UIElement", "true");
      DRCFuzzyStringClassLauncher launcher = new DRCFuzzyStringClassLauncher("Class Launcher");      
      launcher.start();
   }
}
