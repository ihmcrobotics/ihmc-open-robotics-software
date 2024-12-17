package us.ihmc.externalControl.test;

import us.ihmc.externalControl.global.ExternalControlWrapper;
import us.ihmc.externalControl.library.ExternalControlNativeLibrary;

public class ExternalControlTest
{
   public static void main(String[] args)
   {
      ExternalControlNativeLibrary.load();
      ExternalControlWrapper.test();
      ExternalControlWrapper.test2(5.0);
   }
}
