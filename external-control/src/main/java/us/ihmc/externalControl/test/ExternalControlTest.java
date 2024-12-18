package us.ihmc.externalControl.test;

import us.ihmc.externalControl.global.ExternalControlWrapper;
import us.ihmc.externalControl.global.ExternalControlWrapper.TestImpl;
import us.ihmc.externalControl.library.ExternalControlNativeLibrary;

public class ExternalControlTest
{
   public static void main(String[] args)
   {
      ExternalControlNativeLibrary.load();

      TestImpl test = new TestImpl();
      test.test();
      test.test2(5.0);

      ExternalControlWrapper.ExternalControlImpl externalControl = new ExternalControlWrapper.ExternalControlImpl(5.0, 5.0, 12);
   }
}
