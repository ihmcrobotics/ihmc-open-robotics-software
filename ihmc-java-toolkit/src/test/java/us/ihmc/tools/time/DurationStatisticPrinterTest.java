package us.ihmc.tools.time;

import org.junit.jupiter.api.Test;
import us.ihmc.tools.thread.MissingThreadTools;

public class DurationStatisticPrinterTest
{
   @Test
   public void test()
   {
      DurationStatisticPrinter printer = new DurationStatisticPrinter();

      for (int i = 0; i < 300; i++)
      {
         printer.before();
         MissingThreadTools.sleepMillis(10);
         printer.after();
      }

      printer.reset();
   }
}
