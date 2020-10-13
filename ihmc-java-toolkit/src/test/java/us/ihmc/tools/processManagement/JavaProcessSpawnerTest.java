package us.ihmc.tools.processManagement;

import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;

import static org.junit.jupiter.api.Assertions.*;

public class JavaProcessSpawnerTest
{
   @Test
   public void testJavaProcessSpawnerRunsWithoutErrors()
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true);
      Process process = spawner.spawn(JavaProcessSpawnerTest.class);

      ThreadTools.sleepSeconds(2.0);

      assertDoesNotThrow(() -> spawner.kill(process));
   }

   @Test
   public void testJavaProcessSpawnerPrintsToStream()
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true);

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      String utf8 = StandardCharsets.UTF_8.name();

      Process process = spawner.spawn(JavaProcessSpawnerTest.class, null, null, new PrintStream(outputStream), null);

      ThreadTools.sleepSeconds(2.0);

      MutableObject<String> data = new MutableObject<>();
      assertDoesNotThrow(() -> data.setValue(outputStream.toString(utf8)));

      LogTools.info("Output: " + data.getValue());
      assertTrue(data.getValue().length() > 0);

      assertDoesNotThrow(() -> spawner.kill(process));
   }

   public static void main(String[] args)
   {
      MutableInt mutableInt = new MutableInt();
      new PausablePeriodicThread(JavaProcessSpawnerTest.class.getSimpleName(), 0.25, () ->
      {
         LogTools.info(mutableInt.getAndIncrement());
      }).start();
   }
}
