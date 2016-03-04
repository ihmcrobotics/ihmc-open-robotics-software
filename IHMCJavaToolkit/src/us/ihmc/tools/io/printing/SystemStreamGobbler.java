package us.ihmc.tools.io.printing;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;

public class SystemStreamGobbler
{
   private final GobbleType[] gobbleTypes;
   private PrintStream originalSysout;
   private PrintStream originalSyserr;
   
   public enum GobbleType
   {
      SYSTEM_ERROR,
      SYSTEM_OUT
   }
   
   public static void gobbleOutput(Runnable runnable, GobbleType... gobbleTypes)
   {
      SystemStreamGobbler gobbler = new SystemStreamGobbler(gobbleTypes);
      
      runnable.run();
      
      gobbler.stopGobbling();
   }
   
   public SystemStreamGobbler(GobbleType... gobbleTypes)
   {
      this.gobbleTypes = gobbleTypes;
      
      startGobbling();
   }
   
   private void startGobbling()
   {
      PrintStream gobbleStream = new PrintStream(new OutputStream()
      {
         @Override
         public void write(int b) throws IOException
         {
            // gobble gobble
         }
      });
      
      for (GobbleType gobbleType : gobbleTypes)
      {
         if (gobbleType == GobbleType.SYSTEM_OUT)
         {
            originalSysout = System.out;
            System.setOut(gobbleStream);
         }
         else if (gobbleType == GobbleType.SYSTEM_ERROR)
         {
            originalSyserr = System.err;
            System.setErr(gobbleStream);
         }
      }
   }
   
   public void stopGobbling()
   {
      for (GobbleType gobbleType : gobbleTypes)
      {
         if (gobbleType == GobbleType.SYSTEM_OUT)
         {
            System.setOut(originalSysout);
         }
         else if (gobbleType == GobbleType.SYSTEM_ERROR)
         {
            System.setErr(originalSyserr);
         }
      }
   }
}
