package us.ihmc.tools.processManagement;

import java.io.IOException;
import java.lang.reflect.Field;

public class UnixProcessKiller
{
   public static boolean isUnixProcess(Process process)
   {
      if (process.getClass().getName().equals("java.lang.UNIXProcess"))
      {
         return true;
      }

      return false;
   }

   public static int getUnixPID(Process process)
   {
      try
      {
         if (process.getClass().getName().equals("java.lang.UNIXProcess"))
         {
            Class<?> cl = process.getClass();
            Field field = cl.getDeclaredField("pid");
            field.setAccessible(true);
            Object pidObject = field.get(process);
            return (Integer) pidObject;
         }
         else
         {
            throw new IllegalArgumentException("Needs to be a UNIXProcess");
         }
      }
      catch (NoSuchFieldException e)
      {
         throw new RuntimeException(e);
      }
      catch (IllegalArgumentException e)
      {
         throw new RuntimeException(e);
      }
      catch (IllegalAccessException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static int killSigIntUnixProcess(Process process)
   {
      int pid = getUnixPID(process);
      try
      {
         ProcessBuilder builder = new ProcessBuilder();
         builder.command(new String[]{"kill", "-2", "" + pid});
         return builder.start().waitFor();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static int killSigKillUnixProcess(Process process)
   {
      int pid = getUnixPID(process);
      try
      {
         ProcessBuilder builder = new ProcessBuilder();
         builder.command(new String[]{"kill", "-9", "" + pid});
         return builder.start().waitFor();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static int killSigTermUnixProcess(Process process)
   {
      int pid = getUnixPID(process);
      try
      {
         ProcessBuilder builder = new ProcessBuilder();
         builder.command(new String[]{"kill", "" + pid});
         return builder.start().waitFor();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
