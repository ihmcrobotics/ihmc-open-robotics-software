package us.ihmc.avatar.ros.networkTest;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class ROS2NetworkTest
{
   private final HashMap<String, Runnable> profiles = new HashMap<>();
   {
      addProfile(IntegersAt1HzNetworkTestProfile.class);
   }

   public ROS2NetworkTest(List<String> args)
   {

      if (args.contains("--host"))
      {
         LogTools.info("Running as host. Starting other nodes.");

         // TODO
         // decide time to start

         // TODO
         // use SSHJ to remote execute this class with arguments
         // TODO
         // wait until they are all started

      }
      else
      {
         if (!args.contains("--startTime"))
         {
            throw new RuntimeException("Client mode requires a time to start specified with --startTime");
         }

         // TODO
         // wait until start time
      }

      LocalDateTime now = LocalDateTime.now();

      if (!args.contains("--profile"))
      {
         throw new RuntimeException("Must select a profile with --profile");
      }

      // TODO
      // wait until decided time


      String profileName = args.get(args.indexOf("--profile") + 1);
      Runnable profile = profiles.get(profileName);
      LogTools.info("Running profile: {}", profileName);
      profile.run();
   }

   private void addProfile(Class<?> clazz)
   {
      profiles.put(clazz.getSimpleName(), () -> ExceptionTools.handle(clazz::newInstance, DefaultExceptionHandler.RUNTIME_EXCEPTION));
   }

   public static void main(String[] args)
   {
      new ROS2NetworkTest(Arrays.asList(args));
   }
}
