package us.ihmc.avatar.ros.networkTest;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.time.temporal.ChronoUnit;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class ROS2NetworkTest
{
   private static final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ISO_LOCAL_DATE_TIME;

   private final HashMap<String, ROS2NetworkTestProfile> profiles = new HashMap<>();
   {
      addProfile(IntegersAt1HzNetworkTestProfile.class);
   }

   public ROS2NetworkTest(List<String> args)
   {
      String profileName = args.get(args.indexOf("--profile") + 1);
      ROS2NetworkTestProfile profile = profiles.get(profileName);
      LogTools.info("Running profile: {}", profileName);

      LocalDateTime now = LocalDateTime.now();
      LocalDateTime startTime;

      if (args.contains("--host"))
      {
         LogTools.info("Running as host. Starting other nodes.");

         // decide time to start
         startTime = now.plusSeconds(5);
         String formattedStartTime = startTime.format(dateTimeFormatter);

         // use SSHJ to remote execute this class with arguments
         // TODO


         // wait until they are all started
         // start YoVariableClients
         // TODO
         for (String remoteHostname : profile.getRemoteHostnames())
         {
            // connect to server at hostname
         }
      }
      else
      {
         if (!args.contains("--startTime"))
         {
            throw new RuntimeException("Client mode requires a time to start specified with --startTime");
         }

         String startTimeString = args.get(args.indexOf("--startTime") + 1);
         startTime = LocalDateTime.parse(startTimeString, dateTimeFormatter);

         // start YoVariableServer
      }

      if (!args.contains("--profile"))
      {
         throw new RuntimeException("Must select a profile with --profile");
      }

      // wait until decided time
      ThreadTools.sleep(LocalDateTime.now().until(startTime, ChronoUnit.MILLIS));


      profile.runExperiment();
   }

   private void addProfile(Class<? extends ROS2NetworkTestProfile> clazz)
   {
      profiles.put(clazz.getSimpleName(), ExceptionTools.handle(clazz::newInstance, DefaultExceptionHandler.RUNTIME_EXCEPTION));
   }

   public static void main(String[] args)
   {
      new ROS2NetworkTest(Arrays.asList(args));
   }
}
