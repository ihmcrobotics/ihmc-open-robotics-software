package us.ihmc.avatar.ros.networkTest;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.time.temporal.ChronoUnit;

import static org.junit.jupiter.api.Assertions.*;

public class ROS2NetworkTestTest
{
   @Test
   public void testTimeStuffWorks()
   {
      LocalDateTime now = LocalDateTime.now();
      String format1 = now.format(DateTimeFormatter.ISO_LOCAL_DATE_TIME);
      LogTools.info("Now: {}", format1);
      LocalDateTime nowOther = LocalDateTime.parse(format1, DateTimeFormatter.ISO_LOCAL_DATE_TIME);
      LogTools.info("Parsed: {}", nowOther);
      assertEquals(format1, nowOther.format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));

      LocalDateTime inFive = nowOther.plusSeconds(5);
      long millis = nowOther.until(inFive, ChronoUnit.MILLIS);
      LogTools.info("Millis: {}", millis);
      assertEquals(5000, millis);
   }

   @Test
   public void testGetHostname()
   {
      InetAddress localHost = null;
      try
      {
         localHost = InetAddress.getLocalHost();
         LogTools.info("Hostname: {}", localHost.getHostName());
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
   }
}
