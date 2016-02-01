package us.ihmc.tools.time;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.GregorianCalendar;

public class DateTools
{
   private static final SimpleDateFormat dateStringFormatter = new SimpleDateFormat("yyyyMMdd");

   public static String getDateString()
   {
      Date date = new Date();

      return dateStringFormatter.format(date);
   }

   private static final SimpleDateFormat timeStringFormatter = new SimpleDateFormat("HHmm");

   public static String getTimeString()
   {
      Date date = new Date();

      return timeStringFormatter.format(date);
   }

   private static final SimpleDateFormat timeStringFormatterWithSeconds = new SimpleDateFormat("HHmmss");

   public static String getTimeStringWithSeconds()
   {
      Date date = new Date();

      return timeStringFormatterWithSeconds.format(date);
   }

   public static Date getCurrentDate()
   {
      return (new GregorianCalendar()).getTime();
   }

}
