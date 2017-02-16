package us.ihmc.tools.time;

import static org.junit.Assert.assertEquals;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.TimeZone;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class DateToolsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetDateString()
   {
      String dateToolsDateString = DateTools.getDateString();
      StringBuilder dateBuilder = new StringBuilder();

      Calendar calendar = new GregorianCalendar(TimeZone.getDefault());

      int year = calendar.get(GregorianCalendar.YEAR);
      int month = calendar.get(GregorianCalendar.MONTH) + 1;
      int day = calendar.get(GregorianCalendar.DAY_OF_MONTH);

      dateBuilder.append(year);
      if (month / 10 < 1)
         dateBuilder.append("0" + month);
      else
         dateBuilder.append(month);

      if (day / 10 < 1)
         dateBuilder.append("0" + day);
      else
         dateBuilder.append(day);

      assertEquals(dateBuilder.toString(), dateToolsDateString);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTimeString()
   {
      StringBuilder timeBuilder = new StringBuilder();
      Calendar calendar = new GregorianCalendar(TimeZone.getDefault());

      int hours = calendar.get(GregorianCalendar.HOUR_OF_DAY);
      int minutes = calendar.get(GregorianCalendar.MINUTE);
      String timeString = DateTools.getTimeString();

      if (hours / 10 < 1)
         timeBuilder.append("0" + hours);
      else
         timeBuilder.append(hours);

      if (minutes / 10 < 1)
         timeBuilder.append("0" + minutes);
      else
         timeBuilder.append(minutes);

      assertEquals(timeBuilder.toString(), timeString);

   }

	@SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetCurrentDate()
   {
      Date testDate = DateTools.getCurrentDate();
      Date todaysDate = DateTools.getCurrentDate();

      assertEquals(testDate.getDate(), todaysDate.getDate());
      assertEquals(testDate.getMonth(), todaysDate.getMonth());
      assertEquals(testDate.getYear(), todaysDate.getYear());
      assertEquals(testDate.getHours(), todaysDate.getHours());
   }

}
