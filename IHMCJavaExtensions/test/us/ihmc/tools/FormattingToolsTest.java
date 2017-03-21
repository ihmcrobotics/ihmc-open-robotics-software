package us.ihmc.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.Calendar;
import java.util.GregorianCalendar;
import java.util.TimeZone;

import org.apache.commons.lang3.StringUtils;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.MathTools;

public class FormattingToolsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetFormattedDecimal3D()
   {
      String resultingFormattedString = FormattingTools.getFormattedDecimal3D(1.2345678);
      assertTrue(resultingFormattedString.equals("1.235"));

      resultingFormattedString = FormattingTools.getFormattedDecimal3D(0.1234);
      assertTrue(resultingFormattedString.equals("0.123"));

      resultingFormattedString = FormattingTools.getFormattedDecimal3D(-0.1234);
      assertTrue(resultingFormattedString.equals("-0.123"));

      resultingFormattedString = FormattingTools.getFormattedDecimal3D(0.0234);
      assertTrue(resultingFormattedString.equals("0.023"));

      resultingFormattedString = FormattingTools.getFormattedDecimal3D(-0.0234);
      assertTrue(resultingFormattedString.equals("-0.023"));

      resultingFormattedString = FormattingTools.getFormattedDecimal3D(22.0234);
      assertTrue(resultingFormattedString.equals("22.023"));

      resultingFormattedString = FormattingTools.getFormattedDecimal3D(-22.0234);
      assertTrue(resultingFormattedString.equals("-22.023"));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCapitalizeFirstLetter()
   {
      String resultingString = StringUtils.capitalize("capital");
      assertTrue(resultingString.equals("Capital"));
      resultingString = StringUtils.uncapitalize(resultingString);
      assertTrue(resultingString.equals("capital"));

      resultingString = StringUtils.capitalize("robot");
      assertTrue(resultingString.equals("Robot"));
      resultingString = StringUtils.uncapitalize(resultingString);
      assertTrue(resultingString.equals("robot"));

      resultingString = StringUtils.capitalize("Robot");
      assertTrue(resultingString.equals("Robot"));
      resultingString = StringUtils.uncapitalize(resultingString);
      assertTrue(resultingString.equals("robot"));
      resultingString = StringUtils.uncapitalize(resultingString);
      assertTrue(resultingString.equals("robot"));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testUnderScoredToCamelCase()
   {
      String resultingFormattedString;
      resultingFormattedString = FormattingTools.underscoredToCamelCase("TEST_ABCD_DEFG", true);
      assertTrue(resultingFormattedString.equals("TestAbcdDefg"));

      resultingFormattedString = FormattingTools.underscoredToCamelCase("TEST_ABCD_DEFG", false);
      assertTrue(resultingFormattedString.equals("testAbcdDefg"));

      resultingFormattedString = FormattingTools.underscoredToCamelCase("1234_@$%_BCDF", true);
      assertTrue(resultingFormattedString.equals("1234@$%Bcdf"));
   }
	
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testFormatToSignificantFigures()
   {
      assertEquals("not equal", 100, MathTools.roundToSignificantFigures(123.45, 1), 1e-12);
      assertEquals("not equal", 120, MathTools.roundToSignificantFigures(123.45, 2), 1e-12);
      
      ByteArrayOutputStream byteArrayOutputStream;
      PrintStream systemOut;
      
      byteArrayOutputStream = new ByteArrayOutputStream();
      systemOut = System.out;
      System.setOut(new PrintStream(byteArrayOutputStream));
      System.out.println(FormattingTools.getFormattedToPrecision(0.00000000000001, 0.01, 2));
      System.out.flush();
      System.setOut(systemOut);
      System.out.println("ByteArrayOutputStream.toString(): " + byteArrayOutputStream.toString());
      assertEquals("FormattingTools.getFormattedToSignificantFigures didn't work.", "0\r\n", byteArrayOutputStream.toString());
      
      byteArrayOutputStream = new ByteArrayOutputStream();
      systemOut = System.out;
      System.setOut(new PrintStream(byteArrayOutputStream));
      System.out.println(FormattingTools.getFormattedToPrecision(0.01000000000001, 0.01, 2));
      System.out.flush();
      System.setOut(systemOut);
      System.out.println("ByteArrayOutputStream.toString(): " + byteArrayOutputStream.toString());
      assertEquals("FormattingTools.getFormattedToSignificantFigures didn't work.", "0.01\r\n", byteArrayOutputStream.toString());
   }
	
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetDateString()
   {
      String dateToolsDateString = FormattingTools.getDateString();
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
      String timeString = FormattingTools.getTimeString();

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
}
