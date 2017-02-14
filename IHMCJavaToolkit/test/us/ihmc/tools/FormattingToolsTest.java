package us.ihmc.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.apache.commons.lang3.StringUtils;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

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
      assertEquals("not equal", 100, FormattingTools.roundToSignificantFigures(123.45, 1), 1e-12);
      assertEquals("not equal", 120, FormattingTools.roundToSignificantFigures(123.45, 2), 1e-12);
   }
}
