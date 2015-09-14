package us.ihmc.tools;

import static org.junit.Assert.assertTrue;

import org.apache.commons.lang3.StringUtils;
import org.junit.Test;

import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class FormattingToolsTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
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

	@DeployableTestMethod(estimatedDuration = 0.0)
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

	@DeployableTestMethod(estimatedDuration = 0.0)
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
}
