package us.ihmc.tools;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class FormattingToolsTest
{

	@DeployableTestMethod(duration = 0.0)
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

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testCapitalizeFirstLetter()
   {
      String resultingString = FormattingTools.capitalizeFirstLetter("capital");
      assertTrue(resultingString.equals("Capital"));
      resultingString = FormattingTools.lowerCaseFirstLetter(resultingString);
      assertTrue(resultingString.equals("capital"));

      resultingString = FormattingTools.capitalizeFirstLetter("robot");
      assertTrue(resultingString.equals("Robot"));
      resultingString = FormattingTools.lowerCaseFirstLetter(resultingString);
      assertTrue(resultingString.equals("robot"));

      resultingString = FormattingTools.capitalizeFirstLetter("Robot");
      assertTrue(resultingString.equals("Robot"));
      resultingString = FormattingTools.lowerCaseFirstLetter(resultingString);
      assertTrue(resultingString.equals("robot"));
      resultingString = FormattingTools.lowerCaseFirstLetter(resultingString);
      assertTrue(resultingString.equals("robot"));

   }

	@DeployableTestMethod(duration = 0.0)
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
