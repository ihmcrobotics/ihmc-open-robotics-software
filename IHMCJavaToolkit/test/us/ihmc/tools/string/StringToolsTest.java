package us.ihmc.tools.string;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class StringToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testString()
   {
      String string = "YoLowLevelOneDoFJointDesiredDataHolder";
      String uppercaseLetters = StringTools.getEveryUppercaseLetter(string);
      assertTrue(uppercaseLetters.equals("YLLODFJDDH"));

      string = "YoLowLevelOneDoFJointDesiredDataHolderT";
      uppercaseLetters = StringTools.getEveryUppercaseLetter(string);
      assertTrue(uppercaseLetters.equals("YLLODFJDDHT"));

      string = "";
      uppercaseLetters = StringTools.getEveryUppercaseLetter(string);
      assertTrue(uppercaseLetters.equals(""));

      string = "DDFHHHDSRTRDFHNUYFRYJJ";
      uppercaseLetters = StringTools.getEveryUppercaseLetter(string);
      assertTrue(uppercaseLetters.equals("DDFHHHDSRTRDFHNUYFRYJJ"));


      string = "DDFHHHD7474S747R8585T88586RD685686F?HNU&)$#YFRYJJ";
      uppercaseLetters = StringTools.getEveryUppercaseLetter(string);
      assertTrue(uppercaseLetters.equals("DDFHHHDSRTRDFHNUYFRYJJ"));
   }
}