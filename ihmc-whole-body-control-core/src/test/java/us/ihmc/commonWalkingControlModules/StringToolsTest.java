package us.ihmc.commonWalkingControlModules;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class StringToolsTest
{
   @Test
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
