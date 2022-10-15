package us.ihmc.tools.string;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
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

   @Test
   public void testCasingMethods()
   {
      assertEquals("snake", StringTools.titleToSnakeCase("Snake"));
      assertEquals("snake_case", StringTools.titleToSnakeCase("Snake Case"));
      assertEquals("snake_case_blep", StringTools.titleToSnakeCase("snake case BLEP"));
      assertEquals("sfu_ai2_1_!!", StringTools.titleToSnakeCase("     sfu ai2 1    !!"));
      assertEquals("kebab", StringTools.titleToKebabCase("Kebab"));
      assertEquals("kebab-case", StringTools.titleToKebabCase("Kebab Case"));
      assertEquals("hellokebab-case", StringTools.titleToKebabCase("HELLOKebab Case"));
      assertEquals("hello-kebab-case", StringTools.titleToKebabCase(" HELLO Kebab Case"));
      assertEquals("MeepMan", StringTools.titleToPascalCase("meep man"));
      assertEquals("meepMan", StringTools.titleToCamelCase("meep man"));
      assertEquals("camelCase", StringTools.titleToCamelCase("Camel case"));
      assertEquals("gpuPlanarRegions", StringTools.titleToCamelCase("GPU planar regions"));
   }
}