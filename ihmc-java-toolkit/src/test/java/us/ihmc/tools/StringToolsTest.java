package us.ihmc.tools;


import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class StringToolsTest
{
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

   @Test
   public void testPascalCaseToSentenceCase()
   {
      assertEquals("Test pascal to sentence case", StringTools.pascalCaseToSentenceCase("TestPascalToSentenceCase"));
      assertEquals("Test pascal to sentence case but there is an acronym such as UI", StringTools.pascalCaseToSentenceCase("TestPascalToSentenceCaseButThereIsAnAcronymSuchAsUI"));
      assertEquals("It should handle stuff like 3D as well", StringTools.pascalCaseToSentenceCase("ItShouldHandleStuffLike3DAsWell"));
      assertNotEquals("It cannot handle all cases and struggles with some acronyms like UI and 3D in certain order", StringTools.pascalCaseToSentenceCase("ItCannotHandleAssCasesAndStrugglesWithSomeAcronymsLikeUIAnd3DInCertainOrder"));
   }
}