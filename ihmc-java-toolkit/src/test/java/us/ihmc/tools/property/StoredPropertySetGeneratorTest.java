package us.ihmc.tools.property;

public class StoredPropertySetGeneratorTest
{
   public static void main(String[] args)
   {
      StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(StoredPropertySetGeneratorTest.class,
                                                                                    "ihmc-open-robotics-software",
                                                                                    "ihmc-java-toolkit/src/test/resources",
                                                                                    "ihmc-java-toolkit/src/test/java");
      generator.generate();
   }
}
