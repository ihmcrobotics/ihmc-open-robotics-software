package us.ihmc.tools.property;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.nio.file.Paths;

public class StoredPropertySetGeneratorTest
{
   public static void main(String[] args)
   {
      WorkspaceDirectory javaDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-java-toolkit/src/test/java");
      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-java-toolkit/src/test/resources");

//      Resou
      Class<StoredPropertySetGeneratorTest> clazz = StoredPropertySetGeneratorTest.class;

      String iniFileName = StringUtils.uncapitalize(clazz.getSimpleName()) + ".ini";
      clazz.getResourceAsStream(iniFileName);

      StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(clazz, javaDirectory);
      generator.generate();
   }
}
