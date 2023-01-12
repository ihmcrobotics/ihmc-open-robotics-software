package us.ihmc.tools;

import java.nio.file.Path;
import java.nio.file.Paths;

public class IHMCCommonPaths
{
   public static final String USER_HOME_STRING = System.getProperty("user.home");
   public static final Path USER_HOME_DIRECTORY = Paths.get(USER_HOME_STRING);

   public static final String DOT_IHMC_DIRECTORY_NAME = ".ihmc";
   public static final Path DOT_IHMC_DIRECTORY = USER_HOME_DIRECTORY.resolve(DOT_IHMC_DIRECTORY_NAME);

   public static final String LOGS_DIRECTORY_NAME = "logs";
   public static final Path LOGS_DIRECTORY = DOT_IHMC_DIRECTORY.resolve(LOGS_DIRECTORY_NAME);
}
