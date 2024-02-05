package us.ihmc.tools;

import java.nio.file.Path;
import java.nio.file.Paths;

public class IHMCCommonPaths
{
   private static final String USER_HOME_STRING = System.getProperty("user.home");
   public static final Path USER_HOME_DIRECTORY = Paths.get(USER_HOME_STRING);

   public static final String DOT_IHMC_DIRECTORY_NAME = ".ihmc";
   public static final Path DOT_IHMC_DIRECTORY = USER_HOME_DIRECTORY.resolve(DOT_IHMC_DIRECTORY_NAME);

   public static final String LOGS_DIRECTORY_NAME = "logs";
   public static final Path LOGS_DIRECTORY = DOT_IHMC_DIRECTORY.resolve(LOGS_DIRECTORY_NAME);

   public static final String PERCEPTION_LOGS_DIRECTORY_NAME = "perception";
   public static final Path PERCEPTION_LOGS_DIRECTORY = LOGS_DIRECTORY.resolve(PERCEPTION_LOGS_DIRECTORY_NAME);

   public static final String WEIGHTS_DIRECTORY_NAME = "weights";
   public static final Path WEIGHTS_DIRECTORY = PERCEPTION_LOGS_DIRECTORY.resolve(WEIGHTS_DIRECTORY_NAME);

   public static final String MISSION_CONTROL_LOGS_DIRECTORY_NAME = "mission-control";
   public static final Path MISSION_CONTROL_LOGS_DIRECTORY = LOGS_DIRECTORY.resolve(MISSION_CONTROL_LOGS_DIRECTORY_NAME);
}
