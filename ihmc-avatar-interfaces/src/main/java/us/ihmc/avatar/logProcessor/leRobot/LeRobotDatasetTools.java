package us.ihmc.avatar.logProcessor.leRobot;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.util.function.Predicate;

/**
 * Assorted tools.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotDatasetTools
{
   public static List<Path> findLeRobotDatasetSubdirectories(Path startDirectory)
   {
      try (var paths = Files.walk(startDirectory))
      {
         return paths.filter(Files::isDirectory).filter(directory -> Files.exists(directory.resolve("meta/info.json"))).toList();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error searching for dataset directories with meta/info.json", e);
      }
   }

   public static void appendLine(Path path, String line)
   {
      ExceptionTools.handle(() -> Files.writeString(path, "%s%n".formatted(line.trim()), StandardOpenOption.APPEND), DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public static String findRegistry(YoRegistry yoRegistry, String path, String childSearchSuffix)
   {
      Predicate<YoRegistry> filter = reg -> reg.getName().endsWith(childSearchSuffix);
      YoRegistry result = yoRegistry.findRegistry(path).getChildren().stream().filter(filter).findFirst().orElse(null);
      return path + "." + result.getName() + ".";
   }

   public static SideDependentList<String> getRobotHandNames(RobotDefinition robotDefinition)
   {
      SideDependentList<String> handLinkNames = new SideDependentList<>();
      for (RigidBodyDefinition link : robotDefinition.getAllRigidBodies())
      {
         String linkName = link.getName().toLowerCase();
         if (link.getChildrenJoints().size() != 1 && (linkName.contains("wrist") || linkName.contains("gripper")))
            for (RobotSide side : RobotSide.values)
               if (linkName.contains(side.getLowerCaseName()))
                  handLinkNames.put(side, link.getName());
      }
      return handLinkNames;
   }
}
