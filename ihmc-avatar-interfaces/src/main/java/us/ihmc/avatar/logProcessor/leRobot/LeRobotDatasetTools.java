package us.ihmc.avatar.logProcessor.leRobot;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
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
      YoRegistry parent = yoRegistry.findRegistry(path);
      if (parent == null)
         return null;
      Predicate<YoRegistry> filter = reg -> reg.getName().endsWith(childSearchSuffix);
      YoRegistry result = parent.getChildren().stream().filter(filter).findFirst().orElse(null);
      if (result == null)
         return null;
      return path + "." + result.getName() + ".";
   }

   public static SideDependentList<List<String>> getRobotArmJointNames(RobotDefinition robotDefinition)
   {
      SideDependentList<List<String>> armJointNames = new SideDependentList<>();
      for (RigidBodyDefinition link : robotDefinition.getAllRigidBodies())
         if (link.getName().toLowerCase().contains("torso")) // found torso link
            for (JointDefinition joint : link.getChildrenJoints())
               for (RobotSide side : RobotSide.values)
                  if (joint.getName().toLowerCase().contains(side.getLowerCaseName())) // 1st arm joint
                  {
                     armJointNames.put(side, new ArrayList<>());
                     recursivelyAddArmJoints(joint, armJointNames.get(side));
                  }
      return armJointNames;
   }

   private static void recursivelyAddArmJoints(JointDefinition joint, List<String> names)
   {
      names.add(joint.getName());

      if (joint.getSuccessor().getChildrenJoints().size() == 1) // filter for nub & fingers
      {
         JointDefinition childJoint = joint.getSuccessor().getChildrenJoints().get(0);
         recursivelyAddArmJoints(childJoint, names);
      }
   }

   /** Unused for now. Keeping it around in case we need it later. */
   public static void getJointAngleVariables(RobotDefinition robotDefinition, YoRegistry rootRegistry, LogSession session)
   {
      SideDependentList<YoDouble[]> jointAnglesCurrent = new SideDependentList<>();
      SideDependentList<YoDouble[]> jointAnglesDesired = new SideDependentList<>();

      SideDependentList<List<String>> robotArmJointNames = LeRobotDatasetTools.getRobotArmJointNames(robotDefinition);
      for (RobotSide side : robotArmJointNames.sides())
      {
         String kstModule = LeRobotDatasetTools.findRegistry(rootRegistry, "root.main", "IKStreamingRTThread");
         String kstController = kstModule + "KinematicsStreamingToolboxController.HumanoidKinematicsToolboxController.";
         String capitalizedRobotName = StringUtils.capitalize(session.getLogProperties().getModel().getNameAsString());
         String hwPosition = kstModule + "%sROS2HardwareCommunication.".formatted(capitalizedRobotName);
         String ikSolver = kstController + "WholeBodyControllerCore.WholeBodyInverseKinematicsSolver.";
         List<String> armJointNames = robotArmJointNames.get(side);
         YoDouble[] currentState = new YoDouble[armJointNames.size()];
         YoDouble[] desiredState = new YoDouble[armJointNames.size()];
         for (int i = 0; i < armJointNames.size(); i++)
            if (rootRegistry.findVariable("%sMotorState_Position_%s_%s".formatted(hwPosition,
                                                                                  armJointNames.get(i),
                                                                                  capitalizedRobotName)) instanceof YoDouble variable)
               currentState[i] = variable;
         for (int i = 0; i < armJointNames.size(); i++)
            if (rootRegistry.findVariable("%sq_qp_%s".formatted(ikSolver, armJointNames.get(i))) instanceof YoDouble variable)
               desiredState[i] = variable;
         jointAnglesCurrent.put(side, currentState);
         jointAnglesDesired.put(side, desiredState);
      }
   }
}
