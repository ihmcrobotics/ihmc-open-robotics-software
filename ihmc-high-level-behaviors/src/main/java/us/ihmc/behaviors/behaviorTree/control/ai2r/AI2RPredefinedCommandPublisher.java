package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.msg.dds.AI2RActionFailureMessage;
import behavior_msgs.msg.dds.AI2RCommandMessage;
import behavior_msgs.msg.dds.AI2RNavigationMessage;
import behavior_msgs.msg.dds.AI2RObjectMessage;
import behavior_msgs.msg.dds.AI2RStatusMessage;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Java equivalent of the predefined Python behavior coordinator.
 * <p>
 * Subscribes to AI2R status, then publishes the next predefined AI2R command when the current behavior is complete.
 */
public class AI2RPredefinedCommandPublisher
{
   private static final long COMMAND_REPUBLISH_INTERVAL_NANOS = 500_000_000L;

   private static final String PREDEFINED_PLAN =
         "behavior_list = ["
         + "SCAN, "
         + "GOTO (to the person to the right of the barrier), "
         + "RECEIVE CHARGE, "
         + "GOTO (to the front of the DoorPanel), "
         + "PLACE CHARGE ON DOOR, "
         + "GOTO (behind the barrier)"
         + "]";

   private void planMission()
   {
      LogTools.info("[PREDEFINED] Using hardcoded mission plan.");
      LogTools.info("Plan: {}", PREDEFINED_PLAN);

      planQueue.clear();
      planQueue.add(new PlanStep("SCAN", "SCAN"));
      planQueue.add(new PlanStep("GOTO", "GOTO (to the person to the right of the barrier)"));
      planQueue.add(new PlanStep("RECEIVE CHARGE", "RECEIVE CHARGE"));
      planQueue.add(new PlanStep("GOTO", "GOTO (to the front of the DoorPanel)"));
      planQueue.add(new PlanStep("PLACE CHARGE ON DOOR", "PLACE CHARGE ON DOOR"));
      planQueue.add(new PlanStep("GOTO", "GOTO (behind the barrier)"));
   }

   private static final String[] PREDEFINED_SCAN_TARGETS = {"person", "charge", "traffic_barrier", "door_panel"};

   // Order matters: first match wins, same behavior as the Python predefined coordinator.
   private static final List<GotoRule> PREDEFINED_GOTO = List.of(
         new GotoRule("person", new GotoConfig("person", "DEFAULT", "-", "traffic_barrier", "LEFT", "CLOSE")),
         new GotoRule("doorpanel", new GotoConfig("door_panel", "FRONT", "-", "-", "-", "CLOSE")),
         new GotoRule("barrier", new GotoConfig("traffic_barrier", "BEHIND", "door_panel", "-", "-", "CLOSE")));

   private final ROS2Node ros2Node;
   private final ROS2Publisher<AI2RCommandMessage> commandPublisher;
   private final ArrayDeque<PlanStep> planQueue = new ArrayDeque<>();

   private boolean initialized = false;
   private boolean loggedFailure = false;
   private String nextBehavior = "";
   private String nextDescription = "";
   private String lastCompletion = null;
   private String commandedBehavior = "";
   private AI2RCommandMessage pendingCommand = null;
   private boolean waitingForExecutionAck = false;
   private long lastCommandPublishNanos = 0L;

   public AI2RPredefinedCommandPublisher()
   {
      ros2Node = new ROS2NodeBuilder().build("ai2r_predefined_behavior_coordinator");
      commandPublisher = ros2Node.createPublisher(AutonomyAPI.AI2R_COMMAND);

      ros2Node.createSubscription2(AutonomyAPI.AI2R_STATUS, this::onStatus);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));

      LogTools.info("AI2RPredefinedCommandPublisher ready.");
   }

   private synchronized void onStatus(AI2RStatusMessage status)
   {
      List<String> sceneNames = new ArrayList<>();
      for (int i = 0; i < status.getObjects().size(); i++)
      {
         sceneNames.add(status.getObjects().get(i).getObjectNameAsString());
      }

      List<String> availableBehaviors = new ArrayList<>();
      for (int i = 0; i < status.getAvailableBehaviors().size(); i++)
      {
         availableBehaviors.add(status.getAvailableBehaviors().getString(i));
      }

      if (!initialized)
      {
         LogTools.info("Scene objects: {}", sceneNames);
         LogTools.info("Available behaviors: {}", availableBehaviors);
      }

      String completedBehavior = status.getCompletedBehaviorAsString();
      if (!completedBehavior.equals("-") && !completedBehavior.equals(lastCompletion))
      {
         LogTools.info("Completed: {}", completedBehavior);
         lastCompletion = completedBehavior;
      }

      String failedBehavior = status.getFailedBehaviorAsString();
      if (!"-".equals(failedBehavior) && !loggedFailure)
      {
         logFailure(status.getFailure(), failedBehavior);
         loggedFailure = true;
      }

      String behaviorInProgress = status.getBehaviorInProgressAsString();
      if (waitingForExecutionAck && behaviorNamesMatch(behaviorInProgress, commandedBehavior))
      {
         waitingForExecutionAck = false;
         pendingCommand = null;
         LogTools.info("Execution acknowledged for commanded behavior: {}", commandedBehavior);
      }
      else if (waitingForExecutionAck)
      {
         maybeRepublishPendingCommand();
      }

      if (!"-".equals(behaviorInProgress))
         return;

      if (waitingForExecutionAck)
         return;

      if (planQueue.isEmpty())
      {
         if (initialized)
         {
            LogTools.info("Mission complete. All behaviors executed.");
            return;
         }

         planMission();
      }

      if (planQueue.isEmpty())
         return;


      PlanStep step = planQueue.removeFirst();
      nextBehavior = step.behavior;
      nextDescription = step.description;

      LogTools.info("Commanding [{}]: {} ({} remaining)", nextBehavior, nextDescription, planQueue.size());

      AI2RCommandMessage command = buildCommand(nextBehavior, nextDescription, status);
      commandedBehavior = command.getBehaviorToExecuteAsString();
      pendingCommand = command;
      waitingForExecutionAck = true;
      commandPublisher.publish(command);
      lastCommandPublishNanos = System.nanoTime();

      initialized = true;
      loggedFailure = false;
   }

   private AI2RCommandMessage buildCommand(String behavior, String description, AI2RStatusMessage status)
   {
      AI2RCommandMessage command = new AI2RCommandMessage();
      command.setBehaviorToExecute(behavior);
      command.setAdaptingBehavior(false);

      if ("SCAN".equals(behavior))
         return buildScan(command);
      if ("GOTO".equals(behavior))
         return buildGoto(command, description, status);
      if ("RECEIVE OBJECT".equals(behavior) || "RECEIVE CHARGE".equals(behavior))
         return buildReceiveObject(command);

      return command;
   }

   private AI2RCommandMessage buildScan(AI2RCommandMessage command)
   {
      command.setAdaptingBehavior(true);
      for (String target : PREDEFINED_SCAN_TARGETS)
      {
         command.getScan().getObjectNames().add(target);
      }
      LogTools.info("[PREDEFINED] SCAN targets: {}", List.of(PREDEFINED_SCAN_TARGETS));
      return command;
   }

   private AI2RCommandMessage buildGoto(AI2RCommandMessage command, String description, AI2RStatusMessage status)
   {
      command.setAdaptingBehavior(true);

      String descLower = description.toLowerCase();
      GotoConfig config = null;
      for (GotoRule rule : PREDEFINED_GOTO)
      {
         if (descLower.contains(rule.keyword))
         {
            config = rule.config;
            break;
         }
      }

      if (config == null)
      {
         LogTools.error("No predefined GOTO response for description: '{}'", description);
         return command;
      }

      LogTools.info("[PREDEFINED] GOTO params: {}", config);

      List<String> sceneObjectNames = new ArrayList<>();
      List<Pose3D> sceneObjectPoses = new ArrayList<>();
      for (int i = 0; i < status.getObjects().size(); i++)
      {
         AI2RObjectMessage object = status.getObjects().get(i);
         sceneObjectNames.add(object.getObjectNameAsString());
         sceneObjectPoses.add(object.getObjectPoseInWorld());
      }

      Pose3D robotPose = status.getRobotMidFeetUnderPelvisPoseInWorld();

      String target = config.targetObject;
      if (!sceneObjectNames.contains(target))
      {
         String selected = selectTargetObject(config,
                                              sceneObjectNames,
                                              sceneObjectPoses,
                                              robotPose);
         if (selected != null)
            target = selected;
      }

      LogTools.info("GOTO resolved target: {}", target);

      command.getNavigation().setTargetObject(target);
      command.getNavigation().setDistanceToObject(1.0);
      command.getNavigation().setSpatialRelation(spatialRelationFromString(config.spatialRelationGoto));

      String povObject = config.povObjectGoto;
      if (command.getNavigation().getSpatialRelation() == AI2RNavigationMessage.DEFAULT || "-".equals(povObject) || povObject.isBlank())
         povObject = "Walking";

      command.getNavigation().setPovObject(povObject);
      return command;
   }

   private AI2RCommandMessage buildReceiveObject(AI2RCommandMessage command)
   {
      command.setAdaptingBehavior(true);
      command.getReceiveObject().setObjectName("charge");
      command.getReceiveObject().setSide(RobotSide.RIGHT.toByte());
      LogTools.info("[PREDEFINED] RECEIVE OBJECT params: object_name=charge, side=RIGHT");
      return command;
   }

   private void logFailure(AI2RActionFailureMessage failure, String failedBehavior)
   {
      double norm = Math.sqrt(Math.pow(failure.getPositionError().getX(), 2)
                              + Math.pow(failure.getPositionError().getY(), 2)
                              + Math.pow(failure.getPositionError().getZ(), 2));

      String missingFrame = failure.getMissingFrame() ? failure.getActionFrameAsString() : "null";
      String collision = "-".equals(failure.getCollisionNameAsString()) ? "null" : failure.getCollisionNameAsString();
      String positionError = norm > failure.getPositionTolerance() ? Double.toString(norm) : "null";
   }

   private static byte spatialRelationFromString(String relation)
   {
      return switch (relation)
      {
         case "FRONT" -> AI2RNavigationMessage.FRONT;
         case "BEHIND" -> AI2RNavigationMessage.BEHIND;
         case "LEFT" -> AI2RNavigationMessage.LEFT;
         case "RIGHT" -> AI2RNavigationMessage.RIGHT;
         default -> AI2RNavigationMessage.DEFAULT;
      };
   }

   private static String selectTargetObject(GotoConfig config,
                                            List<String> sceneObjectNames,
                                            List<Pose3D> sceneObjectPoses,
                                            Pose3D robotPose)
   {
      List<Integer> candidates = new ArrayList<>();
      for (int i = 0; i < sceneObjectNames.size(); i++)
      {
         String name = sceneObjectNames.get(i);
         if (isBasePlusDigits(name, config.targetObject))
            candidates.add(i);
      }

      if (candidates.isEmpty())
         return null;
      if (candidates.size() == 1)
         return sceneObjectNames.get(candidates.get(0));

      String refName = ("-".equals(config.spatiallyRelatedObject) || config.spatiallyRelatedObject.isBlank()) ? "Robot" : config.spatiallyRelatedObject;
      Pose3D refPose = getPoseByName(refName, sceneObjectNames, sceneObjectPoses, robotPose);
      if (refPose == null)
         return null;

      if ("DEFAULT".equals(config.spatialRelationObject) || "-".equals(config.spatialRelationObject) || config.spatialRelationObject.isBlank())
      {
         Comparator<Integer> comparator = Comparator.comparingDouble(i -> distance(sceneObjectPoses.get(i), refPose));
         int chosen = "CLOSE".equals(config.classDiscriminator)
               ? candidates.stream().min(comparator).orElse(candidates.get(0))
               : candidates.stream().max(comparator).orElse(candidates.get(0));
         return sceneObjectNames.get(chosen);
      }

      double refX = refPose.getX();
      double refY = refPose.getY();
      double refZ = refPose.getZ();

      double robotX = robotPose.getX();
      double robotY = robotPose.getY();
      double robotZ = robotPose.getZ();

      double dirX = robotX - refX;
      double dirY = robotY - refY;
      double dirZ = robotZ - refZ;
      double dirNorm = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);

      if (dirNorm < 1.0e-6)
         return sceneObjectNames.get(candidates.get(0));

      dirX /= dirNorm;
      dirY /= dirNorm;
      dirZ /= dirNorm;

      // left = cross([0,0,1], direction)
      double leftX = -dirY;
      double leftY = dirX;
      double leftZ = 0.0;
      double leftNorm = Math.sqrt(leftX * leftX + leftY * leftY + leftZ * leftZ);
      if (leftNorm > 1.0e-6)
      {
         leftX /= leftNorm;
         leftY /= leftNorm;
         leftZ /= leftNorm;
      }

      List<QualifiedCandidate> qualified = new ArrayList<>();
      for (Integer candidateIndex : candidates)
      {
         Pose3D pose = sceneObjectPoses.get(candidateIndex);
         double offsetX = pose.getX() - refX;
         double offsetY = pose.getY() - refY;
         double offsetZ = pose.getZ() - refZ;

         double forwardDot = offsetX * dirX + offsetY * dirY + offsetZ * dirZ;
         double leftDot = offsetX * leftX + offsetY * leftY + offsetZ * leftZ;

         boolean keep = switch (config.spatialRelationObject)
         {
            case "BEHIND" -> forwardDot < -0.1;
            case "FRONT" -> forwardDot > 0.1;
            case "RIGHT" -> leftDot > 0.5;
            case "LEFT" -> leftDot < -0.5;
            default -> false;
         };

         if (keep)
            qualified.add(new QualifiedCandidate(candidateIndex, Math.sqrt(offsetX * offsetX + offsetY * offsetY + offsetZ * offsetZ)));
      }

      if (qualified.isEmpty())
         return null;

      Comparator<QualifiedCandidate> comparator = Comparator.comparingDouble(q -> q.distance);
      QualifiedCandidate chosen = "CLOSE".equals(config.classDiscriminator)
            ? qualified.stream().min(comparator).orElse(qualified.get(0))
            : qualified.stream().max(comparator).orElse(qualified.get(0));

      return sceneObjectNames.get(chosen.sceneIndex);
   }

   private static Pose3D getPoseByName(String name,
                                       List<String> sceneObjectNames,
                                       List<Pose3D> sceneObjectPoses,
                                       Pose3D robotPose)
   {
      if ("robot".equalsIgnoreCase(name))
         return robotPose;
      if ("-".equals(name) || name.isBlank())
         return null;

      for (int i = 0; i < sceneObjectNames.size(); i++)
      {
         if (sceneObjectNames.get(i).equals(name))
            return sceneObjectPoses.get(i);
      }

      return null;
   }

   private static boolean isBasePlusDigits(String candidate, String base)
   {
      if (!candidate.startsWith(base))
         return false;
      if (candidate.length() == base.length())
         return false;

      for (int i = base.length(); i < candidate.length(); i++)
      {
         if (!Character.isDigit(candidate.charAt(i)))
            return false;
      }
      return true;
   }

   private static double distance(Pose3D a, Pose3D b)
   {
      double dx = a.getX() - b.getX();
      double dy = a.getY() - b.getY();
      double dz = a.getZ() - b.getZ();
      return Math.sqrt(dx * dx + dy * dy + dz * dz);
   }

   private void destroy()
   {
      ros2Node.destroy();
   }

   private void maybeRepublishPendingCommand()
   {
      if (pendingCommand == null)
         return;

      long now = System.nanoTime();
      if (now - lastCommandPublishNanos < COMMAND_REPUBLISH_INTERVAL_NANOS)
         return;

      commandPublisher.publish(pendingCommand);
      lastCommandPublishNanos = now;
      LogTools.info("Republishing commanded behavior while waiting for ack: {}", commandedBehavior);
   }

   private static boolean behaviorNamesMatch(String a, String b)
   {
      if (a == null || b == null)
         return false;
      if (a.equals(b))
         return true;
      return false;
   }

   public static void main(String[] args) throws InterruptedException
   {
      new AI2RPredefinedCommandPublisher();

      // Keep the process alive for ROS2 callbacks.
      while (true)
      {
         Thread.sleep(1000);
      }
   }

   private record PlanStep(String behavior, String description)
   {
   }

   private record GotoConfig(String targetObject,
                             String spatialRelationGoto,
                             String povObjectGoto,
                             String spatiallyRelatedObject,
                             String spatialRelationObject,
                             String classDiscriminator)
   {
   }

   private record GotoRule(String keyword, GotoConfig config)
   {
   }

   private record QualifiedCandidate(int sceneIndex, double distance)
   {
   }
}
