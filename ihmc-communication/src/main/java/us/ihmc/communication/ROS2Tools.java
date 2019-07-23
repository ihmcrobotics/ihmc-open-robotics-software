package us.ihmc.communication;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.function.Supplier;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.RealtimeRos2Subscription;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2QosProfile;
import us.ihmc.ros2.Ros2Subscription;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

public class ROS2Tools
{
   public static final ROS2ModuleIdentifier HUMANOID_CONTROLLER = new ROS2ModuleIdentifier("ihmc_controller", "/humanoid_control");
   public static final ROS2ModuleIdentifier REA = new ROS2ModuleIdentifier("REA_module", "/rea");
   public static final ROS2ModuleIdentifier LLAMA = new ROS2ModuleIdentifier("llama_network", "/quadruped_control");

   public static final String IHMC_ROS_TOPIC_PREFIX = "/ihmc";
   public static final String OUTPUT_ROS_TOPIC_PREFIX = "/output";
   public static final String INPUT_ROS_TOPIC_PREFIX = "/input";

   public static final String HUMANOID_CONTROL_MODULE = HUMANOID_CONTROLLER.getModuleTopicQualifier();
   public static final String QUADRUPED_CONTROL_MODULE = LLAMA.getModuleTopicQualifier();

   public static final String FOOTSTEP_PLANNER_TOOLBOX = "/toolbox/footstep_plan";
   public static final String HEIGHT_QUADTREE_TOOLBOX = "/toolbox/height_quad_tree";
   public static final String KINEMATICS_TOOLBOX = "/toolbox/ik";
   public static final String KINEMATICS_PLANNING_TOOLBOX = "/toolbox/ik_planning";
   public static final String WHOLE_BODY_TRAJECTORY_TOOLBOX = "/toolbox/ik_trajectory";
   public static final String WALKING_PREVIEW_TOOLBOX = "/toolbox/walking_controller_preview";

   public static final String STEP_TELEOP_TOOLBOX = "/toolbox/teleop/step_teleop";
   public static final String QUADRUPED_SUPPORT_REGION_PUBLISHER = "/quadruped_support_region_publisher";

   public static final String BIPED_SUPPORT_REGION_PUBLISHER = "/bipedal_support_region_publisher";
   public static final String BEHAVIOR_MODULE = "/behavior";
   public static final String REA_MODULE = REA.getModuleTopicQualifier();
   public static final String REA_CUSTOM_REGION_QUALIFIER = "/custom_region";

   public static final String SREA_MODULE = "/srea";

   public enum ROS2TopicQualifier
   {
      INPUT(INPUT_ROS_TOPIC_PREFIX), OUTPUT(OUTPUT_ROS_TOPIC_PREFIX);

      private final String name;

      ROS2TopicQualifier(String name)
      {
         this.name = name;
      }

      @Override
      public String toString()
      {
         return name;
      }
   };

   /**
    * Generator to automatically generate a topic name based on the type of message to send.
    */
   public static interface MessageTopicNameGenerator
   {
      String generateTopicName(Class<?> messageType);
   }

   public final static ExceptionHandler RUNTIME_EXCEPTION = e -> {
      throw new RuntimeException(e);
   };
   public final static String NAMESPACE = "/us/ihmc"; // ? no idea what this does

   private static final int DOMAIN_ID = new RTPSCommunicationFactory().getDomainId();

   /**
    * Creates a ROS2 node that shares the same implementation as a real-time node <b>but that should
    * not be run in a real-time environment</b>.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName the name of the new ROS node.
    * @return the ROS node.
    */
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createRealtimeRos2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
   }

   /**
    * Creates a ROS2 node that shares the same implementation as a real-time node <b>but that should
    * not be run in a real-time environment</b>.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName the name of the new ROS node.
    * @param exceptionHandler how to handle exceptions thrown during the instantiation.
    * @return the ROS node.
    */
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      return createRealtimeRos2Node(pubSubImplementation, new PeriodicNonRealtimeThreadSchedulerFactory(), nodeName, exceptionHandler);
   }

   /**
    * Creates a ROS2 node that is meant to run in real-time environment if the given
    * {@code periodicThreadSchedulerFactory} is a {@link PeriodicRealtimeThreadSchedulerFactory}.
    *
    * @param pubSubImplementation the implementation to use.
    * @param periodicThreadSchedulerFactory the factory used to create a periodic thread.
    * @param nodeName the name of the new ROS node.
    * @return the ROS node.
    */
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory, String nodeName)
   {
      return createRealtimeRos2Node(pubSubImplementation, periodicThreadSchedulerFactory, nodeName, RUNTIME_EXCEPTION);
   }

   /**
    * Creates a ROS2 node that is meant to run in real-time environment if the given
    * {@code periodicThreadSchedulerFactory} is a {@link PeriodicRealtimeThreadSchedulerFactory}.
    *
    * @param pubSubImplementation the implementation to use.
    * @param periodicThreadSchedulerFactory the factory used to create a periodic thread.
    * @param nodeName the name of the new ROS node.
    * @param exceptionHandler how to handle exceptions thrown during the instantiation.
    * @return the ROS node.
    */
   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory, String nodeName,
                                                         ExceptionHandler exceptionHandler)
   {
      try
      {
         return new RealtimeRos2Node(pubSubImplementation, periodicThreadSchedulerFactory, nodeName, NAMESPACE, DOMAIN_ID);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static Ros2Node createRos2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createRos2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
   }

   public static Ros2Node createRos2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new Ros2Node(pubSubImplementation, nodeName, NAMESPACE, DOMAIN_ID);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> Ros2Subscription<T> createCallbackSubscription(Ros2Node ros2Node, Class<T> messageType, MessageTopicNameGenerator topicNameGenerator,
                                                                    NewMessageListener<T> newMessageListener)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener);
   }

   public static <T> Ros2Subscription<T> createCallbackSubscription(Ros2Node ros2Node, Class<T> messageType, String topicName,
                                                                    NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, messageType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> Ros2Subscription<T> createCallbackSubscription(Ros2Node ros2Node, Class<T> messageType, String topicName,
                                                                    NewMessageListener<T> newMessageListener, ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageType);
         return ros2Node.createSubscription(topicDataType, newMessageListener, topicName, Ros2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> Ros2QueuedSubscription<T> createQueuedSubscription(Ros2Node ros2Node, Class<T> messageType,
                                                                        MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createQueuedSubscription(ros2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> Ros2QueuedSubscription<T> createQueuedSubscription(Ros2Node ros2Node, Class<T> messageType,
                                                                        String topicName)
   {
      return createQueuedSubscription(ros2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> Ros2QueuedSubscription<T> createQueuedSubscription(Ros2Node ros2Node, Class<T> messageType, String topicName, ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageType);
         Ros2QueuedSubscription<T> ros2QueuedSubscription = new Ros2QueuedSubscription<>(topicDataType, 10);
         ros2QueuedSubscription.setRos2Subscription(ros2Node.createSubscription(topicDataType, ros2QueuedSubscription, topicName, Ros2QosProfile.DEFAULT()));
         return ros2QueuedSubscription;
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, MessageTopicNameGenerator topicNameGenerator,
                                                     NewMessageListener<T> newMessageListener)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      createCallbackSubscription(realtimeRos2Node, messageType, topicName, newMessageListener);
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeRos2Node, messageType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName,
                                                     NewMessageListener<T> newMessageListener, ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageType);
         realtimeRos2Node.createCallbackSubscription(topicDataType, topicName, newMessageListener, Ros2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> RealtimeRos2Subscription<T> createQueuedSubscription(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createQueuedSubscription(realtimeRos2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> RealtimeRos2Subscription<T> createQueuedSubscription(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName)
   {
      return createQueuedSubscription(realtimeRos2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> RealtimeRos2Subscription<T> createQueuedSubscription(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName, ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageType);
         return realtimeRos2Node.createQueuedSubscription(topicDataType, topicName, Ros2QosProfile.DEFAULT(), 10);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node, Class<T> messageType,
                                                                  MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createPublisher(realtimeRos2Node, messageType, topicName);
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName)
   {
      return createPublisher(realtimeRos2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node, Class<T> messageType, String topicName,
                                                                  ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageType);
         return new IHMCRealtimeROS2Publisher<T>(realtimeRos2Node.createPublisher(topicDataType, topicName, Ros2QosProfile.DEFAULT(), 10));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2Node ros2Node, Class<T> messageType, MessageTopicNameGenerator topicNameGenerator)
   {
      String topicName = topicNameGenerator.generateTopicName(messageType);
      return createPublisher(ros2Node, messageType, topicName);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2Node ros2Node, Class<T> messageType, String robotName, ROS2ModuleIdentifier identifier)
   {
      return new IHMCROS2Publisher<>(ros2Node, messageType, robotName, identifier);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2Node ros2Node, Class<T> messageType, String topicName)
   {
      return createPublisher(ros2Node, messageType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2Node ros2Node, Class<T> messageType, String topicName, ExceptionHandler exceptionHandler)
   {
      try
      {
         TopicDataType<T> topicDataType = newMessageTopicDataTypeInstance(messageType);
         return new IHMCROS2Publisher<T>(ros2Node.createPublisher(topicDataType, topicName, Ros2QosProfile.DEFAULT()));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   /**
    * Creates a default topic name generator that uses {@value #IHMC_ROS_TOPIC_PREFIX} as prefix.
    * <p>
    * This generator is not great at all as the topic name does not include the name of the robot,
    * the name of the module, nor info about whether the topic is an input or output of the module
    * declaring it.
    * </p>
    * <p>
    * Here is a couple examples for this generator:
    * <ul>
    * <li>For {@code TextToSpeechPacket} this generates the topic name:
    * {@code "/ihmc/text_to_speech"}.
    * <li>For {@code ArmTrajectoryMessage} this generates the topic name:
    * {@code "/ihmc/arm_trajectory"}.
    * </ul>
    * </p>
    *
    * @return the default generator.
    */
   public static MessageTopicNameGenerator getDefaultTopicNameGenerator()
   {
      return ROS2Tools::generateDefaultTopicName;
   }

   /**
    * Creates a default topic name generator that uses {@value #IHMC_ROS_TOPIC_PREFIX} plus the name
    * of the robot as prefix.
    * <p>
    * This generator is not great as the topic name does not include the name of the module, nor
    * info about whether the topic is an input or output of the module declaring it.
    * </p>
    * <p>
    * Here is a couple examples for this generator:
    * <ul>
    * <li>For {@code TextToSpeechPacket} when running Atlas this generates the topic name:
    * {@code "/ihmc/atlas/text_to_speech"}.
    * <li>For {@code ArmTrajectoryMessage} when running Valkyrie this generates the topic name:
    * {@code "/ihmc/valkyrie/arm_trajectory"}.
    * </ul>
    * </p>
    *
    * @return the default generator.
    */
   public static MessageTopicNameGenerator getDefaultTopicNameGenerator(String robotName)
   {
      return messageType -> generateDefaultTopicName(messageType, robotName);
   }

   /**
    * Creates a default topic name generator that uses {@value #IHMC_ROS_TOPIC_PREFIX} plus the name
    * of the robot as prefix.
    * <p>
    * This generator is not great as the topic name does not include the name of the module, nor
    * info about whether the topic is an input or output of the module declaring it.
    * </p>
    * <p>
    * Here is a couple examples for this generator:
    * <ul>
    * <li>For {@code TextToSpeechPacket} when running Atlas this generates the topic name:
    * {@code "/ihmc/atlas/text_to_speech"}.
    * <li>For {@code ArmTrajectoryMessage} when running Valkyrie this generates the topic name:
    * {@code "/ihmc/valkyrie/arm_trajectory"}.
    * </ul>
    * </p>
    *
    * @return the generator.
    */
   public static MessageTopicNameGenerator getTopicNameGenerator(String robotName, String moduleName, ROS2TopicQualifier qualifier)
   {
      return messageType -> generateDefaultTopicName(messageType, robotName, moduleName, qualifier);
   }

   /**
    * Generates a default topic name using the class name of the message, for instance:<br>
    * For {@code TextToSpeechPacket} this generates the topic name: {@code "/ihmc/text_to_speech"}.
    *
    * @param messageClass the class of the message to generate the topic name for.
    * @return the topic name.
    */
   public static String generateDefaultTopicName(Class<?> messageClass)
   {
      return generateDefaultTopicName(messageClass, null, null, null);
   }

   /**
    * Generates a default topic name using the class name of the message, for instance:<br>
    * For {@code TextToSpeechPacket} when running Valkyrie this generates the topic name:
    * {@code "/ihmc/valkyrie/text_to_speech"}.
    *
    * @param messageClass the class of the message to generate the topic name for.
    * @return the topic name.
    */
   public static String generateDefaultTopicName(Class<?> messageClass, String robotName)
   {
      return generateDefaultTopicName(messageClass, robotName, null, null);
   }

   /**
    * Generates a topic name in a similar way to
    * {@link #generateDefaultTopicName(Class, String)}:<br>
    * For {@code TextToSpeechPacket} when running Valkyrie this generates the topic name:<br>
    * {@code "/ihmc/valkyrie/" + moduleName.toLowerCase() + qualifier + "/text_to_speech"}.
    *
    *
    * @param messageClass the class of the message to generate the topic name for.
    * @return the topic name.
    */
   public static String generateDefaultTopicName(Class<?> messageClass, String robotName, String moduleName, ROS2TopicQualifier qualifier)
   {
      String prefix = IHMC_ROS_TOPIC_PREFIX;

      if (robotName != null && !robotName.isEmpty())
      {
         if (!robotName.startsWith("/"))
            prefix += "/" + robotName.toLowerCase();
         else
            prefix += robotName.toLowerCase();
      }

      if (moduleName != null && !moduleName.isEmpty())
      {
         if (!moduleName.startsWith("/"))
            prefix += "/" + moduleName.toLowerCase();
         else
            prefix += moduleName.toLowerCase();
      }

      if (qualifier != null)
         prefix += qualifier.toString();

      return appendTypeToTopicName(prefix, messageClass);
   }

   /**
    * Appends to the given prefix, the simple name of the message class in a ROS topic fashion, for
    * instance:
    * <ul>
    * <li>{@code MessageCollection} becomes: {@code "/message_collection"}.
    * <li>{@code TextToSpeechPacket} becomes: {@code "/text_to_speech"}.
    * <li>{@code WholeBodyTrajectoryMessage} becomes: {@code "/whole_body_trajectory"}.
    * </ul>
    *
    * @param prefix the prefix of the returned {@code String}.
    * @param messageClass used for its simple name to generate a suffix.
    * @return the composed {@code String}.
    */
   public static String appendTypeToTopicName(String prefix, Class<?> messageClass)
   {
      String topicName = messageClass.getSimpleName();
      topicName = StringUtils.removeEnd(topicName, "Packet"); // This makes BehaviorControlModePacket => BehaviorControlMode
      topicName = StringUtils.removeEnd(topicName, "Message"); // This makes ArmTrajectoryMessage => ArmTrajectory
      topicName = "/" + toROSTopicFormat(topicName); // This makes ArmTrajectory => arm_trajectory & handle acronyms as follows: REAStateRequest => rea_state_request
      return prefix + topicName;
   }

   /**
    * Converts the given {@code String} from a camel-case convention to ROS topic name convention which
    * is lower-case with underscores.
    * <p>
    * This method in general behaves as from Guava:
    * {@code CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, camelCase)}. The only difference is
    * the handling of acronyms. For instance, given the {@code String} {@code "REAStatusMessage"}:
    * <ul>
    * <li>result from Guava: {@code "r_e_a_status_message"} which breaks the acronym 'REA'.
    * <li>result from this method: {@code "rea_status_message"} which conserves the acronym as one
    * word.
    * </p>
    * 
    * @param camelCase the camel-case {@code String} to be converted.
    * @return the converted {@code String} using lower-case with underscores.
    */
   public static String toROSTopicFormat(String camelCase)
   {
      if (camelCase == null)
         return null;

      if (camelCase.isEmpty())
         return camelCase;

      if (camelCase.length() == 1)
         return camelCase.toLowerCase();

      StringBuilder stringBuilder = new StringBuilder();

      boolean isNewWord = true;
      boolean isPreviousUpper = false;

      for (int charIndex = 0; charIndex < camelCase.length(); charIndex++)
      {
         boolean isCharUpper = Character.isUpperCase(camelCase.charAt(charIndex));

         if (charIndex == 0 || !isCharUpper)
         {
            isNewWord = false;
         }
         else if (!isPreviousUpper)
         { // This is clearly the beginning of new word as the previous character is lower-case.
            isNewWord = true;
         }
         else
         { // This might still be an acronym.
            int nextIndex = charIndex + 1;
            boolean isNextUpper = nextIndex == camelCase.length() || Character.isUpperCase(camelCase.charAt(nextIndex));
            isNewWord = !isNextUpper; // If next is lower-case, this is clearly a new word, but otherwise we're going through an acronym.
         }

         isPreviousUpper = isCharUpper;

         if (isNewWord)
            stringBuilder.append("_"); // Any new word but the first, starts with an underscore.
         stringBuilder.append(Character.toLowerCase(camelCase.charAt(charIndex)));
      }
      return stringBuilder.toString();
   }

   public static final String pubSubTypeGetterName = "getPubSubType";

   public static <T> T newMessageInstance(Class<T> messageType)
   {
      try
      {
         return messageType.newInstance();
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         throw new RuntimeException("Something went wrong when invoking " + messageType.getSimpleName() + "'s empty constructor.", e);
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static <T> TopicDataType<T> newMessageTopicDataTypeInstance(Class<T> messageType)
   {
      Method pubSubTypeGetter;

      try
      {
         pubSubTypeGetter = messageType.getDeclaredMethod(pubSubTypeGetterName);
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Something went wrong when looking up for the method " + messageType.getSimpleName() + "." + pubSubTypeGetterName + "().",
                                    e);
      }

      TopicDataType<T> topicDataType;

      try
      {
         topicDataType = (TopicDataType<T>) ((Supplier) pubSubTypeGetter.invoke(newMessageInstance(messageType))).get();
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         throw new RuntimeException("Something went wrong when invoking the method " + messageType.getSimpleName() + "." + pubSubTypeGetterName + "().", e);
      }
      return topicDataType;
   }
}
