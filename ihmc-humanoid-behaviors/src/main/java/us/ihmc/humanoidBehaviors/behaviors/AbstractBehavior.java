package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.Pair;

import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.BehaviorService;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Any behavior needs to implement this abstract class. It helps in setting up the communications to
 * receive and send packets to the other modules as the controller or the network processor.
 *
 */
public abstract class AbstractBehavior implements RobotController
{
   public static enum BehaviorStatus
   {
      INITIALIZED, PAUSED, ABORTED, DONE, FINALIZED
   }

   protected final Ros2Node ros2Node;
   private final Map<MessageTopicPair<?>, IHMCROS2Publisher<?>> publishers = new HashMap<>();

   protected final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue<?>>> localListeningNetworkQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue<?>>>();

   protected final String behaviorName;

   /**
    * Every variable that can be a {@link YoVariable} should be a {@link YoVariable}, so they can be
    * visualized in SCS.
    */
   protected final YoVariableRegistry registry;

   protected final YoEnum<BehaviorStatus> yoBehaviorStatus;
   protected final YoBoolean hasBeenInitialized;
   protected final YoBoolean isPaused;
   protected final YoBoolean isAborted;
   protected final YoDouble percentCompleted;

   private final List<BehaviorService> behaviorsServices;
   private final IHMCROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;
   protected final String robotName;

   protected final MessageTopicNameGenerator controllerSubGenerator, controllerPubGenerator;
   protected final MessageTopicNameGenerator behaviorSubGenerator, behaviorPubGenerator;

   protected final MessageTopicNameGenerator footstepPlanningToolboxSubGenerator, footstepPlanningToolboxPubGenerator;
   protected final MessageTopicNameGenerator kinematicsToolboxSubGenerator, kinematicsToolboxPubGenerator;
   protected final MessageTopicNameGenerator kinematicsPlanningToolboxSubGenerator, kinematicsPlanningToolboxPubGenerator;

   public AbstractBehavior(String robotName, Ros2Node ros2Node)
   {
      this(robotName, null, ros2Node);
   }

   public AbstractBehavior(String robotName, String namePrefix, Ros2Node ros2Node)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      behaviorName = FormattingTools.addPrefixAndKeepCamelCaseForMiddleOfExpression(namePrefix, getClass().getSimpleName());
      registry = new YoVariableRegistry(behaviorName);

      yoBehaviorStatus = new YoEnum<BehaviorStatus>(namePrefix + "Status", registry, BehaviorStatus.class);
      hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);
      isPaused = new YoBoolean("isPaused" + behaviorName, registry);
      isAborted = new YoBoolean("isAborted" + behaviorName, registry);
      percentCompleted = new YoDouble("percentCompleted", registry);

      behaviorsServices = new ArrayList<>();

      footstepPlanningToolboxSubGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2TopicQualifier.INPUT);
      footstepPlanningToolboxPubGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2TopicQualifier.OUTPUT);
      kinematicsToolboxSubGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_TOOLBOX, ROS2TopicQualifier.INPUT);
      kinematicsToolboxPubGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_TOOLBOX, ROS2TopicQualifier.OUTPUT);
      kinematicsPlanningToolboxSubGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.INPUT);
      kinematicsPlanningToolboxPubGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.OUTPUT);

      controllerSubGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      behaviorSubGenerator = IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(robotName);
      behaviorPubGenerator = IHMCHumanoidBehaviorManager.getPublisherTopicNameGenerator(robotName);

      textToSpeechPublisher = createPublisher(TextToSpeechPacket.class, ROS2Tools.getDefaultTopicNameGenerator());
   }

   public <T> IHMCROS2Publisher<T> createPublisherForController(Class<T> messageType)
   {
      return createPublisher(messageType, controllerSubGenerator);
   }

   public <T> IHMCROS2Publisher<T> createBehaviorOutputPublisher(Class<T> messageType)
   {
      return createPublisher(messageType, behaviorPubGenerator);
   }

   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, MessageTopicNameGenerator topicNameGenerator)
   {
      return createPublisher(messageType, topicNameGenerator.generateTopicName(messageType));
   }

   @SuppressWarnings("unchecked")
   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, String topicName)
   {
      MessageTopicPair<T> key = new MessageTopicPair<>(messageType, topicName);
      IHMCROS2Publisher<T> publisher = (IHMCROS2Publisher<T>) publishers.get(key);

      if (publisher != null)
         return publisher;

      publisher = ROS2Tools.createPublisher(ros2Node, messageType, topicName);
      publishers.put(key, publisher);
      return publisher;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, controllerPubGenerator, consumer);
   }

   public <T> void createBehaviorInputSubscriber(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, behaviorSubGenerator, consumer);
   }

   public <T> void createBehaviorInputSubscriber(Class<T> messageType, String topicSuffix, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, IHMCHumanoidBehaviorManager.getBehaviorInputRosTopicPrefix(robotName) + topicSuffix, consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, MessageTopicNameGenerator topicNameGenerator, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, topicNameGenerator.generateTopicName(messageType), consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
   }

   public void addBehaviorService(BehaviorService behaviorService)
   {
      behaviorsServices.add(behaviorService);
   }

   /**
    * Initialization method called when switching to this behavior.
    */
   @Override
   public final void initialize()
   {
      isPaused.set(false);
      isAborted.set(false);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.run();
      }

      onBehaviorEntered();
   }

   public abstract void onBehaviorEntered();

   /**
    * The implementation of this method should result in a clean shut down of the behavior.
    */
   public final void abort()
   {
      isAborted.set(true);
      isPaused.set(false);
      publishTextToSpeech("Aborting Behavior");

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.pause();
      }

      onBehaviorAborted();
   }

   public abstract void onBehaviorAborted();

   /**
    * The implementation of this method should result in pausing the behavior (pause current action
    * and no more actions sent to the controller, the robot remains still). The behavior should be
    * resumable.
    */
   public final void pause()
   {
      publishTextToSpeech("Pausing Behavior");
      isPaused.set(true);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.pause();
      }

      onBehaviorPaused();
   }

   public abstract void onBehaviorPaused();

   /**
    * The implementation of this method should result in resuming the behavior after being paused.
    * Should not do anything if the behavior has not been paused.
    */
   public final void resume()
   {
      publishTextToSpeech("Resuming Behavior");
      isPaused.set(false);
      isPaused.set(false);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.run();
      }

      onBehaviorResumed();
   }

   public void publishTextToSpeech(String textToSpeak)
   {
      textToSpeechPublisher.publish(MessageTools.createTextToSpeechPacket(textToSpeak));
   }

   public abstract void onBehaviorResumed();

   /**
    * Clean up method that is called when leaving the behavior for another one.
    */
   public final void doPostBehaviorCleanup()
   {
      isPaused.set(false);
      isAborted.set(false);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.pause();
      }

      onBehaviorExited();
   }

   public abstract void onBehaviorExited();

   /**
    * Only method to check if the behavior is done.
    * 
    * @return
    */
   public abstract boolean isDone();

   protected boolean isPaused()
   {
      return isPaused.getBooleanValue() || isAborted.getBooleanValue();
   }

   public BehaviorStatus getBehaviorStatus()
   {
      return yoBehaviorStatus.getEnumValue();
   }

   public CoactiveElement getCoactiveElement()
   {
      return null;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return behaviorName;
   }

   @Override
   public String getDescription()
   {
      return this.getClass().getCanonicalName();
   }

   public static class MessageTopicPair<T> extends Pair<Class<T>, String>
   {
      private static final long serialVersionUID = 7511864115932037512L;
      private final Class<T> messageType;
      private final String topicName;

      public MessageTopicPair(Class<T> messageType, String topicName)
      {
         this.messageType = messageType;
         this.topicName = topicName;
      }

      @Override
      public Class<T> getLeft()
      {
         return messageType;
      }

      @Override
      public String getRight()
      {
         return topicName;
      }

      @Override
      public String setValue(String value)
      {
         throw new UnsupportedOperationException();
      }
   }
}
