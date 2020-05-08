package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2TopicName;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.BehaviorService;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
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

   KryoMessager messager;

   protected final Ros2Node ros2Node;
   private final Map<ROS2TopicName, IHMCROS2Publisher<?>> publishers = new HashMap<>();

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
   private final IHMCROS2Publisher<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;

   protected final String robotName;

   protected final ROS2TopicName controllerInputTopicName, controllerOutputTopicName;
   protected final ROS2TopicName behaviorInputTopicName, behaviorOutputTopicName;

   protected final ROS2TopicName footstepPlannerInputTopicName, footstepPlannerOutputTopicName;
   protected final ROS2TopicName kinematicsToolboxInputTopicName, kinematicsToolboxOutputTopicName;
   protected final ROS2TopicName kinematicsPlanningToolboxInputTopicName, kinematicsPlanningToolboxOutputTopicName;

   private static int behaviorUniqID = 0;
   
   public AbstractBehavior(String robotName, Ros2Node ros2Node)
   {
      this(robotName, null, ros2Node);
   }

   public AbstractBehavior(String robotName, String namePrefix, Ros2Node ros2Node)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      behaviorName = FormattingTools.addPrefixAndKeepCamelCaseForMiddleOfExpression(namePrefix, getClass().getSimpleName()+"-"+behaviorUniqID++);
      registry = new YoVariableRegistry(behaviorName);

      yoBehaviorStatus = new YoEnum<BehaviorStatus>(namePrefix + "Status", registry, BehaviorStatus.class);
      hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);
      isPaused = new YoBoolean("isPaused" + behaviorName, registry);
      isAborted = new YoBoolean("isAborted" + behaviorName, registry);
      percentCompleted = new YoDouble("percentCompleted", registry);

      behaviorsServices = new ArrayList<>();

      footstepPlannerInputTopicName = ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withInput();
      footstepPlannerOutputTopicName = ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
      kinematicsToolboxInputTopicName = ROS2Tools.KINEMATICS_TOOLBOX.withRobot(robotName).withInput();
      kinematicsToolboxOutputTopicName = ROS2Tools.KINEMATICS_TOOLBOX.withRobot(robotName).withOutput();
      kinematicsPlanningToolboxInputTopicName = ROS2Tools.KINEMATICS_PLANNING_TOOLBOX.withRobot(robotName).withInput();
      kinematicsPlanningToolboxOutputTopicName = ROS2Tools.KINEMATICS_PLANNING_TOOLBOX.withRobot(robotName).withOutput();

      controllerInputTopicName = ROS2Tools.HUMANOID_CONTROLLER.withRobot(robotName).withInput();
      controllerOutputTopicName = ROS2Tools.HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
      behaviorInputTopicName = ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withInput();
      behaviorOutputTopicName = ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withOutput();

      textToSpeechPublisher = createPublisher(TextToSpeechPacket.class, ROS2Tools.IHMC_ROOT);
      uiPositionCheckerPacketpublisher = createBehaviorPublisher(UIPositionCheckerPacket.class);
   }

   public MessagerAPI getBehaviorAPI()
   {
      return null;
   }

   public <T> IHMCROS2Publisher<T> createPublisherForController(Class<T> messageType)
   {
      return createPublisher(messageType, controllerInputTopicName);
   }

   public <T> IHMCROS2Publisher<T> createBehaviorPublisher(Class<T> messageType)
   {
      return createPublisher(messageType, behaviorOutputTopicName);
   }

   @SuppressWarnings("unchecked")
   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, ROS2TopicName topicName)
   {
      IHMCROS2Publisher<T> publisher = (IHMCROS2Publisher<T>) publishers.get(topicName);

      if (publisher == null) // !containsKey
      {
         publisher = ROS2Tools.createPublisherTypeNamed(ros2Node, messageType, topicName);
         publishers.put(topicName, publisher);
      }

      return publisher;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, controllerOutputTopicName, consumer);
   }

   public <T> void createBehaviorInputSubscriber(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, behaviorInputTopicName, consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2TopicName topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
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
         behaviorService.destroy();
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

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.run();
      }

      onBehaviorResumed();
   }

   public void publishTextToSpeech(String textToSpeak)
   {
      LogTools.info(1, "TextToSpeech: " + textToSpeak);
      textToSpeechPublisher.publish(MessageTools.createTextToSpeechPacket(textToSpeak));
   }

   public void publishUIPositionCheckerPacket(Point3DReadOnly position)
   {
      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(position));

   }

   public void publishUIPositionCheckerPacket(Point3DReadOnly position, Quaternion orientation)
   {
      uiPositionCheckerPacketpublisher.publish(MessageTools.createUIPositionCheckerPacket(position, orientation));

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
}

