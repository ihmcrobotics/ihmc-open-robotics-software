package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.UIPositionCheckerPacket;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.FootstepPlannerAPI;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.DeprecatedAPIs;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.BehaviorService;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
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

   protected final ROS2Node ros2Node;
   private final Map<ROS2Topic<?>, ROS2PublisherBasics<?>> publishers = new HashMap<>();

   protected final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue<?>>> localListeningNetworkQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue<?>>>();

   protected final String behaviorName;

   /**
    * Every variable that can be a {@link YoVariable} should be a {@link YoVariable}, so they can be
    * visualized in SCS.
    */
   protected final YoRegistry registry;

   protected final YoEnum<BehaviorStatus> yoBehaviorStatus;
   protected final YoBoolean hasBeenInitialized;
   protected final YoBoolean isPaused;
   protected final YoBoolean isAborted;
   protected final YoDouble percentCompleted;

   private final List<BehaviorService> behaviorsServices;
   private final ROS2PublisherBasics<TextToSpeechPacket> textToSpeechPublisher;
   private final ROS2PublisherBasics<UIPositionCheckerPacket> uiPositionCheckerPacketpublisher;

   protected final String robotName;

   protected final ROS2Topic controllerInputTopic, controllerOutputTopic;
   protected final ROS2Topic behaviorInputTopic, behaviorOutputTopic;

   protected final ROS2Topic footstepPlannerInputTopic, footstepPlannerOutputTopic;
   protected final ROS2Topic kinematicsToolboxInputTopic, kinematicsToolboxOutputTopic;
   protected final ROS2Topic kinematicsPlanningToolboxInputTopic, kinematicsPlanningToolboxOutputTopic;

   private static int behaviorUniqID = 0;
   
   public AbstractBehavior(String robotName, ROS2Node ros2Node)
   {
      this(robotName, null, ros2Node);
   }

   public AbstractBehavior(String robotName, String namePrefix, ROS2Node ros2Node)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      behaviorName = FormattingTools.addPrefixAndKeepCamelCaseForMiddleOfExpression(namePrefix, getClass().getSimpleName()+"-"+behaviorUniqID++);
      registry = new YoRegistry(behaviorName);

      yoBehaviorStatus = new YoEnum<BehaviorStatus>(namePrefix + "Status", registry, BehaviorStatus.class);
      hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);
      isPaused = new YoBoolean("isPaused" + behaviorName, registry);
      isAborted = new YoBoolean("isAborted" + behaviorName, registry);
      percentCompleted = new YoDouble("percentCompleted", registry);

      behaviorsServices = new ArrayList<>();

      footstepPlannerInputTopic = FootstepPlannerAPI.FOOTSTEP_PLANNER.withRobot(robotName).withInput();
      footstepPlannerOutputTopic = FootstepPlannerAPI.FOOTSTEP_PLANNER.withRobot(robotName).withOutput();
      kinematicsToolboxInputTopic = ToolboxAPIs.KINEMATICS_TOOLBOX.withRobot(robotName).withInput();
      kinematicsToolboxOutputTopic = ToolboxAPIs.KINEMATICS_TOOLBOX.withRobot(robotName).withOutput();
      kinematicsPlanningToolboxInputTopic = ToolboxAPIs.KINEMATICS_PLANNING_TOOLBOX.withRobot(robotName).withInput();
      kinematicsPlanningToolboxOutputTopic = ToolboxAPIs.KINEMATICS_PLANNING_TOOLBOX.withRobot(robotName).withOutput();

      controllerInputTopic = HumanoidControllerAPI.HUMANOID_CONTROLLER.withRobot(robotName).withInput();
      controllerOutputTopic = HumanoidControllerAPI.HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
      behaviorInputTopic = DeprecatedAPIs.BEHAVIOR_MODULE.withRobot(robotName).withInput();
      behaviorOutputTopic = DeprecatedAPIs.BEHAVIOR_MODULE.withRobot(robotName).withOutput();

      textToSpeechPublisher = createPublisher(TextToSpeechPacket.class, ROS2Tools.IHMC_ROOT);
      uiPositionCheckerPacketpublisher = createBehaviorOutputPublisher(UIPositionCheckerPacket.class);
   }

   public MessagerAPI getBehaviorAPI()
   {
      return null;
   }

   public <T> ROS2PublisherBasics<T> createPublisherForController(Class<T> messageType)
   {
      return createPublisher(messageType, controllerInputTopic);
   }

   public <T> ROS2PublisherBasics<T> createBehaviorOutputPublisher(Class<T> messageType)
   {
      return createPublisher(messageType, behaviorOutputTopic);
   }

   public <T> ROS2PublisherBasics<T> createBehaviorInputPublisher(Class<T> messageType)
   {
      return createPublisher(messageType, behaviorInputTopic);
   }

   @SuppressWarnings("unchecked")
   public <T> ROS2PublisherBasics<T> createPublisher(Class<T> messageType, ROS2Topic<?> topicName)
   {
      ROS2Topic<T> typedNamedTopic = topicName.withTypeName(messageType);
      ROS2PublisherBasics<T> publisher = (ROS2PublisherBasics<T>) publishers.get(typedNamedTopic);

      if (publisher == null) // !containsKey
      {
         publisher = ros2Node.createPublisher(messageType, typedNamedTopic.getName(), typedNamedTopic.getQoS());
         publishers.put(typedNamedTopic, publisher);
      }

      return publisher;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, controllerOutputTopic, consumer);
   }

   public <T> void createBehaviorInputSubscriber(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, behaviorInputTopic, consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2Topic topicName, ObjectConsumer<T> consumer)
   {
      ros2Node.createSubscription(((ROS2Topic<?>) topicName).withTypeName(messageType), s -> consumer.consumeObject(s.takeNextData()));
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
   public YoRegistry getYoRegistry()
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

