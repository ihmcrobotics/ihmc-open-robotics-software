package us.ihmc.avatar.testTools.scs2;

import static us.ihmc.robotics.Assert.fail;

import java.io.File;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javafx.application.Platform;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools.VideoAndDataExporter;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2AvatarTestingSimulation
{
   private final SCS2AvatarSimulation avatarSimulation;

   private ROS2Node ros2Node;
   @SuppressWarnings("rawtypes")
   private Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers;

   private final SimulationSessionControls simulationSessionControls;
   private final AtomicReference<Throwable> lastThrowable = new AtomicReference<>();

   private SessionVisualizerControls sessionVisualizerControls;

   private boolean createVideo = false;

   public SCS2AvatarTestingSimulation(SCS2AvatarSimulation avatarSimulation)
   {
      this.avatarSimulation = avatarSimulation;
      simulationSessionControls = avatarSimulation.getSimulationSession().getSimulationSessionControls();
      simulationSessionControls.addSimulationThrowableListener(lastThrowable::set);
   }

   public void setCreateVideo(boolean createVideo)
   {
      this.createVideo = createVideo;
   }

   public void start()
   {
      if (Platform.isImplicitExit())
         Platform.setImplicitExit(false);

      avatarSimulation.start();
      sessionVisualizerControls = avatarSimulation.getSessionVisualizerControls();
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.waitUntilFullyUp();
   }

   // Simulation controls:

   public boolean simulateAndWait(double duration)
   {
      lastThrowable.set(null);
      return simulationSessionControls.simulateAndWait(duration);
   }

   public void assertRobotsRootJointIsInBoundingBox(BoundingBox3DReadOnly boundingBox)
   {
      Robot robot = avatarSimulation.getSimulationSession().getPhysicsEngine().getRobots().get(0);
      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getRootBody().getChildrenJoints().get(0);
      boolean inside = boundingBox.isInsideInclusive(rootJoint.getJointPose().getPosition());
      if (!inside)
      {
         fail("Joint was at " + rootJoint.getJointPose().getPosition() + ". Expecting it to be inside boundingBox " + boundingBox);
      }
   }

   // Buffer controls:
   public void setBufferInPointIndexToCurrent()
   {
      simulationSessionControls.setBufferInPointIndexToCurrent();
   }

   public void setBufferOutPointIndexToCurrent()
   {
      simulationSessionControls.setBufferOutPointIndexToCurrent();
   }

   // GUI controls:
   public void setCameraFocusPosition(double x, double y, double z)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.setCameraFocusPosition(x, y, z);
   }

   public void setCameraPosition(double x, double y, double z)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.setCameraPosition(x, y, z);
   }

   public void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.requestCameraRigidBodyTracking(robotName, rigidBodyName);
   }

   // Misc.
   public void finishTest()
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.waitUntilDown();
      else
         destroy();
   }

   public void destroy()
   {
      avatarSimulation.destroy();
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public void publishToController(Object message)
   {
      IHMCROS2Publisher ihmcros2Publisher = defaultControllerPublishers.get(message.getClass());
      ihmcros2Publisher.publish(message);
   }

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }

   public void setROS2Node(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   @SuppressWarnings("rawtypes")
   public void setDefaultControllerPublishers(Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers)
   {
      this.defaultControllerPublishers = defaultControllerPublishers;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, ROS2Tools.getControllerOutputTopic(avatarSimulation.getRobotModel().getSimpleRobotName()), consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2Topic<?> generator, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, messageType, generator, s -> consumer.consumeObject(s.takeNextData()));
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
   }

   public YoRegistry getEstimatorRegistry()
   {
      return avatarSimulation.getEstimatorThread().getYoRegistry();
   }

   public YoRegistry getControllerRegistry()
   {
      return avatarSimulation.getControllerThread().getYoVariableRegistry();
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return avatarSimulation.getControllerFullRobotModel();
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return avatarSimulation.getHighLevelHumanoidControllerFactory();
   }

   /**
    * For unit testing only
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      avatarSimulation.addRobotControllerOnControllerThread(controller);
   }

   public SimulationSession getSimulationSession()
   {
      return avatarSimulation.getSimulationSession();
   }

   public void createVideo(String simplifiedRobotModelName, int callStackHeight)
   {
      if (createVideo)
      {
         BambooTools.createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(simplifiedRobotModelName,
                                                                                        createBambooToolsVideoAndDataExporter(),
                                                                                        callStackHeight,
                                                                                        avatarSimulation.getShowGUI());
      }
      else
      {
         LogTools.info("Skipping video generation.");
      }
   }

   public void createVideo(String videoName)
   {
      if (createVideo)
      {
         BambooTools.createVideoWithDateTimeAndStoreInDefaultDirectory(createBambooToolsVideoAndDataExporter(), videoName, avatarSimulation.getShowGUI());
      }
      else
      {
         LogTools.info("Skipping video generation.");
      }
   }

   private VideoAndDataExporter createBambooToolsVideoAndDataExporter()
   {
      return new VideoAndDataExporter()
      {

         @Override
         public void writeData(File dataFile)
         {
            // TODO Implement me
         }

         @Override
         public void gotoOutPointNow()
         {
            simulationSessionControls.setBufferCurrentIndexToOutPoint();
         }

         @Override
         public File createVideo(String string)
         {
            File videoFile = new File(string);
            sessionVisualizerControls.exportVideo(videoFile);
            return videoFile;
         }
      };
   }
}
