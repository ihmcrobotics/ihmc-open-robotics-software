package us.ihmc.avatar.testTools.scs2;

import static us.ihmc.robotics.Assert.fail;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javafx.application.Platform;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;

public class SCS2AvatarTestingSimulation
{
   private final SCS2AvatarSimulation avatarSimulation;

   private ROS2Node ros2Node;
   @SuppressWarnings("rawtypes")
   private Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers;

   private final SimulationSessionControls simulationSessionControls;
   private final AtomicReference<Throwable> lastThrowable = new AtomicReference<>();

   private SessionVisualizerControls sessionVisualizerControls;

   public SCS2AvatarTestingSimulation(SCS2AvatarSimulation avatarSimulation)
   {
      this.avatarSimulation = avatarSimulation;
      simulationSessionControls = avatarSimulation.getSimulationSession().getSimulationSessionControls();
      simulationSessionControls.addSimulationThrowableListener(lastThrowable::set);
   }

   public void start()
   {
      if (Platform.isImplicitExit())
         Platform.setImplicitExit(false);

      avatarSimulation.start();
      sessionVisualizerControls = avatarSimulation.getSessionVisualizerControls();
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
   public void setBufferInPointToCurrent()
   {
      simulationSessionControls.setBufferInPointToCurrent();
   }

   public void setBufferOutPointToCurrent()
   {
      simulationSessionControls.setBufferOutPointToCurrent();
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

   @SuppressWarnings("unchecked")
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

   public void setDefaultControllerPublishers(Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers)
   {
      this.defaultControllerPublishers = defaultControllerPublishers;
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return avatarSimulation.getControllerFullRobotModel();
   }

   /**
    * For unit testing only
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      avatarSimulation.addRobotControllerOnControllerThread(controller);
   }

   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return avatarSimulation.getFullRobotModelCorruptor();
   }

   public SimulationSession getSimulationSession()
   {
      return avatarSimulation.getSimulationSession();
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return avatarSimulation.getHighLevelHumanoidControllerFactory();
   }

   public SimulatedDRCRobotTimeProvider getSimulatedRobotTimeProvider()
   {
      return avatarSimulation.getSimulatedRobotTimeProvider();
   }
}
