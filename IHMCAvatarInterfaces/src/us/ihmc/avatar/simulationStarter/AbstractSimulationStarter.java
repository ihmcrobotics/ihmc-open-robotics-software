package us.ihmc.avatar.simulationStarter;

import javax.vecmath.Vector3d;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelBehaviorFactory;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;

public interface AbstractSimulationStarter
{
   public abstract CommonAvatarEnvironmentInterface getEnvironment();

   /**
    * Deativate the controller failure detector.
    * If called the walking controller will running even if the robot falls.
    */
   public abstract void deactivateWalkingFallDetector();

   /**
    * Register a controller to be created in addition to the walking controller.
    * For instance, the {@link CarIngressEgressController} can be created by passing its factory, i.e. {@link CarIngressEgressControllerFactory}.
    * The active controller can then be switched by either changing the variable {@code requestedHighLevelState} from SCS or by sending a {@link HighLevelStatePacket} to the controller.
    * @param controllerFactory a factory to create an additional controller.
    */
   public abstract void registerHighLevelController(HighLevelBehaviorFactory controllerFactory);

   /**
    * Call this method to disable simulated sensors such as LIDAR and camera.
    */
   public abstract void disableSCSSimulatedSensors();

   public abstract void attachControllerFailureListener(ControllerFailureListener listener);

   /**
    * Sets whether the estimator and the controller are running on the same thread or multiThreaded. Defaults to multiThreaded.
    * Need to set to false if you want the simulation to be rewindable.
    * @param runMultiThreaded
    */
   public abstract void setRunMultiThreaded(boolean runMultiThreaded);

   /**
    * Set whether state estimation as on the the real robot or perfect sensors coming from the simulated robot should be used.
    * By default the state estimator is used. 
    * @param usePerfectSensors
    */
   public abstract void setUsePerfectSensors(boolean usePerfectSensors);

   /**
    * Indicates if the state estimator should be aware of the robot starting location.
    * It is set to false by default, meaning that independently from the robot starting location, the state estimator will think that the robot started at (0, 0) in world but will be aware of the initial yaw.
    */
   public abstract void setInitializeEstimatorToActual(boolean initializeEstimatorToActual);

   /**
    * Provide a subscriber for receiving pelvis poses (for instance from the iterative closest point module) to be accounted for in the state estimator.
    * @param externalPelvisCorrectorSubscriber
    */
   public abstract void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber);

   /**
    * Set a GUI initial setup. If not called, a default GUI initial setup is used.
    */
   public abstract void setGuiInitialSetup(DRCGuiInitialSetup guiInitialSetup);

   /**
    * Set a robot initial setup to use instead of the one in DRCRobotModel.
    * @param robotInitialSetup
    */
   public abstract void setRobotInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup);

   /**
    * Set a specific starting location. By default, the robot will start at (0, 0) in world with no yaw.
    */
   public abstract void setStartingLocation(DRCStartingLocation startingLocation);

   /**
    * Set a specific starting location offset. By default, the robot will start at (0, 0) in world with no yaw.
    */
   public abstract void setStartingLocationOffset(OffsetAndYawRobotInitialSetup startingLocationOffset);

   /**
    * Set a specific starting location offset. By default, the robot will start at (0, 0) in world with no yaw.
    */
   public abstract void setStartingLocationOffset(Vector3d robotInitialPosition, double yaw);

   /**
    * Sets the initial SCS camera position.
    * @param positionX
    * @param positionY
    * @param positionZ
    */
   public abstract void setSCSCameraPosition(double positionX, double positionY, double positionZ);

   /**
    * Sets the initial fix point that the SCS camera looks at.
    * @param fixX
    * @param fixY
    * @param fixZ
    */
   public abstract void setSCSCameraFix(double fixX, double fixY, double fixZ);

//   /** Make the controller use a specific PacketCommunicator instead of the default. If you don't know what you're doing, forget about that method. */
//   public void setControllerPacketCommunicator(PacketCommunicator controllerInputPacketCommunicator)
//   {
//      this.controllerPacketCommunicator = controllerInputPacketCommunicator;
//   }

   /**
    * Starts the SCS visualizer for the behavior module.
    */
   public abstract void startBehaviorVisualizer();

   /**
    * Creates and starts the simulation and automatically starts the network processor if required.
    * All the specific requirements (environment, robot initial setup, etc.) have to be set before calling this method.
    * @param startNetworkProcessor if true the network processor is created and started.
    * @param automaticallyStartSimulation if true SCS will be simulating when it shows up.
    * @return
    */
   public abstract void startSimulation(DRCNetworkModuleParameters networkParameters, boolean automaticallyStartSimulation);

   public abstract LocalObjectCommunicator createSimulatedSensorsPacketCommunicator();

   public abstract AvatarSimulation getAvatarSimulation();

   public abstract SimulationConstructionSet getSimulationConstructionSet();

   public abstract HumanoidFloatingRootJointRobot getSDFRobot();

   public abstract PacketRouter<PacketDestination> getPacketRouter();

   public abstract LocalObjectCommunicator getSimulatedSensorsPacketCommunicator();

   public abstract void close();
}
