package us.ihmc.avatar.simulationStarter;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;

public interface SimulationStarterInterface
{
   /**
    * Set a specific starting location. By default, the robot will start at (0, 0) in world with no yaw.
    */
   public abstract void setStartingLocation(DRCStartingLocation startingLocation);

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

   public abstract void close();
}
