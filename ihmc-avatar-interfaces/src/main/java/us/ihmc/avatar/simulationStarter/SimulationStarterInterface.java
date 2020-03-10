package us.ihmc.avatar.simulationStarter;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;

public interface SimulationStarterInterface
{
   /**
    * Set a specific starting location. By default, the robot will start at (0, 0) in world with no
    * yaw.
    */
   public abstract void setStartingLocation(DRCStartingLocation startingLocation);

   /**
    * Starts the SCS visualizer for the behavior module.
    */
   public abstract void startBehaviorVisualizer();

   default void startSimulation(boolean automaticallyStartSimulation)
   {
      startSimulation(null, automaticallyStartSimulation);
   }

   /**
    * Creates and starts the simulation and automatically starts the network processor if required. All
    * the specific requirements (environment, robot initial setup, etc.) have to be set before calling
    * this method.
    * 
    * @param networkParameters            the network processor is created and started. Can be
    *                                     {@code null} to not create the network processor.
    * @param automaticallyStartSimulation if true SCS will be simulating when it shows up.
    * @return
    */
   public abstract void startSimulation(HumanoidNetworkProcessorParameters networkParameters, boolean automaticallyStartSimulation);

   public abstract void close();
}
