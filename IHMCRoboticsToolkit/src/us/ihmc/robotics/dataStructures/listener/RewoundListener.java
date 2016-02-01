package us.ihmc.robotics.dataStructures.listener;

/*
 * Listener on a SimulationConstructionSet simulation.
 * simulationRewound() will be called when the gui data index is changed by the user or outside call.
 * However, it will not be called when the simulation ticks forward naturally.
 */
public interface RewoundListener
{
	public void wasRewound();
}
