package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.utilities.net.ObjectCommunicator;

public abstract class HandControlThreadManager implements Runnable
{
	protected ObjectCommunicator objectCommunicator;
	
	public HandControlThreadManager(ObjectCommunicator objectCommunicator)
	{
		this.objectCommunicator = objectCommunicator;
		
		if (objectCommunicator != null)
			attachObjectConsumers();
	}
	
	public abstract void start();
	
	public abstract void connect();
	
	protected abstract void attachObjectConsumers();
}
