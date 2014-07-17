package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.utilities.net.KryoObjectClient;

public abstract class HandControlThreadManager
{
	protected KryoObjectClient objectCommunicator;
	
	public HandControlThreadManager(int tcpPort)
	{
		this.objectCommunicator = new KryoObjectClient("localhost", tcpPort, new DRCNetClassList());
	}
	
   public abstract void start();
	
   public abstract void connect();
}
