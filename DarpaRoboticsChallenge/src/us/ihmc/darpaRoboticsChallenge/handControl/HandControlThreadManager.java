package us.ihmc.darpaRoboticsChallenge.handControl;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.KryoObjectClient;

public abstract class HandControlThreadManager
{
	protected KryoObjectClient objectCommunicator;
	
	public HandControlThreadManager(int tcpPort)
	{
		this.objectCommunicator = new KryoObjectClient("localhost", tcpPort, new IHMCCommunicationKryoNetClassList());
	}
	
   public abstract void start();
	
   public abstract void connect();
}
