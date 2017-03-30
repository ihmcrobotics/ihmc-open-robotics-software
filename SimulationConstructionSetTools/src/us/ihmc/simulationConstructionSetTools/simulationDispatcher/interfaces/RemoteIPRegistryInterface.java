package us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces;

import java.rmi.Remote;
import java.rmi.RemoteException;


public interface RemoteIPRegistryInterface extends Remote
{
   public abstract void registerMe(String hostname, String password) throws RemoteException;

   public abstract String[] getAllRegistered(String password) throws RemoteException;
}
