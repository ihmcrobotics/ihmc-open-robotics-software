package us.ihmc.simulationConstructionSetTools.simulationDispatcher.client;

import java.rmi.RemoteException;
import java.rmi.server.UnicastRemoteObject;

import us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces.RemoteIPRegistryInterface;

@SuppressWarnings("serial")
public class RemoteIPRegistryServer extends UnicastRemoteObject implements RemoteIPRegistryInterface
{
   private DispatchHostList hostList;
   private String pwd;

   public RemoteIPRegistryServer(DispatchHostList hostList, String pwd) throws RemoteException
   {
      this.pwd = pwd;
      this.hostList = hostList;
   }

   @Override
   public void registerMe(String hostName, String pwd) throws RemoteException
   {
      if (!pwd.equals(this.pwd))
      {
         System.out.println("Invalid Password! : " + pwd);

         throw new RemoteException("Invalid Password!");
      }

      hostList.registerMe(hostName);
   }

   @Override
   public String[] getAllRegistered(String pwd) throws RemoteException
   {
      if (!pwd.equals(this.pwd))
         throw new RemoteException("Invalid Password!");

      return hostList.getAllRegistered();
   }

}
