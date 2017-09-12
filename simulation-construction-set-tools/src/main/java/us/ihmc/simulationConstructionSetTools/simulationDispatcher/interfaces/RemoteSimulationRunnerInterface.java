package us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces;

import java.rmi.Remote;
import java.rmi.RemoteException;


public interface RemoteSimulationRunnerInterface extends Remote
{
   public abstract int add(int a, int b) throws RemoteException;

   public abstract void createSimulation(RemoteSimulationDescription description, String[] structuralParameterNames, double[] structuralParameterValues, String password) throws RemoteException;

   public abstract void destroySimulation(String password) throws RemoteException;

   public abstract boolean ping(String password) throws RemoteException;

   public abstract void setSimulationState(Object state, String password) throws RemoteException;

   public abstract void startSimulation(String password) throws RemoteException;

   public abstract boolean isSimulationDone(String password) throws RemoteException;

   public abstract Object getSimulationState(String password) throws RemoteException;

   public abstract Object getSimulationData(String password) throws RemoteException;
}
