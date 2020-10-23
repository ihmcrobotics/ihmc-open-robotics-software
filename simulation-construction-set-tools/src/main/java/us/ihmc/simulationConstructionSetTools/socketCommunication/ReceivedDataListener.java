package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.util.List;

import us.ihmc.yoVariables.variable.YoVariable;


public interface ReceivedDataListener
{
   public void receivedData(List<YoVariable> sendVariables);
}
