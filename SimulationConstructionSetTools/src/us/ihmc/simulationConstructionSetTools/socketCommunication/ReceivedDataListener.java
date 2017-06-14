package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.util.ArrayList;

import us.ihmc.yoVariables.variable.YoVariable;


public interface ReceivedDataListener
{
   public void receivedData(ArrayList<YoVariable<?>> sendVariables);
}
