package us.ihmc.simulationconstructionset.robotcommprotocol;

import java.util.ArrayList;

import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;


public interface ReceivedDataListener
{
   public void receivedData(ArrayList<YoVariable> sendVariables);
}
