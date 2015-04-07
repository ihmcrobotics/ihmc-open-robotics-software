package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;
import java.util.List;

import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public interface YoVariablesUpdatedListener
{
   public boolean populateRegistry();

   public boolean changesVariables();

   public void receivedHandshake(LogHandshake handshake);

   public void receivedUpdate(long timestamp, ByteBuffer buf);

   public void start(LogModelLoader logModelLoader, YoVariableRegistry yoVariableRegistry, List<JointState<? extends Joint>> list, YoGraphicsListRegistry yoGraphicsListRegistry, int bufferSize, boolean showOverheadView);

   public void disconnected();

   public void setYoVariableClient(YoVariableClient client);

   public void receiveTimedOut();

   public int getDisplayOneInNPackets();

   public void timestampReceived(long timestamp);

   public void clearLog();

}
