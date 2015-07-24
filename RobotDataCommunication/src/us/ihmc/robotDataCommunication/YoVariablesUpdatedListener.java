package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;
import java.util.List;

import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public interface YoVariablesUpdatedListener
{
   boolean populateRegistry();

   boolean changesVariables();

   void receivedHandshake(LogHandshake handshake);

   void receivedUpdate(long timestamp, ByteBuffer buf);

   void setShowOverheadView(boolean showOverheadView);

   void start(YoVariableRegistry yoVariableRegistry, List<JointState<? extends Joint>> list, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableHandshakeParser parser);

   void disconnected();

   void setYoVariableClient(YoVariableClient client);

   void receiveTimedOut();

   int getDisplayOneInNPackets();

   void timestampReceived(long timestamp);

   void clearLog();

   boolean executeVariableChangedListeners();
}
