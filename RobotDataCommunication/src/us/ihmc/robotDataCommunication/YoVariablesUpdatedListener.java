package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;
import java.util.List;

import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public interface YoVariablesUpdatedListener
{
   public boolean populateRegistry();

   public boolean changesVariables();

   public void setRegistry(YoVariableRegistry registry);

   public void registerDynamicGraphicObjectListsRegistry(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, boolean showOverheadView);

   public void receivedHandshake(YoProtoHandshake handshake);

   public void receivedUpdate(long timestamp, ByteBuffer buf);

   public void start();

   public void disconnected();

   public void setJointStates(List<JointState<? extends Joint>> jointStates);

   public void setYoVariableClient(YoVariableClient client);

   public void receiveTimedOut(long timeoutInMillis);

   public long getDisplayOneInNPackets();

}
