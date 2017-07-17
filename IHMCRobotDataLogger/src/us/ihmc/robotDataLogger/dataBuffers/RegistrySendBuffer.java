package us.ihmc.robotDataLogger.dataBuffers;

import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.List;

import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.yoVariables.variable.YoVariable;

public class RegistrySendBuffer extends RegistryBuffer
{

   private final ByteBuffer buffer;
   private final LongBuffer data;
   private final YoVariable<?>[] variables;

   private final List<JointHolder> jointHolders;

   protected RegistrySendBuffer(int registeryID, List<YoVariable<?>> variables, List<JointHolder> jointHolders)
   {
      this.buffer = ByteBuffer.allocate(variables.size() * 8);

      this.data = this.buffer.asLongBuffer();
      this.registryID = registeryID;

      this.variables = variables.toArray(new YoVariable[variables.size()]);

      this.jointHolders = jointHolders;
      this.jointStates = new double[RegistrySendBufferBuilder.getNumberOfJointStates(jointHolders)];

   }

   /**
    * Pack the internal buffer with data from the variables
    * 
    * @param timestamp
    * @param uid
    */
   public void updateBufferFromVariables(long timestamp, long uid)
   {
      this.uid = uid;
      this.timestamp = timestamp;
      this.transmitTime = System.nanoTime();
      data.clear();
      for (int i = 0; i < variables.length; i++)
      {
         data.put(variables[i].getValueAsLongBits());
      }

      int offset = 0;
      for (int i = 0; i < jointHolders.size(); i++)
      {
         JointHolder jointHolder = jointHolders.get(i);
         jointHolder.get(jointStates, offset);
         offset += jointHolder.getNumberOfStateVariables();
      }

   }

   public ByteBuffer getBuffer()
   {
      return buffer;
   }

}
