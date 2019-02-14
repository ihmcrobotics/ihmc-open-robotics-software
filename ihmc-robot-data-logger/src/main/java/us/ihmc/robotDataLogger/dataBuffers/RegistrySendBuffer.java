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
   private final JointHolder[] jointHolders;
   private final double[] jointStates;
   
   

   
   
   protected RegistrySendBuffer(int registeryID, List<YoVariable<?>> variables, List<JointHolder> jointHolders)
   {
      int numberOfJointStates = RegistrySendBufferBuilder.getNumberOfJointStates(jointHolders);
      int maximumNumberOfVariables = variables.size() + numberOfJointStates;//LogParticipantTools.calculateMaximumNumberOfVariables(variables.size(), numberOfJointStates);
      this.buffer = ByteBuffer.allocate(maximumNumberOfVariables * 8);
      
      this.data = this.buffer.asLongBuffer();
      this.registryID = registeryID;

      this.variables = variables.toArray(new YoVariable[variables.size()]);
      this.jointHolders = jointHolders.toArray(new JointHolder[jointHolders.size()]);

      this.jointStates = new double[numberOfJointStates];
      
   }

   /**
    * Pack the internal buffer with data from the variables
    * 
    * @param timestamp
    * @param uid
    */
   public void updateBufferFromVariables(long timestamp, long uid, int numberOfVariables)
   {
      this.uid = uid;
      this.timestamp = timestamp;
      this.transmitTime = System.nanoTime();
      this.numberOfVariables = numberOfVariables;
      this.data.clear();
      for (int i = 0; i < numberOfVariables; i++)
      {
         this.data.put(variables[i].getValueAsLongBits());
      }
      this.data.flip();
      this.buffer.clear();
      this.buffer.limit(this.data.limit() * 8);
      int jointOffset = 0;
      for (JointHolder jointHolder : jointHolders)
      {
         jointHolder.get(jointStates, jointOffset);
         jointOffset += jointHolder.getNumberOfStateVariables();
      }
   }


   public double[] getJointStates()
   {
      return jointStates;
   }
   
   public ByteBuffer getBuffer()
   {
      return buffer;
   }

}
