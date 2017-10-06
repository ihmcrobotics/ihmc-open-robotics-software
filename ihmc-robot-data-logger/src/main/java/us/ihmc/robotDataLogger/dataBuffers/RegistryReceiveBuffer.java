package us.ihmc.robotDataLogger.dataBuffers;

import java.nio.ByteBuffer;
import java.util.Arrays;

public class RegistryReceiveBuffer extends RegistryBuffer
{
   
   private final long receivedTimestamp;
   private ByteBuffer compressedVariableDataBuffer;
   private double[] jointStates;

   
   public RegistryReceiveBuffer(long receivedTimestamp)
   {
      this.receivedTimestamp = receivedTimestamp;  
   }
   
   public long getReceivedTimestamp()
   {
      return receivedTimestamp;
   }
   
   public ByteBuffer allocateBuffer(int size)
   {
      this.compressedVariableDataBuffer = ByteBuffer.allocate(size);
      return compressedVariableDataBuffer;
   }

   public double[] allocateStates(int stateLength)
   {
      this.jointStates = new double[stateLength];
      return this.jointStates;
   }

   public double[] getJointStates()
   {
      return jointStates;
   }

   public ByteBuffer getData()
   {
      return compressedVariableDataBuffer;
   }

   @Override
   public String toString()
   {
      return "RegistryReceiveBuffer [receivedTimestamp=" + receivedTimestamp + ", compressedVariableDataBuffer=" + compressedVariableDataBuffer + ", registryID=" + registryID + ", jointStates=" + Arrays.toString(jointStates) + ", timestamp="
            + timestamp + ", uid=" + uid + "]";
   }
   
   
}
