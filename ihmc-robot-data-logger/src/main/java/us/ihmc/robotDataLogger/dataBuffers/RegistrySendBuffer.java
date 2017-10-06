package us.ihmc.robotDataLogger.dataBuffers;

import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.List;

import us.ihmc.robotDataLogger.LogDataType;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.rtps.LogParticipantTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class RegistrySendBuffer extends RegistryBuffer
{

   private final ByteBuffer buffer;
   private final LongBuffer data;
   private final YoVariable<?>[] variables;

   private final List<JointHolder> jointHolders;

   private double[] jointStates;
   private final double[] allocatedJointStates;
   private final double[] emptyJointStates = new double[0];
   
   /**
    * Keep alive variant of buffer
    * 
    */
   protected RegistrySendBuffer()
   {
      this.buffer = null;
      this.data = null;
      this.variables = null;
      this.jointHolders = null;
      this.allocatedJointStates = null;
      
      setTimestamp(-1);
      setRegistryID(-1);
      setOffset(-1);
      setNumberOfVariables(0);
      setType(LogDataType.KEEP_ALIVE_PACKET);
   }
   
   protected RegistrySendBuffer(int registeryID, List<YoVariable<?>> variables, List<JointHolder> jointHolders)
   {
      int numberOfJointStates = RegistrySendBufferBuilder.getNumberOfJointStates(jointHolders);
      int maximumNumberOfVariables = LogParticipantTools.calculateMaximumNumberOfVariables(variables.size(), numberOfJointStates);
      this.buffer = ByteBuffer.allocate(maximumNumberOfVariables * 8);

      this.data = this.buffer.asLongBuffer();
      this.registryID = registeryID;

      this.variables = variables.toArray(new YoVariable[variables.size()]);

      this.jointHolders = jointHolders;
      this.allocatedJointStates = new double[numberOfJointStates];
      
   }

   /**
    * Pack the internal buffer with data from the variables
    * 
    * @param timestamp
    * @param uid
    */
   public void updateBufferFromVariables(long timestamp, long uid, int segment, int offset, int numberOfVariables)
   {
      this.uid = uid;
      this.timestamp = timestamp;
      this.transmitTime = System.nanoTime();
      this.offset = offset;
      this.numberOfVariables = numberOfVariables;
      this.data.clear();
      for (int i = offset; i < offset + numberOfVariables; i++)
      {
         this.data.put(variables[i].getValueAsLongBits());
      }
      this.data.flip();
      this.buffer.clear();
      this.buffer.limit(this.data.limit() * 8);

      if(segment == 0)
      {
         int jointOffset = 0;
         for (int i = 0; i < jointHolders.size(); i++)
         {
            JointHolder jointHolder = jointHolders.get(i);
            jointHolder.get(allocatedJointStates, jointOffset);
            jointOffset += jointHolder.getNumberOfStateVariables();
         }
         
         jointStates = allocatedJointStates;
      }
      else
      {
         jointStates = emptyJointStates;
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
