package us.ihmc.robotDataCommunication;

import java.nio.LongBuffer;
import java.util.List;

import us.ihmc.concurrent.dataStructures.ChangeListenerLongBuffer;
import us.ihmc.robotDataCommunication.jointState.JointHolder;
import us.ihmc.util.RealtimeTools;

public class FullStateBuffer extends ChangeListenerLongBuffer
{

   private long timestamp;
   private final double[] jointStates;
   private final List<JointHolder> jointHolders;
   private final int numberOfJointStates;
   
   public FullStateBuffer(int numberOfVariables, List<JointHolder> jointHolders)
   {
      super(numberOfVariables);
      this.numberOfJointStates = getNumberOfJointStates(jointHolders);
      this.jointStates = new double[RealtimeTools.nextDivisibleByEight(this.numberOfJointStates)];
      this.jointHolders = jointHolders;
   }
   
   public static int getNumberOfJointStates(List<JointHolder> jointHolders)
   {
      int numberOfJointStates = 0;
      for(int i = 0; i < jointHolders.size(); i++)
      {
        numberOfJointStates += jointHolders.get(i).getNumberOfStateVariables();
      }
      return numberOfJointStates;
   }
   
   public void updateJointStates(long timestamp)
   {
      this.timestamp = timestamp;
      
      int offset = 0;
      for(int i = 0; i < jointHolders.size(); i++)
      {
         JointHolder jointHolder = jointHolders.get(i);
         jointHolder.get(jointStates, offset);
         offset += jointHolder.getNumberOfStateVariables();
      }
   }
   
   public void getJointStatesInBuffer(LongBuffer buffer, int offset)
   {
      for(int i = 0; i < numberOfJointStates; i++)
      {
         buffer.put(i + offset, Double.doubleToLongBits(jointStates[i]));
      }
   }

   public static class Builder implements us.ihmc.concurrent.Builder<FullStateBuffer>
   {
      private final int numberOfVariables;
      private final List<JointHolder> jointHolders;
      
      public Builder(int numberOfVariables, List<JointHolder> jointHolders)
      {
         this.numberOfVariables = numberOfVariables;
         this.jointHolders = jointHolders;
      }

      public FullStateBuffer newInstance()
      {
         return new FullStateBuffer(numberOfVariables, jointHolders);
      }

      
   }

   public long getTimestamp()
   {
      return timestamp;
   }
}
