package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import gnu.trove.list.array.TIntArrayList;

import java.util.List;

public class ThirdOrderCoMTrajectoryPlannerIndexHandler
{
   private static final int sequenceSize = 6;

   private final List< ? extends ContactStateProvider> contactSequence;

   private int size;
   private final TIntArrayList startIndices = new TIntArrayList();

   public ThirdOrderCoMTrajectoryPlannerIndexHandler(List<? extends ContactStateProvider> contactSequence)
   {
      this.contactSequence = contactSequence;
   }

   public void update()
   {
      startIndices.clear();
      size = 0;
      startIndices.add(0);
      size += sequenceSize;
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         startIndices.add(size);
         size += sequenceSize;
      }
   }

   public int getTotalSize()
   {
      return size;
   }

   public int getContactSequenceStartIndex(int sequenceNumber)
   {
      return startIndices.get(sequenceNumber);
   }

}
