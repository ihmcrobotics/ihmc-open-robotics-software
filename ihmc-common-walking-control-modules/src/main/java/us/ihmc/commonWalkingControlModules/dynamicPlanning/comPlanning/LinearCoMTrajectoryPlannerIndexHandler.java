package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import gnu.trove.list.array.TIntArrayList;

import java.util.List;

public class LinearCoMTrajectoryPlannerIndexHandler
{
   private static final int contactSequenceSize = 4;
   private static final int flightSequenceSize = 2;

   private final List< ? extends ContactStateProvider> contactSequence;

   private int size;
   private final TIntArrayList startIndices = new TIntArrayList();

   public LinearCoMTrajectoryPlannerIndexHandler(List<? extends ContactStateProvider> contactSequence)
   {
      this.contactSequence = contactSequence;
   }

   public void update()
   {
      startIndices.clear();
      size = 0;
      startIndices.add(0);
      size += contactSequence.get(0).getContactState().isLoadBearing() ? contactSequenceSize : flightSequenceSize;
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         startIndices.add(size);
         size += contactSequence.get(sequenceId).getContactState().isLoadBearing() ? contactSequenceSize : flightSequenceSize;
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
