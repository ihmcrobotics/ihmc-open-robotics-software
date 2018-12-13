package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import gnu.trove.list.array.TIntArrayList;

import java.util.List;

public class CoMTrajectoryPlannerIndexHandler
{
   private static final int flightSequenceSize = 2;

   private final List< ? extends ContactStateProvider> contactSequence;

   private int size;
   private final TIntArrayList startIndices = new TIntArrayList();

   public CoMTrajectoryPlannerIndexHandler(List<? extends ContactStateProvider> contactSequence)
   {
      this.contactSequence = contactSequence;
   }

   public void update()
   {
      startIndices.clear();
      size = 0;
      startIndices.add(0);
      size += getSize(0);
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         startIndices.add(size);
         size += getSize(sequenceId);
      }
   }

   private int getSize(int sequenceId)
   {
      ContactStateProvider stateProvider = contactSequence.get(sequenceId);
      if (!stateProvider.getContactState().isLoadBearing())
         return flightSequenceSize;
      else
         return stateProvider.getContactMotion().getNumberOfCoefficients();
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
