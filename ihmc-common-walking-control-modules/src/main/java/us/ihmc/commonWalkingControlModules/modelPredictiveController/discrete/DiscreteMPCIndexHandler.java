package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import gnu.trove.list.array.TIntArrayList;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCIndexHandler;

import java.util.List;
import java.util.function.IntUnaryOperator;

public class DiscreteMPCIndexHandler extends LinearMPCIndexHandler
{
   public static final double orientationDt = 1e-2;
   public static final int orientationVariablesPerTick = 6;

   private final TIntArrayList orientationTicksInSegment = new TIntArrayList();
   private final TIntArrayList orientationStartSegment = new TIntArrayList();
   private int totalOrientationTicks = -1;

   public DiscreteMPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      super(numberOfBasisVectorsPerContactPoint);
   }

   @Override
   public void initialize(List<ContactPlaneProvider> contactSequence)
   {
      throw new NotImplementedException();
   }

   public void initialize(List<ContactPlaneProvider> contactSequence, double orientationWindowDuration)
   {
      listToSizeReturn.setContacts(contactSequence);
      initialize(listToSizeReturn, contactSequence.size());

      orientationTicksInSegment.clear();
      orientationStartSegment.clear();

      totalOrientationTicks = (int) Math.floor(orientationWindowDuration / orientationDt);
      int ticksRemaining = totalOrientationTicks;

      for (int segmentId = 0; segmentId < contactSequence.size(); segmentId++)
      {
         double duration = contactSequence.get(0).getTimeInterval().getDuration();
         int ticksInSegment = (int) Math.floor(duration / orientationDt);
         ticksInSegment = Math.min(ticksInSegment, ticksRemaining);
         orientationTicksInSegment.add(ticksInSegment);

         ticksRemaining -= ticksInSegment;

         orientationStartSegment.add(totalProblemSize);
         totalProblemSize += ticksInSegment * orientationVariablesPerTick;
      }
   }

   public int getOrientationTicksInSegment(int segmentId)
   {
      return orientationTicksInSegment.get(segmentId);
   }

   public int getOrientationStart(int segmentId)
   {
      return orientationStartSegment.get(segmentId);
   }

   public int getTotalOrientationTicks()
   {
      return totalOrientationTicks;
   }
}
