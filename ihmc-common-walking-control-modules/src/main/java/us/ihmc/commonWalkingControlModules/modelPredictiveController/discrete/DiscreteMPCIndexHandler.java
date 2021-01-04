package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import gnu.trove.list.array.TIntArrayList;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCIndexHandler;

import java.util.List;
import java.util.function.IntUnaryOperator;

public class DiscreteMPCIndexHandler extends LinearMPCIndexHandler
{
   private static final double defaultOrientationDt = 5e-2;
   public static final int orientationVariablesPerTick = 6;

   private final double orientationDt;
   private final TIntArrayList orientationTicksInSegment = new TIntArrayList();
   private final TIntArrayList orientationTicksBeforeSegment = new TIntArrayList();
   private final TIntArrayList orientationStartSegment = new TIntArrayList();
   private int totalOrientationTicks = -1;

   public DiscreteMPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      this(numberOfBasisVectorsPerContactPoint, defaultOrientationDt);
   }

   public DiscreteMPCIndexHandler(int numberOfBasisVectorsPerContactPoint, double orientationDt)
   {
      super(numberOfBasisVectorsPerContactPoint);

      this.orientationDt = orientationDt;
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

      orientationTicksInSegment.reset();
      orientationStartSegment.reset();
      orientationTicksBeforeSegment.reset();

      double maxWindowDuration = 0.0;
      for (int segmentId = 0; segmentId < contactSequence.size(); segmentId++)
         maxWindowDuration += contactSequence.get(segmentId).getTimeInterval().getDuration();
      maxWindowDuration = Math.min(maxWindowDuration, orientationWindowDuration);

      totalOrientationTicks = (int) Math.floor(maxWindowDuration / orientationDt + 1e-5);
      int ticksRemaining = totalOrientationTicks;
      int ticksBefore = 0;

      double residualTime = 0.0;

      int segmentId = 0;
      for (; segmentId < contactSequence.size(); segmentId++)
      {
         double duration = contactSequence.get(segmentId).getTimeInterval().getDuration() + residualTime;
         int ticksInSegment = (int) Math.floor((duration + 1e-5) / orientationDt);
         ticksInSegment = Math.min(ticksInSegment, ticksRemaining);
         orientationTicksInSegment.add(ticksInSegment);

         residualTime = duration - ticksInSegment * orientationDt;

         ticksRemaining -= ticksInSegment;
         orientationTicksBeforeSegment.add(ticksBefore);
         ticksBefore += ticksInSegment;

         orientationStartSegment.add(totalProblemSize);
         totalProblemSize += ticksInSegment * orientationVariablesPerTick;
      }
   }

   public int getOrientationTicksInSegment(int segmentId)
   {
      return orientationTicksInSegment.get(segmentId);
   }

   public int getOrientationTicksBeforeSegment(int segmentId)
   {
      return orientationTicksBeforeSegment.get(segmentId);
   }

   public int getOrientationStart(int segmentId)
   {
      return orientationStartSegment.get(segmentId);
   }

   public int getTotalOrientationTicks()
   {
      return totalOrientationTicks;
   }

   public double getOrientationDt()
   {
      return orientationDt;
   }

   public int getSegmentForTick(int tick)
   {
      int segmentId = 0;
      while (segmentId < getNumberOfSegments() && tick > (getOrientationTicksBeforeSegment(segmentId) + getOrientationTicksInSegment(segmentId)))
         segmentId++;

      if (segmentId > getNumberOfSegments() - 1)
         return -1;
      return segmentId;
   }
}
