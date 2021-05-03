package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import gnu.trove.list.array.TIntArrayList;

public class ActiveSetData
{
   private boolean hasData = false;

   private int segmentNumber = -1;
   private int numberOfVariablesInSegment = -1;
   private int numberOfInequalityConstraints = -1;
   private int numberOfLowerBoundConstraints = -1;
   private int numberOfUpperBoundConstraints = -1;

   private final TIntArrayList activeInequalityIndices = new TIntArrayList();
   private final TIntArrayList activeLowerBoundIndices = new TIntArrayList();
   private final TIntArrayList activeUpperBoundIndices = new TIntArrayList();

   public void reset()
   {
      hasData = false;

      segmentNumber = -1;
      numberOfVariablesInSegment = -1;

      numberOfInequalityConstraints = 0;
      numberOfLowerBoundConstraints = 0;
      numberOfUpperBoundConstraints = 0;

      activeInequalityIndices.reset();
      activeLowerBoundIndices.reset();
      activeUpperBoundIndices.reset();
   }

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public void setNumberOfVariablesInSegment(int numberOfVariablesInSegment)
   {
      this.numberOfVariablesInSegment = numberOfVariablesInSegment;
   }

   public void addInequalityConstraints(int numberOfConstraints)
   {
      numberOfInequalityConstraints += numberOfConstraints;
   }

   public void addLowerBoundConstraints(int numberOfConstraints)
   {
      numberOfLowerBoundConstraints += numberOfConstraints;
   }

   public void addUpperBoundConstraints(int numberOfConstraints)
   {
      numberOfUpperBoundConstraints += numberOfConstraints;
   }

   public void addActiveInequalityConstraint(int constraintIndex)
   {
      activeInequalityIndices.add(constraintIndex);
   }

   public void addActiveLowerBoundConstraint(int constraintIndex)
   {
      activeLowerBoundIndices.add(constraintIndex);
   }

   public void addActiveUpperBoundConstraint(int constraintIndex)
   {
      activeUpperBoundIndices.add(constraintIndex);
   }

   public int getNumberOfInequalityConstraints()
   {
      return numberOfInequalityConstraints;
   }

   public int getNumberOfLowerBoundConstraints()
   {
      return numberOfLowerBoundConstraints;
   }

   public int getNumberOfUpperBoundConstraints()
   {
      return numberOfUpperBoundConstraints;
   }
}
