package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import gnu.trove.map.hash.TIntIntHashMap;

import java.util.List;

/**
 * Provides the internal indices for the constraint matrices for the {@link CoMTrajectoryPlanner}.
 */
public class CoMTrajectoryPlannerIndexHandler
{
   private static final int polynomialCoefficientsPerSegment = 6;
   private static final int vrpConstraintsPerSegment = 4;

   private int totalNumberOfCoefficients;
   private int numberOfVRPWaypoints;
   private final TIntIntHashMap vrpWaypointIndices = new TIntIntHashMap();

   public void update(List<? extends ContactStateProvider> contactSequence)
   {
      vrpWaypointIndices.clear();
      totalNumberOfCoefficients = 0;
      numberOfVRPWaypoints = 0;
      if (contactSequence.get(0).getContactState() == ContactState.IN_CONTACT)
      {
         vrpWaypointIndices.put(0, numberOfVRPWaypoints);
         numberOfVRPWaypoints += vrpConstraintsPerSegment; // start and end
      }
      totalNumberOfCoefficients += polynomialCoefficientsPerSegment;
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         if (contactSequence.get(sequenceId).getContactState() == ContactState.IN_CONTACT)
         {
            vrpWaypointIndices.put(sequenceId, numberOfVRPWaypoints);
            numberOfVRPWaypoints += vrpConstraintsPerSegment;
         }
         totalNumberOfCoefficients += polynomialCoefficientsPerSegment;
      }
   }

   public int getTotalNumberOfCoefficients()
   {
      return totalNumberOfCoefficients;
   }

   public int getNumberOfVRPWaypoints()
   {
      return numberOfVRPWaypoints;
   }

   public int getContactSequenceStartIndex(int sequenceNumber)
   {
      return 6 * sequenceNumber;
   }

   public int getVRPWaypointStartPositionIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber);
   }

   public int getVRPWaypointStartVelocityIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 1;
   }

   public int getVRPWaypointFinalPositionIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 2;
   }

   public int getVRPWaypointFinalVelocityIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 3;
   }
}
