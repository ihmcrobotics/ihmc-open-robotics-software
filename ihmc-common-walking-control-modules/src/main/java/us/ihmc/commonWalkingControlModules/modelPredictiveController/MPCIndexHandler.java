package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MPCIndexHandler
{
   public static final int coefficientsPerRho = 4;
   public static final int comCoefficientsPerSegment = 6;

   private int comCoefficientSize = 0;
   private int orientationCoefficientSize = 0;
   private int totalRhoSize = 0;
   private int totalProblemSize = 0;

   private final int numberOfBasisVectorsPerContactPoint;
   private final TIntArrayList bodiesInContact = new TIntArrayList();
   private final TIntArrayList rhoStartIndices = new TIntArrayList();
   private final TIntArrayList rhoCoefficientsInSegment = new TIntArrayList();
   private final TIntArrayList rhoBasesInSegment = new TIntArrayList();

   public MPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
   }

   public void initialize(List<ContactPlaneProvider> contactSequence)
   {
      comCoefficientSize = 0;
      orientationCoefficientSize = 0;
      totalRhoSize = 0;
      bodiesInContact.clear();
      rhoStartIndices.clear();
      rhoCoefficientsInSegment.clear();
      rhoBasesInSegment.clear();

      for (int i = 0; i < contactSequence.size(); i++)
      {
         comCoefficientSize += comCoefficientsPerSegment;
         bodiesInContact.add(contactSequence.get(i).getNumberOfContactPlanes());
         int rhoBasesInSegment = 0;
         for (int j = 0; j < contactSequence.get(i).getNumberOfContactPlanes(); j++)
         {
            rhoBasesInSegment += numberOfBasisVectorsPerContactPoint * contactSequence.get(i).getNumberOfContactPointsInPlane(j);
         }
         int rhoCoefficientsInSegment = coefficientsPerRho * rhoBasesInSegment;
         this.rhoBasesInSegment.add(rhoBasesInSegment);
         this.rhoCoefficientsInSegment.add(rhoCoefficientsInSegment);
      }

      for (int i = 0; i < contactSequence.size(); i++)
      {
         rhoStartIndices.add(totalRhoSize + comCoefficientSize);
         totalRhoSize += rhoCoefficientsInSegment.get(i);
      }

      totalProblemSize = comCoefficientSize + orientationCoefficientSize + totalRhoSize;
   }

   public int getComCoefficientStartIndex(int segmentId, int ordinal)
   {
      return segmentId * comCoefficientsPerSegment + 2 * ordinal;
   }

   public int getRhoBasisStartIndex(int segmentId)
   {
      return rhoBasesInSegment.get(segmentId);
   }

   public int getRhoCoefficientStartIndex(int segmentId)
   {
      return rhoStartIndices.get(segmentId);
   }

   public int getTotalProblemSize()
   {
      return totalProblemSize;
   }
}
