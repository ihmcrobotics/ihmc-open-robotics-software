package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MPCIndexHandler
{
   private static final int coefficientsPerRho = 4;
   private static final int comCoefficientsPerSegment = 6;

   private int comCoefficientSize = 0;
   private int orientationCoefficientSize = 0;
   private int totalRhoSize = 0;
   private int totalProblemSize = 0;

   private final Map<RigidBodyBasics, ContactStateToForceMatrixHelper> contactableMap;
   private final Map<String, ContactStateToForceMatrixHelper> contactableStringMap = new HashMap<>();

   private final TIntArrayList bodiesInContact = new TIntArrayList();
   private final TIntArrayList rhoStartIndices = new TIntArrayList();
   private final TIntArrayList rhoCoefficientsInSegment = new TIntArrayList();
   private final TIntArrayList rhoBasesInSegment = new TIntArrayList();

   public MPCIndexHandler(Map<RigidBodyBasics, ContactStateToForceMatrixHelper> contactableMap)
   {
      this.contactableMap = contactableMap;

      for (RigidBodyBasics rigidBody : contactableMap.keySet())
      {
         contactableStringMap.put(rigidBody.getName(), contactableMap.get(rigidBody));
      }
   }

   public void initialize(List<? extends ContactStateProvider> contactSequence)
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
         comCoefficientSize += 6 * comCoefficientsPerSegment;
         bodiesInContact.add(contactSequence.get(i).getNumberOfBodiesInContact());
         List<String> nameOfBodiesInContact = contactSequence.get(i).getBodiesInContact();
         rhoStartIndices.add(totalRhoSize);
         int rhoBasesInSegment = 0;
         for (int j = 0; j < nameOfBodiesInContact.size(); j++)
         {
            rhoBasesInSegment += contactableStringMap.get(nameOfBodiesInContact.get(i)).getRhoSize();
         }
         int rhoCoefficientsInSegment = coefficientsPerRho * rhoBasesInSegment;
         this.rhoBasesInSegment.add(rhoBasesInSegment);
         this.rhoCoefficientsInSegment.add(rhoCoefficientsInSegment);
         totalRhoSize += rhoCoefficientsInSegment;
      }

      totalProblemSize = comCoefficientSize + orientationCoefficientSize + totalRhoSize;
   }

   public int getComCoefficientStartIndex(int segmentId, int ordinal)
   {
      return segmentId * comCoefficientsPerSegment + ordinal;
   }

}
