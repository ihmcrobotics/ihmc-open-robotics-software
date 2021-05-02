package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPlaneForceViewer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class MPCContactHandler
{
   protected static final int numberOfBasisVectorsPerContactPoint = 4;
   private static final double mu = 0.8;

   private final double maxContactForce;
   private final double mass;
   private final double gravityZ;

   public final RecyclingArrayList<RecyclingArrayList<MPCContactPlane>> contactPlanePool;
   public final RecyclingArrayList<List<MPCContactPlane>> contactPlanes;

   public MPCContactHandler(double gravityZ, double mass)
   {
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;

      this.maxContactForce = 2.0 * Math.abs(gravityZ);

      FrictionConeRotationCalculator coneRotationCalculator = new ZeroConeRotationCalculator();
      Supplier<MPCContactPlane> contactPlaneHelperProvider = () -> new MPCContactPlane(6, numberOfBasisVectorsPerContactPoint, coneRotationCalculator);
      contactPlanePool = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(contactPlaneHelperProvider));
      contactPlanes = new RecyclingArrayList<>(ArrayList::new);
   }


   public void setContactPlaneViewers(Supplier<ContactPlaneForceViewer> viewerSupplier)
   {
      contactPlanePool.clear();
      for (int i = 0; i < 2; i++)
      {
         RecyclingArrayList<MPCContactPlane> helpers = contactPlanePool.add();
         helpers.clear();
         for (int j = 0; j < 6; j++)
         {
            helpers.add().setContactPointForceViewer(viewerSupplier.get());
         }
         helpers.clear();
      }
      contactPlanePool.clear();
   }

   public void computeMatrixHelpers(List<ContactPlaneProvider> contactSequence, double omega)
   {
      contactPlanePool.clear();

      for (int sequenceId = 0; sequenceId < contactSequence.size(); sequenceId++)
      {
         ContactPlaneProvider contact = contactSequence.get(sequenceId);
         double duration = contact.getTimeInterval().getDuration();

         RecyclingArrayList<MPCContactPlane> contactPlaneHelpers = contactPlanePool.add();
         contactPlaneHelpers.clear();

         double objectiveForce = gravityZ / contact.getNumberOfContactPlanes();
         for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         {
            MPCContactPlane contactPlaneHelper = contactPlaneHelpers.add();
            contactPlaneHelper.setMaxNormalForce(maxContactForce);
            contactPlaneHelper.computeBasisVectors(contact.getContactsInBodyFrame(contactId), contact.getContactPose(contactId), mu);
            contactPlaneHelper.computeAccelerationIntegrationMatrix(duration, omega, objectiveForce);
         }
      }
   }

   /*
   private void computeMatrixHelpers(List<ContactPlaneProvider> contactSequence)
   {
      List<ContactPlaneProvider> planningWindowForPreviousSolution = linearTrajectoryHandler.getPlanningWindowForSolution();
      boolean firstSegmentRemoved = false;

      if (!doTwoPlaneProvidersMatch(planningWindowForPreviousSolution.get(0), contactSequence.get(0)))
      {
         // check if the next one changed, which is possible
         firstSegmentRemoved = true;
         contactPlanePool.fastRemove(0);
      }

      int sequenceId = 0;
      for (; sequenceId < contactSequence.size(); sequenceId++)
      {
         ContactPlaneProvider contact = contactSequence.get(sequenceId);
         double duration = contact.getTimeInterval().getDuration();

         int previousId = firstSegmentRemoved ? sequenceId + 1 : sequenceId;
         boolean recycleSegment = doTwoPlaneProvidersMatch(planningWindowForPreviousSolution.get(previousId), contactSequence.get(sequenceId));

         RecyclingArrayList<MPCContactPlane> contactPlanes;
         if (recycleSegment)
         {
            contactPlanes = contactPlanePool.get(sequenceId);
         }
         else
         {
            contactPlanePool.remove(sequenceId);
            contactPlanes = contactPlanePool.insertAtIndex(sequenceId);
            contactPlanes.clear();
         }

         double objectiveForce = gravityZ / contact.getNumberOfContactPlanes();
         for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         {
            MPCContactPlane contactPlane;
            if (recycleSegment)
            {
               contactPlane = contactPlanes.get(contactId);
            }
            else
            {
               contactPlane = contactPlanes.add();
               // TODO make this do something
               contactPlane.reset();
            }
            contactPlane.setMaxNormalForce(maxContactForce);
            contactPlane.computeBasisVectors(contact.getContactsInBodyFrame(contactId), contact.getContactPose(contactId), mu);
            contactPlane.computeAccelerationIntegrationMatrix(duration, omega.getValue(), objectiveForce);
         }
      }
   }

    */


   private static final double timeEpsilon = 1e-4;
   private static final double positionEpsilon = 5e-3;

   private static boolean doTwoPlaneProvidersMatch(ContactPlaneProvider groundTruth, ContactPlaneProvider candidate)
   {
      if (groundTruth == null || candidate == null)
         return false;

      if (!groundTruth.getTimeInterval().epsilonContains(candidate.getTimeInterval().getStartTime(), timeEpsilon))
         return false;
      if (!MathTools.epsilonEquals(groundTruth.getTimeInterval().getEndTime(), candidate.getTimeInterval().getEndTime(), timeEpsilon))
         return false;

      if (groundTruth.getNumberOfContactPlanes() != candidate.getNumberOfContactPlanes())
         return false;

      for (int i = 0; i < groundTruth.getNumberOfContactPlanes(); i++)
      {
         if (groundTruth.getNumberOfContactPointsInPlane(i) != candidate.getNumberOfContactPointsInPlane(i))
            return false;

         if (!groundTruth.getContactPose(i).getPosition().epsilonEquals(candidate.getContactPose(i).getPosition(), positionEpsilon))
            return false;
      }

      return true;
   }


   public List<? extends List<MPCContactPlane>> getContactPlanes()
   {
      return contactPlanePool;
   }

   public int getNumberOfContactPlanesInSegment(int segmentId)
   {
      return getContactPlanesForSegment(segmentId).size();
   }

   public List<MPCContactPlane> getContactPlanesForSegment(int segmentId)
   {
      return contactPlanePool.get(segmentId);
   }

   public MPCContactPlane getContactPlane(int segmentId, int planeId)
   {
      return contactPlanePool.get(segmentId).get(planeId);
   }
}
