package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ActiveSetData;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPlaneForceViewer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

public class MPCContactHandler
{
   protected static final int numberOfBasisVectorsPerContactPoint = 4;
   private static final double mu = 0.8;

   private final double maxContactForce;
   private final double mass;
   private final double gravityZ;

   private final HashMap<ContactPlaneProvider, ContactData> contactMap = new HashMap<>();

   private final RecyclingArrayList<ActiveSetData> activeSetPool = new RecyclingArrayList<>(ActiveSetData::new);
   private final RecyclingArrayList<MPCContactPlane> contactPlanePool;
   private final RecyclingArrayList<List<MPCContactPlane>> contactPlaneListPool = new RecyclingArrayList<>(ArrayList::new);
   private final RecyclingArrayList<ContactData> contactDataPool = new RecyclingArrayList<>(ContactData::new);

   private final List<ActiveSetData> unusedActiveSetList = new ArrayList<>();
   private final List<MPCContactPlane> unusedContactPlanes = new ArrayList<>();
   private final List<List<MPCContactPlane>> unusedContactLists = new ArrayList<>();
   private final List<ContactData> unusedContactDataList = new ArrayList<>();
   private final List<ContactPlaneProvider> unusedPlaneProvider = new ArrayList<>();

   public final List<List<MPCContactPlane>> contactPlanes = new ArrayList<>();
   public final List<ActiveSetData> activeSetData = new ArrayList<>();

   public MPCContactHandler(double gravityZ, double mass)
   {
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;

      this.maxContactForce = 2.0 * Math.abs(gravityZ);

      FrictionConeRotationCalculator coneRotationCalculator = new ZeroConeRotationCalculator();
      Supplier<MPCContactPlane> contactPlaneHelperProvider = () -> new MPCContactPlane(6, numberOfBasisVectorsPerContactPoint, coneRotationCalculator);
      contactPlanePool = new RecyclingArrayList<>(contactPlaneHelperProvider);
   }


   public void setContactPlaneViewers(Supplier<ContactPlaneForceViewer> viewerSupplier)
   {
      contactPlanePool.clear();
      for (int i = 0; i < 12; i++)
         contactPlanePool.add().setContactPointForceViewer(viewerSupplier.get());
      contactPlanePool.clear();
   }

   public void computeMatrixHelpers(List<ContactPlaneProvider> currentContactSequence, List<ContactPlaneProvider> previousContactSequence, double omega)
   {
      updatePreviousLists(previousContactSequence);

      for (int sequenceId = 0; sequenceId < currentContactSequence.size(); sequenceId++)
      {
         ContactPlaneProvider contact = currentContactSequence.get(sequenceId);

         ContactData contactData = contactMap.get(contact);
         if (contactData == null)
            contactData = createNewContactData(contact);
         else
            registerAsUsed(contact, contactData);

         List<MPCContactPlane> contactPlanes = contactData.getPlanes();
         double objectiveForce = gravityZ / contact.getNumberOfContactPlanes();
         double duration = contact.getTimeInterval().getDuration();

         for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         {
            MPCContactPlane contactPlane = contactPlanes.get(contactId);
            contactPlane.setMaxNormalForce(maxContactForce);
            contactPlane.computeBasisVectors(contact.getContactsInBodyFrame(contactId), contact.getContactPose(contactId), mu);
            contactPlane.computeAccelerationIntegrationMatrix(duration, omega, objectiveForce);
         }

         this.contactPlanes.add(contactPlanes);
         this.activeSetData.add(contactData.getActiveSetData());

         // store it back to update the change a little better
         contactMap.put(contact, contactData);
      }

      // remove the unused ones from the pool
      deallocateUnusedObjects();
   }

   private void updatePreviousLists(List<ContactPlaneProvider> previousContactSequence)
   {
      unusedPlaneProvider.clear();
      for (int i = 0; i < previousContactSequence.size(); i++)
         unusedPlaneProvider.add(previousContactSequence.get(i));

      unusedActiveSetList.clear();
      for (int i = 0; i < activeSetPool.size(); i++)
         unusedActiveSetList.add(activeSetPool.get(i));

      unusedContactPlanes.clear();
      for (int i = 0; i < contactPlanePool.size(); i++)
         unusedContactPlanes.add(contactPlanePool.get(i));

      unusedContactLists.clear();
      for (int i = 0; i < contactPlaneListPool.size(); i++)
         unusedContactLists.add(contactPlaneListPool.get(i));

      unusedContactDataList.clear();
      for (int i = 0; i < contactDataPool.size(); i++)
         unusedContactDataList.add(contactDataPool.get(i));
   }

   private ContactData createNewContactData(ContactPlaneProvider contact)
   {
      ContactData contactData = contactDataPool.add();
      ActiveSetData activeSetData = activeSetPool.add();
      activeSetData.reset();
      List<MPCContactPlane> contactPlanes = contactPlaneListPool.add();
      contactPlanes.clear();
      for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         contactPlanes.add(contactPlanePool.add());

      contactData.setPlanes(contactPlanes);
      contactData.setActiveSetData(activeSetData);
      return contactData;
   }

   private void registerAsUsed(ContactPlaneProvider contact, ContactData contactData)
   {
      unusedPlaneProvider.remove(contact);
      unusedContactDataList.remove(contactData);
      unusedActiveSetList.remove(contactData.getActiveSetData());
      List<MPCContactPlane> contactPlanes = contactData.getPlanes();
      unusedContactLists.remove(contactPlanes);
      for (int contactId = 0; contactId < contactPlanes.size(); contactId++)
         unusedContactPlanes.remove(contactPlanes.get(contactId));
   }

   private void deallocateUnusedObjects()
   {
      for (int i = 0; i < unusedActiveSetList.size(); i++)
         activeSetPool.remove(unusedActiveSetList.get(i));

      for (int i = 0; i < unusedContactPlanes.size(); i++)
         contactPlanePool.remove(unusedContactPlanes.get(i));

      for (int i = 0; i < unusedContactLists.size(); i++)
         contactPlaneListPool.remove(unusedContactLists.get(i));

      for (int i = 0; i < unusedContactDataList.size(); i++)
         contactDataPool.remove(unusedContactDataList.get(i));

      for (int i = 0; i < unusedPlaneProvider.size(); i++)
         contactMap.remove(unusedPlaneProvider.get(i));
   }

   public List<? extends List<MPCContactPlane>> getContactPlanes()
   {
      return contactPlanes;
   }

   public int getNumberOfContactPlanesInSegment(int segmentId)
   {
      return getContactPlanesForSegment(segmentId).size();
   }

   public List<MPCContactPlane> getContactPlanesForSegment(int segmentId)
   {
      return contactPlanes.get(segmentId);
   }

   public MPCContactPlane getContactPlane(int segmentId, int planeId)
   {
      return contactPlanes.get(segmentId).get(planeId);
   }

   public ActiveSetData getActiveSetData(int segmentId)
   {
      return activeSetData.get(segmentId);
   }

   private static class ContactData extends MutablePair<List<MPCContactPlane>, ActiveSetData>
   {
      public List<MPCContactPlane> getPlanes()
      {
         return getLeft();
      }

      public ActiveSetData getActiveSetData()
      {
         return getRight();
      }

      public void setPlanes(List<MPCContactPlane> planes)
      {
         setLeft(planes);
      }

      public void setActiveSetData(ActiveSetData activeSetData)
      {
         setRight(activeSetData);
      }
   }
}
