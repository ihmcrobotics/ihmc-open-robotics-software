package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ActiveSetData;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPlaneForceViewer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

public class MPCContactHandler
{
   protected static final int numberOfBasisVectorsPerContactPoint = 4;
   private static final double mu = 0.8;

   private final double mass;
   private final double gravityZ;

   private final ContactDataList contactDataList = new ContactDataList();

   private final List<ContactData> unusedContactDataList = new ArrayList<>();

   public final List<List<MPCContactPlane>> contactPlanes = new ArrayList<>();
   public final List<ActiveSetData> activeSetData = new ArrayList<>();

   private final LinearMPCIndexHandler indexHandler;

   private final FrictionConeRotationCalculator coneRotationCalculator = new ZeroConeRotationCalculator();
   private final Supplier<MPCContactPlane> contactPlaneProvider = () -> new MPCContactPlane(6, numberOfBasisVectorsPerContactPoint, coneRotationCalculator);

   public MPCContactHandler(LinearMPCIndexHandler indexHandler, double gravityZ, double mass)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;
   }


   public void setContactPlaneViewers(Supplier<ContactPlaneForceViewer> viewerSupplier)
   {
      contactDataList.clear();
      for (int i = 0; i < 6; i++)
      {
         ContactData contactData = contactDataList.addContactData();
         contactData.addContactPlane().setContactPointForceViewer(viewerSupplier.get());
         contactData.addContactPlane().setContactPointForceViewer(viewerSupplier.get());
      }
      contactDataList.clear();
   }

   public void computeMatrixHelpers(List<ContactPlaneProvider> currentContactSequence, List<ContactPlaneProvider> previousContactSequence, double omega)
   {
      contactPlanes.clear();
      activeSetData.clear();

      updatePreviousLists();

      for (int sequenceId = 0; sequenceId < currentContactSequence.size(); sequenceId++)
      {
         ContactPlaneProvider contact = currentContactSequence.get(sequenceId);

         int contactIdx = contactDataList.indexOf(contact);
         ContactData contactData;
         if (contactIdx < 0)
         {
            contactData = createNewContactData(sequenceId, contact);
         }
         else
         {
            contactData = contactDataList.getContactData(contactIdx);
            registerAsUsed(contact, contactData);
         }

         List<MPCContactPlane> contactPlanes = contactData.getPlanes();

         for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         {
            MPCContactPlane contactPlane = contactPlanes.get(contactId);
            contactPlane.computeBasisVectors(contact.getContactsInBodyFrame(contactId), contact.getContactPose(contactId), mu);
         }

         this.contactPlanes.add(contactPlanes);
         this.activeSetData.add(contactData.getActiveSetData());
      }

      // remove the unused ones from the pool
      deallocateUnusedObjects();
   }

   private void updatePreviousLists()
   {
      removeCorruptedPlanes();

      unusedContactDataList.clear();
      for (int i = 0; i < contactDataList.numberOfContacts(); i++)
         unusedContactDataList.add(contactDataList.getContactData(i));
   }

   private ContactData createNewContactData(int sequenceId, ContactPlaneProvider contact)
   {
      ContactData contactData = contactDataList.addContactData();
      contactData.clear();
      contactData.setContact(contact);

      ActiveSetData activeSetData = contactData.getActiveSetData();
      activeSetData.setSegmentNumber(sequenceId);
      activeSetData.setNumberOfVariablesInSegment(indexHandler.getVariablesInSegment(sequenceId));

      for (int contactId = 0; contactId < contact.getNumberOfContactPlanes(); contactId++)
         contactData.addContactPlane();

      return contactData;
   }

   private void registerAsUsed(ContactPlaneProvider contact, ContactData contactData)
   {
      contactData.setContact(contact);

      if (!unusedContactDataList.remove(contactData))
         throw new RuntimeException("oops");
   }

   private void deallocateUnusedObjects()
   {
      for (int i = 0; i < unusedContactDataList.size(); i++)
         contactDataList.removeContact(unusedContactDataList.get(i).getContact());

      removeCorruptedPlanes();
   }

   // TODO this likely can be removed.
   private void removeCorruptedPlanes()
   {
      int i = 0;
      while (i < contactDataList.numberOfContacts())
      {
         ContactData contactData = contactDataList.getContactData(i);
         if (contactData.getContact().getNumberOfContactPlanes() != contactData.getPlanes().size())
         {
            contactDataList.removeContact(i);
            continue;
         }

         i++;
      }
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

   private class ContactData
   {
      private final RecyclingArrayList<MPCContactPlane> contactPlanes = new RecyclingArrayList<>(contactPlaneProvider);
      private final ActiveSetData activeSetData = new ActiveSetData();

      private final ContactPlaneProvider contact = new ContactPlaneProvider();

      public void setContact(ContactPlaneProvider contact)
      {
         this.contact.set(contact);
      }

      public ContactPlaneProvider getContact()
      {
         return contact;
      }

      public void clear()
      {
         contact.reset();
         activeSetData.reset();
         contactPlanes.clear();
      }

      public List<MPCContactPlane> getPlanes()
      {
         return contactPlanes;
      }

      public ActiveSetData getActiveSetData()
      {
         return activeSetData;
      }

      public MPCContactPlane addContactPlane()
      {
         return contactPlanes.add();
      }

      @Override
      public String toString()
      {
         return contact.getTimeInterval().toString();
      }
   }

   private class ContactDataList
   {
      private final RecyclingArrayList<ContactData> contactData = new RecyclingArrayList<>(ContactData::new);

      public void clear()
      {
         contactData.clear();
      }

      public int numberOfContacts()
      {
         return contactData.size();
      }

      public ContactData addContactData()
      {
         return contactData.add();
      }

      public ContactData getContactData(int index)
      {
         return contactData.get(index);
      }

      public boolean removeContact(ContactPlaneProvider contactPlaneProvider)
      {
         int index = indexOf(contactPlaneProvider);
         return removeContact(index);
      }

      public boolean removeContact(int index)
      {
         if (index < 0)
            return false;

         contactData.remove(index);
         return true;
      }

      private static final double timeEpsilon = 1e-2;
      private static final double positionEpsilon = 5e-3;

      public boolean containsContact(ContactPlaneProvider contactPlaneProvider)
      {
         for (int i = 0; i < contactData.size(); i++)
         {
            if (ContactPlaneProvider.epsilonEquals(contactData.get(i).getContact(), contactPlaneProvider, timeEpsilon, positionEpsilon))
               return true;
         }

         return false;
      }

      public int indexOf(ContactPlaneProvider contact)
      {
         for (int i = 0; i < contactData.size(); i++)
         {
            if (ContactPlaneProvider.epsilonEquals(contactData.get(i).getContact(), contact, timeEpsilon, positionEpsilon))
            {
               return i;
            }
         }

         return -1;
      }
   }
}
