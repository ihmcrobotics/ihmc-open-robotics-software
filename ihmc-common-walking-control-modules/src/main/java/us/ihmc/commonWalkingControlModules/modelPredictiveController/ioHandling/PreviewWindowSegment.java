package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateBasics;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

import java.util.ArrayList;
import java.util.List;

public class PreviewWindowSegment implements TimeIntervalReadOnly
{
   private ContactState contactState = ContactState.IN_CONTACT;
   private final RecyclingArrayList<SettableContactStateProvider> contactPhasesInSegment = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final List<ConvexPolygon2DReadOnly> contactPointsInBodyFrame = new ArrayList<>();
   private final List<FramePose3DReadOnly> contactPoses = new ArrayList<>();

   private double totalDuration = 0.0;
   private int totalContactPoints = 0;

   public void reset()
   {
      contactPhasesInSegment.clear();
      contactState = null;
      contactPointsInBodyFrame.clear();
      contactPoses.clear();
      totalContactPoints = 0;
      totalDuration = 0.0;
   }

   public void set(PreviewWindowSegment other)
   {
      reset();

      for (int i = 0; i < other.getNumberOfContactPhasesInSegment(); i++)
      {
         ContactStateBasics<?> contact = other.getContactPhase(i);
         addContactPhaseInSegment(contact, contact.getTimeInterval().getStartTime(), contact.getTimeInterval().getEndTime());
      }
      for (int i = 0; i < other.getNumberOfContacts(); i++)
         addContact(other.getContactPose(i), other.getContactsInBodyFrame(i));
   }

   public void set(ContactPlaneProvider other)
   {
      reset();

      addContactPhaseInSegment(other, other.getTimeInterval().getStartTime(), other.getTimeInterval().getEndTime());
      for (int i = 0; i < other.getNumberOfContactPlanes(); i++)
         addContact(other.getContactPose(i), other.getContactsInBodyFrame(i));
   }

   public void addContactPhaseInSegment(ContactStateBasics<?> contactPhase, double startTime, double endTime)
   {
      if (contactState != null)
         validateContactPhase(contactPhase);

      SettableContactStateProvider newContact = contactPhasesInSegment.add();
      newContact.set(contactPhase);
      newContact.getTimeInterval().setInterval(startTime, endTime);
      totalDuration += endTime - startTime;
   }

   public void removeContactPhaseFromSegment(int phaseId)
   {
      SettableContactStateProvider phase = contactPhasesInSegment.get(phaseId);
      contactPhasesInSegment.remove(phaseId);

      totalDuration -= phase.getTimeInterval().getDuration();
   }

   private void validateContactPhase(ContactStateBasics<?> contactPhase)
   {
      if (contactState != contactPhase.getContactState())
         throw new RuntimeException("Invalid contact phase to add");
      if (!MathTools.epsilonEquals(contactPhasesInSegment.getLast().getTimeInterval().getEndTime(), contactPhase.getTimeInterval().getStartTime(), 1e-4))
         throw new RuntimeException("Time intervals aren't consecutive.");

      /*
      if (totalContactPoints != contactPhase.getTotalNumberOfPointsInContact())
         throw new RuntimeException("Change in contact points.");

      for (int i = 0; i < contactPoses.size(); i++)
      {
         if (!contactPoses.get(i).epsilonEquals(contactPhase.getContactPose(i), 1e-3))
            throw new RuntimeException("Poses don't match.");
         if (!contactPointsInBodyFrame.get(i).epsilonEquals(contactPhase.getContactsInBodyFrame(i), 1e-3))
            throw new RuntimeException("Points don't match.");
      }

       */
   }

   public void addContact(FramePose3DReadOnly contactPose, ConvexPolygon2DReadOnly contactPointsInBodyFrame)
   {
      this.contactPoses.add(contactPose);
      this.contactPointsInBodyFrame.add(contactPointsInBodyFrame);
      totalContactPoints += contactPointsInBodyFrame.getNumberOfVertices();
   }

   public int getNumberOfContactPhasesInSegment()
   {
      return contactPhasesInSegment.size();
   }

   public ContactStateBasics<?> getContactPhase(int phaseNumber)
   {
      return contactPhasesInSegment.get(phaseNumber);
   }

   public TimeIntervalReadOnly getTimeInterval(int phaseNumber)
   {
      return contactPhasesInSegment.get(phaseNumber).getTimeInterval();
   }

   public double getDuration()
   {
      return totalDuration;
   }

   public void setStartTime(double startTime)
   {
      contactPhasesInSegment.get(0).getTimeInterval().setStartTime(startTime);
   }

   public double getStartTime()
   {
      return contactPhasesInSegment.get(0).getTimeInterval().getStartTime();
   }

   public double getEndTime()
   {
      return contactPhasesInSegment.getLast().getTimeInterval().getEndTime();
   }

   public int getNumberOfContacts()
   {
      return contactPointsInBodyFrame.size();
   }

   public ConvexPolygon2DReadOnly getContactsInBodyFrame(int contactPlane)
   {
      return contactPointsInBodyFrame.get(contactPlane);
   }

   public FramePose3DReadOnly getContactPose(int contactPlane)
   {
      return contactPoses.get(contactPlane);
   }

   public ContactState getContactState()
   {
      return contactState;
   }

   public int getTotalNumberOfPointsInContact()
   {
      return totalContactPoints;
   }

   public static boolean epsilonEquals(PreviewWindowSegment segmentA, PreviewWindowSegment segmentB, double timeEpsilon, double positionEpsilon)
   {
      if (segmentA == null || segmentB == null)
         return false;

      if (!MathTools.epsilonEquals(segmentA.getEndTime(), segmentB.getEndTime(), timeEpsilon))
         return false;

      if (segmentA.getContactState() != segmentB.getContactState())
         return false;

      if (segmentA.getContactState() == ContactState.FLIGHT)
         return true;

      if (segmentA.getNumberOfContacts() != segmentB.getNumberOfContacts())
         return false;

      for (int i = 0; i < segmentA.getNumberOfContacts(); i++)
      {
         if (segmentA.getContactsInBodyFrame(i).getNumberOfVertices() != segmentB.getContactsInBodyFrame(i).getNumberOfVertices())
            return false;

         if (segmentA.getContactPose(i).getPosition().distanceXYSquared(segmentB.getContactPose(i).getPosition()) >  positionEpsilon * positionEpsilon)
            return false;
      }

      return true;
   }
}
