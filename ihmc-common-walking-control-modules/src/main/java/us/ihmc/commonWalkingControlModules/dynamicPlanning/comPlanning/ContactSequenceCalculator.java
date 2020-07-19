package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import java.util.List;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ContactSequenceCalculator<T extends TimedContactInterval>
{
   private final CoPWaypointCalculator<T> copWaypointCalculator;

   private final RecyclingArrayList<SettableContactStateProvider> contactStates = new RecyclingArrayList<>(SettableContactStateProvider::new);
   private final LineSegment2D tempLine = new LineSegment2D();

   private final YoDouble startTime;
   private final YoFramePoint3D initialCoPPosition;


   public ContactSequenceCalculator(CoPWaypointCalculator<T> copWaypointCalculator, YoRegistry registry)
   {
      this.copWaypointCalculator = copWaypointCalculator;

      startTime = new YoDouble("contactSequenceStartTime", registry);
      initialCoPPosition = new YoFramePoint3D("initialCoPPosition", ReferenceFrame.getWorldFrame(), registry);
   }

   private final FramePoint3D initialCoPWaypoint = new FramePoint3D();
   private final FramePoint2D initialCoPWaypoint2D = new FramePoint2D();

   public List<? extends ContactStateProvider> compute(List<T> timedContactPhases)
   {
      contactStates.clear();

      // First waypoint is a center of initial support.
      T initialPhase = timedContactPhases.get(0);

      if (isPhaseInContact(timedContactPhases, 0))
      {
         if (startTime.getDoubleValue() < initialPhase.getTimeInterval().getStartTime())
         { // FIXME this should never be happening
            SettableContactStateProvider contactStateProvider = contactStates.add();
            contactStateProvider.reset();
            contactStateProvider.setStartCopPosition(initialCoPPosition);
            contactStateProvider.getTimeInterval().setInterval(startTime.getDoubleValue(), initialPhase.getTimeInterval().getStartTime());
            contactStateProvider.setContactState(getContactState(initialPhase));

            copWaypointCalculator.computeCoPWaypoint(initialCoPWaypoint, initialPhase);
            contactStateProvider.setEndCopPosition(initialCoPWaypoint);
         }
         else
         {
            initialCoPWaypoint.set(initialCoPPosition);
         }
      }
      else
      {
         initialCoPWaypoint.setToNaN();
      }

      initialCoPWaypoint2D.set(initialCoPWaypoint);

      ConvexPolygon2DReadOnly initialPolygon = initialPhase.getSupportPolygon();
      if (!initialPolygon.isPointInside(initialCoPWaypoint2D))
      {
         initialPolygon.orthogonalProjection(initialCoPWaypoint2D);
      }

      SettableContactStateProvider contactStateProvider = contactStates.add();
      contactStateProvider.reset();
      contactStateProvider.setStartCopPosition(initialCoPWaypoint2D);
      contactStateProvider.setTimeInterval(initialPhase.getTimeInterval());
      contactStateProvider.setContactState(getContactState(initialPhase));

      for (int i = 1; i < timedContactPhases.size(); i++)
      {
         addComputedCoP(timedContactPhases, i);
      }

      // Last waypoint is at center of final support.
      T finalPhase = timedContactPhases.get(timedContactPhases.size() - 1);
      SettableContactStateProvider finalContact = contactStates.getLast();
      copWaypointCalculator.computeCoPWaypoint(initialCoPWaypoint, finalPhase);
      finalContact.setEndCopPosition(initialCoPWaypoint);
      finalContact.setContactState(getContactState(finalPhase));

      return contactStates;
   }

   public void setInitialCoP(double startTime, FramePoint3DReadOnly initialCoPPosition)
   {
      this.startTime.set(startTime);
      this.initialCoPPosition.set(initialCoPPosition);
   }

   private final Point2D waypoint = new Point2D();
   private final FramePoint3D tempCoPWaypoint = new FramePoint3D();
   private final FramePoint3D tempPreviousCoPWaypoint = new FramePoint3D();


   private void addComputedCoP(List<T> timedContactPhases, int contactIndex)
   {
      T previousContact = timedContactPhases.get(contactIndex - 1);
      T contact = timedContactPhases.get(contactIndex);
      ConvexPolygon2DReadOnly previousPolygon = previousContact.getSupportPolygon();

      // Add a waypoint at the start time of this polygon.
      SettableContactStateProvider previousContactPhase = contactStates.getLast();
      SettableContactStateProvider contactPhase = contactStates.add();
      contactPhase.reset();
      contactPhase.setTimeInterval(contact.getTimeInterval());
      contactPhase.setContactState(getContactState(contact));

      copWaypointCalculator.computeCoPWaypoint(tempCoPWaypoint, contact);
      waypoint.set(tempCoPWaypoint);

      boolean previousPhaseInContact = isPhaseInContact(timedContactPhases, contactIndex - 1);
      boolean currentPhaseInContact = isPhaseInContact(timedContactPhases, contactIndex);

      // This waypoint must be both in this polygon and the previous one:
      if (previousPhaseInContact && currentPhaseInContact && !previousPolygon.isPointInside(waypoint))
      {
         // The line intersection works better then orthogonal projection for funny shaped feet.
         tempLine.set(previousPolygon.getCentroid(), waypoint);
         previousPolygon.intersectionWith(tempLine, waypoint, waypoint);
      }

      tempCoPWaypoint.set(waypoint);

      if (currentPhaseInContact)
      {
         previousContactPhase.setEndCopPosition(tempCoPWaypoint);
      }
      else
      {
         copWaypointCalculator.computeCoPWaypoint(tempPreviousCoPWaypoint, previousContact);
         previousContactPhase.setEndCopPosition(tempPreviousCoPWaypoint);
      }

      if (previousContactPhase.getCopStartPosition().containsNaN())
         previousContactPhase.setStartCopPosition(previousContactPhase.getCopEndPosition());

      contactPhase.setStartCopPosition(tempCoPWaypoint);

      if (!peekIsNextPhaseInContact(timedContactPhases, contactIndex))
         contactPhase.setEndCopPosition(tempCoPWaypoint);
   }

   private static <T extends TimedContactInterval> boolean isPhaseInContact(List<T> timedContactPhases, int contactPhases)
   {
      return isPhaseInContact(timedContactPhases.get(contactPhases));
   }

   private static <T extends TimedContactInterval> ContactState getContactState(T phase)
   {
      return isPhaseInContact(phase) ? ContactState.IN_CONTACT : ContactState.FLIGHT;
   }

   private static <T extends TimedContactInterval> boolean isPhaseInContact(T contactPhase)
   {
      return !contactPhase.getSupportPolygon().isEmpty();
   }

   private static <T extends TimedContactInterval> boolean peekIsNextPhaseInContact(List<T> timedContactPhases, int contactIndex)
   {
      if (timedContactPhases.size() > contactIndex + 1)
         return isPhaseInContact(timedContactPhases, contactIndex + 1);

      return true;
   }
}
