package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class GroundReactionWrenchDistributorInputData
{
   private final ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   
   private SpatialForceVector desiredNetSpatialForceVector;
   private RobotSide upcomingSupportSide;
   
   public void reset()
   {
      contactStates.clear();
   }

   public void addContact(PlaneContactState contactState)
   {
      contactStates.add(contactState);
   }
   
   public void setSpatialForceVectorAndUpcomingSupportSide(SpatialForceVector desiredNetSpatialForceVector, RobotSide upcomingSupportSide)
   {
      this.desiredNetSpatialForceVector = desiredNetSpatialForceVector;
      this.upcomingSupportSide = upcomingSupportSide;
   }
   
   public ArrayList<PlaneContactState> getContactStates()
   {
      return contactStates;
   }

   public SpatialForceVector getDesiredNetSpatialForceVector()
   {
      return desiredNetSpatialForceVector;
   }
   
   public RobotSide getUpcomingSupportSide()
   {
      return upcomingSupportSide;
   }
   
}
