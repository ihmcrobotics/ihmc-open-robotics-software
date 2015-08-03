package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.bipedSupportPolygons.CylindricalContactState;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class GroundReactionWrenchDistributorInputData
{
   private final List<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   private final List<CylindricalContactState> cylinderContacts = new ArrayList<CylindricalContactState>();
   private SpatialForceVector desiredNetSpatialForceVector;
   private RobotSide upcomingSupportSide;
   
   public void reset()
   {
      contactStates.clear();
      cylinderContacts.clear();
   }

   public void addPlaneContact(PlaneContactState contactState)
   {
      contactStates.add(contactState);
   }
   
   public void setSpatialForceVectorAndUpcomingSupportSide(SpatialForceVector desiredNetSpatialForceVector, RobotSide upcomingSupportSide)
   {
      this.desiredNetSpatialForceVector = desiredNetSpatialForceVector;
      this.upcomingSupportSide = upcomingSupportSide;
   }
   
   public List<PlaneContactState> getContactStates()
   {
      return contactStates;
   }
   
   public List<CylindricalContactState> getCylindricalContactStates()
   {
      return cylinderContacts;
   }

   public SpatialForceVector getDesiredNetSpatialForceVector()
   {
      return desiredNetSpatialForceVector;
   }
   
   public RobotSide getUpcomingSupportSide()
   {
      return upcomingSupportSide;
   }

   public void addCylinderContact(CylindricalContactState cylinderContact)
   {
      cylinderContacts.add(cylinderContact);      
   }
   
}
