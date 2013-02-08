package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ContactStatesAndUpcomingFootstepData
{
   private RobotSide supportLeg;
   private ReferenceFrame centerOfMassFrame;
   private Footstep upcomingFootstep;
   private final ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   
   public ContactStatesAndUpcomingFootstepData()
   {
      
   }

   public RobotSide getSupportLeg()
   {
      return supportLeg;
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      this.supportLeg = supportLeg;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public void setCenterOfMassFrame(ReferenceFrame centerOfMassFrame)
   {
      this.centerOfMassFrame = centerOfMassFrame;
   }

   public Footstep getUpcomingFootstep()
   {
      return upcomingFootstep;
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      this.upcomingFootstep = upcomingFootstep;
   }

   public List<PlaneContactState> getContactStates()
   {
      return contactStates;
   }

   public void setContactStates(List<PlaneContactState> contactStates)
   {
      this.contactStates.clear();
      
      for (PlaneContactState planeContactState : contactStates)
      {
         this.contactStates.add(planeContactState);
      }
   }

   public void set(ReferenceFrame centerOfMassFrame, RobotSide supportLeg, Footstep upcomingFootstep, List<PlaneContactState> contactStates)
   {
      this.setCenterOfMassFrame(centerOfMassFrame);
      this.setSupportLeg(supportLeg);
      this.setUpcomingFootstep(upcomingFootstep);
      this.setContactStates(contactStates);
   }
}
