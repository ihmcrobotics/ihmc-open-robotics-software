package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;

public class ContactStatesAndUpcomingFootstepData
{
   private RobotSide supportLeg;
   private ReferenceFrame centerOfMassFrame, pelvisZUpFrame;
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

   public ReferenceFrame getPelvisZUpFrame()
   {
      return pelvisZUpFrame;
   }

   public void setCenterOfMassAndPelvisZUpFrames(ReferenceFrame centerOfMassFrame, ReferenceFrame pelvisZUpFrame)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.pelvisZUpFrame = pelvisZUpFrame;
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

   public void setContactStates(List<? extends PlaneContactState> contactStates)
   {
      this.contactStates.clear();
      for (int i=0; i<contactStates.size(); i++)
      {
         this.contactStates.add(contactStates.get(i));
      }
   }

   public void set(ReferenceFrame centerOfMassFrame, ReferenceFrame pelvisFrame, RobotSide supportLeg, Footstep upcomingFootstep,
         List<PlaneContactState> contactStates)
   {
      this.setCenterOfMassAndPelvisZUpFrames(centerOfMassFrame, pelvisFrame);
      this.setSupportLeg(supportLeg);
      this.setUpcomingFootstep(upcomingFootstep);
      this.setContactStates(contactStates);
   }
}