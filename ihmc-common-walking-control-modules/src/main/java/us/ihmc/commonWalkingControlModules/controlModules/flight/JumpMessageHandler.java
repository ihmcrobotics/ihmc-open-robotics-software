package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class JumpMessageHandler
{
   private final List<ContactState> contactStateList;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public JumpMessageHandler()
   {
      this.contactStateList = new ArrayList<>();
      reset();
   }

   public void reset()
   {
      this.contactStateList.clear();
   }

   public List<ContactState> getContactStateList()
   {
      return contactStateList;
   }

   public void submitContactState(ContactState contactStateToStore)
   {
      throw new RuntimeException("Unimplmented method");
   }

   public void createJumpSequenceForTesting(FramePoint3D currentPosition)
   {
      ContactState launchState = new ContactState(worldFrame);
      launchState.setContactStateInitialTime(0.0);

      ContactState flightState = new ContactState(worldFrame);
      flightState.setContactStateInitialTime(0.0);

      ContactState landingState = new ContactState(worldFrame);
      landingState.setContactStateInitialTime(0.0);
   }
}
