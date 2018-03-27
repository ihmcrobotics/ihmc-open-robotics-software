package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class JumpMessageHandler
{
   private final List<ContactState> contactStateList;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private final FramePoint2D tempPoint = new FramePoint2D();

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
   
   public void createJumpSequenceForTesting(FramePoint3D currentPosition, JumpStateEnum currentState)
   {
      switch (currentState)
      {
      case TAKE_OFF:
         ContactState launchState = new ContactState(worldFrame);
         launchState.setContactStateDuration(0.45);
         createRectangle(currentPosition.getReferenceFrame(), currentPosition.getX(), currentPosition.getY(), 0.01, 0.01, tempPolygon);
         launchState.setSupportPolygon(tempPolygon);
         contactStateList.add(launchState);
      case FLIGHT:
         ContactState flightState = new ContactState(worldFrame);
         flightState.setContactStateDuration(0.1);
         tempPolygon.clear();
         flightState.setSupportPolygon(tempPolygon);
         contactStateList.add(flightState);
      case LANDING:
         ContactState landingState = new ContactState(worldFrame);
         landingState.setContactStateDuration(0.45);
         createRectangle(currentPosition.getReferenceFrame(), currentPosition.getX(), currentPosition.getY(), 0.01, 0.01, tempPolygon);
         landingState.setSupportPolygon(tempPolygon);
         contactStateList.add(landingState);
         break;
      default: throw new RuntimeException("Unhandled jump state");
      }
   }

   private void createRectangle(ReferenceFrame referenceFrame, double centroidX, double centroidY, double lengthX, double lengthY,
                                FrameConvexPolygon2d polygonToSet)
   {
      polygonToSet.clear();
      for (int i = 0; i < 4; i++)
      {
         tempPoint.setIncludingFrame(referenceFrame, centroidX + Math.pow(-1.0, i) * lengthX * 0.5, centroidY + Math.pow(-1.0, (i / 2)) * lengthY * 0.5);
         polygonToSet.addVertex(tempPoint);
      }
      polygonToSet.update();
   }
}
