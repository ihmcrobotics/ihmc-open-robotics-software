package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;

public class JumpMessageHandler
{
   private final List<ContactState> contactStateList;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final Point2D tempPoint = new Point2D();
   private final FramePoint3D finalPosition = new FramePoint3D(worldFrame, 0.0, 0.0, 0.437);
   private final FrameQuaternion finalOrientation = new FrameQuaternion(worldFrame, 0.0, 0.0, 1.0 * Math.sin(Math.PI / 4), Math.cos(Math.PI / 4));

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

   public void createJumpSequenceForTesting(FramePoint3D currentPosition, FrameQuaternion currentOrientation, JumpStateEnum currentState)
   {
      contactStateList.clear();
      switch (currentState)
      {
      case STANDING:
         break;
      case TAKE_OFF:
         ContactState launchState = new ContactState(worldFrame);
         launchState.setDuration(0.4);
         launchState.setCoMPosition(currentPosition);
         launchState.setCoMOrientation(currentOrientation);
         createRectangle(currentPosition.getX(), currentPosition.getY(), 0.01, 0.01, tempPolygon);
         launchState.setSupportPolygon(tempPolygon);
         launchState.setContactType(ContactType.DOUBLE_SUPPORT);
         contactStateList.add(launchState);
      case FLIGHT:
         ContactState flightState = new ContactState(worldFrame);
         flightState.setDuration(0.20);
         flightState.setCoMPosition(currentPosition);
         flightState.setCoMOrientation(currentOrientation);
         tempPolygon.clear();
         flightState.setSupportPolygon(tempPolygon);
         flightState.setContactType(ContactType.NO_SUPPORT);
         contactStateList.add(flightState);
      case LANDING:
         ContactState landingState = new ContactState(worldFrame);
         landingState.setDuration(0.5);
         createRectangle(currentPosition.getX(), currentPosition.getY(), 0.01, 0.01, tempPolygon);
         landingState.setCoMPosition(finalPosition);
         landingState.setCoMOrientation(finalOrientation);
         landingState.setSupportPolygon(tempPolygon);
         landingState.setContactType(ContactType.DOUBLE_SUPPORT);
         contactStateList.add(landingState);
         break;
      default:
         throw new RuntimeException("Unhandled jump state");
      }
   }

   private void createRectangle(double centroidX, double centroidY, double lengthX, double lengthY, ConvexPolygon2D polygonToSet)
   {
      polygonToSet.clear();
      for (int i = 0; i < 4; i++)
      {
         tempPoint.set(centroidX + Math.pow(-1.0, i) * lengthX * 0.5, centroidY + Math.pow(-1.0, (i / 2)) * lengthY * 0.5);
         polygonToSet.addVertex(tempPoint);
      }
      polygonToSet.update();
   }
}
