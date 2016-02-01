package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import java.util.List;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;

public class PlaneContactStateCommand
{
   private final ContactablePlaneBody contactableBody;
   private final List<FramePoint2d> contactPoints;
   private final double coefficientOfFriction;
   private final FrameVector normalContactVector;

   public PlaneContactStateCommand(ContactablePlaneBody contactableBody, List<FramePoint2d> contactPoints, double coefficientOfFriction,
                                   FrameVector normalContactVector)
   {
      this.contactableBody = contactableBody;
      this.contactPoints = contactPoints;
      this.coefficientOfFriction = coefficientOfFriction;
      this.normalContactVector = normalContactVector;
   }

   public String toString()
   {
      return "PlaneContactStateCommand: contactableBody = " + contactableBody.getName();
   }
}
