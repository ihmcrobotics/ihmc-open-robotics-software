package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableRollingBody;
import us.ihmc.robotics.geometry.FramePoint2d;

public class RollingContactStateCommand
{
   private final ContactableRollingBody contactableRollingBody;
   private final List<FramePoint2d> contactPoints;
   private final double coefficientOfFriction;

   public RollingContactStateCommand(ContactableRollingBody contactableRollingBody, List<FramePoint2d> contactPoints, double coefficientOfFriction)
   {
      this.contactableRollingBody = contactableRollingBody;
      this.contactPoints = contactPoints;
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public String toString()
   {
      return "RollingContactStateCommand: contactableRollingBody = " + contactableRollingBody.getName();
   }
}
