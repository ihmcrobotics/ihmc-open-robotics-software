package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CenterOfPressureCommandCalculator
{
   // State variables
   private final ReferenceFrame midFootZUpFrame;
   private final SideDependentList<ContactableFoot> contactableFeet;
   private final DoubleProvider centerOfPressureWeight;

   // Outputs
   private final CenterOfPressureCommand centerOfPressureCommand = new CenterOfPressureCommand();

   // Temp objects for calculation
   private final FramePoint2DBasics desiredCoPFootFrame = new FramePoint2D();


   public CenterOfPressureCommandCalculator(ReferenceFrame midFootZUpFrame,
                                            SideDependentList<ContactableFoot> contactableFeet,
                                            YoRegistry registry)
   {
      this.midFootZUpFrame = midFootZUpFrame;
      this.contactableFeet = contactableFeet;

      centerOfPressureWeight = new DoubleParameter("CenterOfPressureObjectiveWeight", registry, 0.0);
   }

   public CenterOfPressureCommand getCenterOfPressureCommand()
   {
      return centerOfPressureCommand;
   }

   public void computeCenterOfPressureCommand(FramePoint2DReadOnly desiredCoP,
                                              SideDependentList<PlaneContactStateCommand> contactStateCommands,
                                              SideDependentList<? extends FrameConvexPolygon2DReadOnly> footSupportPolygonsInSoleFrame)
   {
      boolean leftInContact = contactStateCommands.get(RobotSide.LEFT).getNumberOfContactPoints() > 0;
      boolean rightInContact = contactStateCommands.get(RobotSide.RIGHT).getNumberOfContactPoints() > 0;

      if (leftInContact != rightInContact)
      {
         if (leftInContact)
            centerOfPressureCommand.setContactingRigidBody(contactableFeet.get(RobotSide.LEFT).getRigidBody());
         else
            centerOfPressureCommand.setContactingRigidBody(contactableFeet.get(RobotSide.RIGHT).getRigidBody());

         desiredCoPFootFrame.setIncludingFrame(desiredCoP);
         desiredCoPFootFrame.changeFrame(midFootZUpFrame);
         centerOfPressureCommand.setDesiredCoP(desiredCoPFootFrame);
         centerOfPressureCommand.setWeight(midFootZUpFrame, centerOfPressureWeight.getValue(), centerOfPressureWeight.getValue());
      }
      else if (leftInContact)
      {
         // check if it's to the outside of the foot
         boolean setCommand = false;
         for (RobotSide robotSide : RobotSide.values)
         {
            desiredCoPFootFrame.setIncludingFrame(desiredCoP);
            desiredCoPFootFrame.changeFrameAndProjectToXYPlane(contactableFeet.get(robotSide).getSoleFrame());
            if (robotSide.negateIfRightSide(desiredCoPFootFrame.getY()) > 0.0 && footSupportPolygonsInSoleFrame.get(robotSide).isPointInside(desiredCoPFootFrame))
            {
               // it is to the outside of the foot, so add the command
               centerOfPressureCommand.setContactingRigidBody(contactableFeet.get(robotSide).getRigidBody());
               centerOfPressureCommand.setDesiredCoP(desiredCoPFootFrame);
               centerOfPressureCommand.setWeight(contactableFeet.get(robotSide).getSoleFrame(), centerOfPressureWeight.getValue(), centerOfPressureWeight.getValue());
               setCommand = true;
               break;
            }
         }

         if (!setCommand)
         {
            centerOfPressureCommand.setContactingRigidBody(null);
            centerOfPressureCommand.setDesiredCoP(desiredCoP);
            centerOfPressureCommand.setWeight(midFootZUpFrame, centerOfPressureWeight.getValue(), centerOfPressureWeight.getValue());
         }
      }
      else
      {
         centerOfPressureCommand.setContactingRigidBody(null);
         centerOfPressureCommand.setWeight(midFootZUpFrame, 0.0, 0.0);
      }
   }
}
