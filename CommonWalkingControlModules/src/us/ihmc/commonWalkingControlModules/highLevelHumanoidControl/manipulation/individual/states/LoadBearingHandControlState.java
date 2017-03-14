package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.tools.FormattingTools;

/**
 * @author twan
 *         Date: 5/30/13
 */
public class LoadBearingHandControlState extends HandControlState
{
   private final String name;
   private final YoVariableRegistry registry;

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final DoubleYoVariable coefficientOfFriction;
   private final SpatialAccelerationVector handAcceleration;

   private final FrameVector contactNormal;

   private final ContactablePlaneBody handPalm;
   private final ReferenceFrame handFrame;
   private final ReferenceFrame elevatorFrame;


   public LoadBearingHandControlState(String namePrefix, HandControlMode stateEnum, RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox,
         RigidBody elevator, RigidBody endEffector, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      name = namePrefix + FormattingTools.underscoredToCamelCase(this.getStateEnum().toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      spatialAccelerationCommand.set(elevator, endEffector);
      spatialAccelerationCommand.setSelectionMatrixToIdentity();

      this.controllerToolbox = controllerToolbox;

      parentRegistry.addChild(registry);

      coefficientOfFriction = new DoubleYoVariable(name + "CoefficientOfFriction", registry);
      handAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), elevator.getBodyFixedFrame(), endEffector.getBodyFixedFrame());

      elevatorFrame = elevator.getBodyFixedFrame();
      handFrame = endEffector.getBodyFixedFrame();

      SideDependentList<ContactablePlaneBody> contactableHands = controllerToolbox.getContactableHands();
      if (contactableHands != null)
      {
         handPalm = contactableHands.get(robotSide);
         contactNormal = new FrameVector();
         contactNormal.setToNaN();
         controllerToolbox.setPlaneContactStateFree(handPalm);
      }
      else
      {
         handPalm = null;
         contactNormal = null;
      }
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormalVector(FrameVector normal)
   {
      double length = normal.length();
      if (length < 1e-7)
      {
         contactNormal.setToNaN();
      }
      else
      {
         contactNormal.setIncludingFrame(normal);
      }
   }

   public void getContactNormalVector(FrameVector normalToPack)
   {
      normalToPack.setIncludingFrame(contactNormal);
   }

   @Override
   public void doAction()
   {
      handAcceleration.setToZero(handFrame, elevatorFrame, handFrame);
      spatialAccelerationCommand.setSpatialAcceleration(handAcceleration);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (handPalm == null)
      {
         System.out.println("No YoPlaneContactState for the hands in the HighLevelHumanoidControllerToolbox, leaving the hand load bearing state.");

         return;
      }

      if (contactNormal.containsNaN())
         contactNormal.setIncludingFrame(handPalm.getSoleFrame(), 0.0, 0.0, 1.0);

      contactNormal.changeFrame(ReferenceFrame.getWorldFrame());
      controllerToolbox.setPlaneContactStateFullyConstrained(handPalm, coefficientOfFriction.getDoubleValue(), contactNormal);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (handPalm == null)
         return;

      controllerToolbox.setPlaneContactStateFree(handPalm);
      contactNormal.setToNaN();
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   @Override
   public SpatialAccelerationCommand getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }
}
