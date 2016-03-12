package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.BodyFixedPointLinearAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class PointFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final BooleanYoVariable isEnabled;

   private final YoFramePoint yoDesiredPosition;
   private final YoFramePoint yoCurrentPosition;

   private final YoFrameVector yoDesiredLinearVelocity;
   private final YoFrameVector yoCurrentLinearVelocity;

   private final YoFrameVector yoFeedForwardLinearAcceleration;
   private final YoFrameVector yoDesiredLinearAcceleration;

   private final DoubleYoVariable weightForSolver;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector desiredLinearAcceleration = new FrameVector();

   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, SpatialAccelerationVector.SIZE);
   private final PointAccelerationCommand output = new PointAccelerationCommand();

   private final BodyFixedPointLinearAccelerationControlModule accelerationControlModule;

   private RigidBody base;

   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;

   public PointFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "PointFBController");
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      double dt = toolbox.getControlDT();
      accelerationControlModule = new BodyFixedPointLinearAccelerationControlModule(endEffectorName, twistCalculator, endEffector, dt, registry);

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new BooleanYoVariable(endEffectorName + "isPointFBControllerEnabled", registry);
      isEnabled.set(false);
      
      yoDesiredPosition = new YoFramePoint(endEffectorName + "DesiredPosition", worldFrame, registry);
      yoCurrentPosition = new YoFramePoint(endEffectorName + "CurrentPosition", worldFrame, registry);
      
      yoDesiredLinearVelocity = new YoFrameVector(endEffectorName + "DesiredLinearVelocity", worldFrame, registry);
      yoCurrentLinearVelocity = new YoFrameVector(endEffectorName + "CurrentLinearVelocity", worldFrame, registry);
      
      yoFeedForwardLinearAcceleration = new YoFrameVector(endEffectorName + "FeedForwardLinearAcceleration", worldFrame, registry);
      yoDesiredLinearAcceleration = new YoFrameVector(endEffectorName + "DesiredLinearAcceleration", worldFrame, registry);

      weightForSolver = new DoubleYoVariable(endEffectorName + "PointWeight", registry);
      weightForSolver.set(Double.POSITIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(PointFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();

      output.setBase(base);
      output.setEndEffector(endEffector);

      setGains(command.getGains());
      setWeightForSolver(command.getWeightForSolver());
      selectionMatrix.set(command.getSelectionMatrix());

      command.getBodyFixedPointIncludingFrame(tempPosition);
      accelerationControlModule.setPointToControl(tempPosition);
      
      command.getIncludingFrame(tempPosition, tempLinearVelocity, feedForwardLinearAcceleration);
      yoDesiredPosition.setAndMatchFrame(tempPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      yoFeedForwardLinearAcceleration.setAndMatchFrame(feedForwardLinearAcceleration);
   }

   public void setGains(PositionPIDGainsInterface gains)
   {
      accelerationControlModule.setGains(gains);
   }

   public void setWeightForSolver(double weightForSolver)
   {
      this.weightForSolver.set(weightForSolver);
      output.setWeight(weightForSolver);
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   {
      accelerationControlModule.reset();
   }

   @Override
   public void compute()
   {
      if (!isEnabled())
         return;

      yoDesiredPosition.getFrameTupleIncludingFrame(tempPosition);
      yoDesiredLinearVelocity.getFrameTupleIncludingFrame(tempLinearVelocity);
      yoFeedForwardLinearAcceleration.getFrameTupleIncludingFrame(feedForwardLinearAcceleration);

      accelerationControlModule.compute(desiredLinearAcceleration, tempPosition, tempLinearVelocity, feedForwardLinearAcceleration, base);

      updatePositionVisualization();

      accelerationControlModule.getBodyFixedPoint(tempPosition);

      output.set(tempPosition, desiredLinearAcceleration);
   }

   private void updatePositionVisualization()
   {
      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

      tempPosition.setToZero(endEffectorFrame);
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      accelerationControlModule.getBodyFixedPointCurrentLinearVelocity(tempLinearVelocity);
      yoCurrentLinearVelocity.setAndMatchFrame(tempLinearVelocity);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public PointAccelerationCommand getOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return output;
   }
}
