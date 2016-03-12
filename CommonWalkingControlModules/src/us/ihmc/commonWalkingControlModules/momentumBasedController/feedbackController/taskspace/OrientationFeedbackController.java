package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import javax.vecmath.AxisAngle4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class OrientationFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final BooleanYoVariable isEnabled;

   private final YoFrameQuaternion yoDesiredOrientation;
   private final YoFrameQuaternion yoCurrentOrientation;

   private final YoFrameVector yoDesiredRotationVector;
   private final YoFrameVector yoCurrentRotationVector;

   private final YoFrameVector yoDesiredAngularVelocity;
   private final YoFrameVector yoCurrentAngularVelocity;

   private final YoFrameVector yoFeedForwardAngularAcceleration;
   private final YoFrameVector yoDesiredAngularAcceleration;

   private final DoubleYoVariable weightForSolver;

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector desiredAngularAcceleration = new FrameVector();

   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();

   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, SpatialAccelerationVector.SIZE);
   private final SpatialAccelerationCommand output = new SpatialAccelerationCommand();

   private final RigidBodyOrientationControlModule accelerationControlModule;

   private RigidBody base;
   private ReferenceFrame baseFrame;

   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;


   public OrientationFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "OrientationFBController");
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      double dt = toolbox.getControlDT();
      accelerationControlModule = new RigidBodyOrientationControlModule(endEffectorName, endEffector, twistCalculator, dt, registry);

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new BooleanYoVariable(endEffectorName + "IsOrientationFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredOrientation = new YoFrameQuaternion(endEffectorName + "DesiredOrientation", worldFrame, registry);
      yoCurrentOrientation = new YoFrameQuaternion(endEffectorName + "CurrentOrientation", worldFrame, registry);

      yoDesiredRotationVector = new YoFrameVector(endEffectorName + "DesiredRotationVector", worldFrame, registry);
      yoCurrentRotationVector = new YoFrameVector(endEffectorName + "CurrentRotationVector", worldFrame, registry);

      yoDesiredAngularVelocity = new YoFrameVector(endEffectorName + "DesiredAngularVelocity", worldFrame, registry);
      yoCurrentAngularVelocity = new YoFrameVector(endEffectorName + "CurrentAngularVelocity", worldFrame, registry);

      yoFeedForwardAngularAcceleration = new YoFrameVector(endEffectorName + "FeedForwardAngularAcceleration", worldFrame, registry);
      yoDesiredAngularAcceleration = new YoFrameVector(endEffectorName + "DesiredAngularAcceleration", worldFrame, registry);

      weightForSolver = new DoubleYoVariable(endEffectorName + "OrientationWeight", registry);
      weightForSolver.set(Double.POSITIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(OrientationFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();
      baseFrame = base.getBodyFixedFrame();

      output.set(base, endEffector);

      setGains(command.getGains());
      setWeightForSolver(command.getWeightForSolver());
      selectionMatrix.set(command.getSelectionMatrix());
      command.getIncludingFrame(tempOrientation, tempAngularVelocity, feedForwardAngularAcceleration);
      yoDesiredOrientation.setAndMatchFrame(tempOrientation);
      yoDesiredAngularVelocity.setAndMatchFrame(tempAngularVelocity);
      yoFeedForwardAngularAcceleration.setAndMatchFrame(feedForwardAngularAcceleration);
   }

   public void setGains(OrientationPIDGainsInterface gains)
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

      yoDesiredOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      yoDesiredAngularVelocity.getFrameTupleIncludingFrame(tempAngularVelocity);
      yoFeedForwardAngularAcceleration.getFrameTupleIncludingFrame(feedForwardAngularAcceleration);

      accelerationControlModule.compute(desiredAngularAcceleration, tempOrientation, tempAngularVelocity, feedForwardAngularAcceleration, base);

      yoDesiredAngularAcceleration.setAndMatchFrame(desiredAngularAcceleration);

      tempOrientation.setToZero(endEffectorFrame);
      yoCurrentOrientation.setAndMatchFrame(tempOrientation);

      accelerationControlModule.getEndEffectorCurrentAngularVelocity(tempAngularVelocity);
      yoCurrentAngularVelocity.setAndMatchFrame(tempAngularVelocity);

      yoDesiredOrientation.get(tempAxisAngle);
      yoDesiredRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoDesiredRotationVector.scale(tempAxisAngle.getAngle());

      yoCurrentOrientation.get(tempAxisAngle);
      yoCurrentRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoCurrentRotationVector.scale(tempAxisAngle.getAngle());

      output.setAngularAcceleration(endEffectorFrame, baseFrame, desiredAngularAcceleration);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public SpatialAccelerationCommand getOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return output;
   }
}
