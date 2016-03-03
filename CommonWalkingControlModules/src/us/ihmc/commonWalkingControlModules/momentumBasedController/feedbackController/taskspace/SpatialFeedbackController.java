package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import javax.vecmath.AxisAngle4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.BodyFixedPointSpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.WholeBodyControlCoreToolbox;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class SpatialFeedbackController implements FeedbackControllerInterface
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

   private final YoFrameQuaternion yoDesiredOrientation;
   private final YoFrameQuaternion yoCurrentOrientation;

   private final YoFrameVector yoDesiredRotationVector;
   private final YoFrameVector yoCurrentRotationVector;

   private final YoFrameVector yoDesiredAngularVelocity;
   private final YoFrameVector yoCurrentAngularVelocity;

   private final YoFrameVector yoFeedForwardAngularAcceleration;
   private final YoFrameVector yoDesiredAngularAcceleration;

   private final DoubleYoVariable weightForSolver;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector desiredLinearAcceleration = new FrameVector();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector desiredAngularAcceleration = new FrameVector();

   private final SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector();

   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();

   private long jacobianForSingularityEscapeId = NameBasedHashCodeTools.NULL_HASHCODE;
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, SpatialAccelerationVector.SIZE);
   private final SpatialAccelerationCommand output = new SpatialAccelerationCommand();

   private final BodyFixedPointSpatialAccelerationControlModule accelerationControlModule;

   private RigidBody base;

   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;

   public SpatialFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "SpatialFBController");
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      double dt = toolbox.getControlDT();
      accelerationControlModule = new BodyFixedPointSpatialAccelerationControlModule(endEffectorName, twistCalculator, endEffector, dt, registry);

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new BooleanYoVariable(endEffectorName + "isSpatialFBControllerEnabled", registry);
      isEnabled.set(false);
      
      yoDesiredPosition = new YoFramePoint(endEffectorName + "DesiredPosition", worldFrame, registry);
      yoCurrentPosition = new YoFramePoint(endEffectorName + "CurrentPosition", worldFrame, registry);
      
      yoDesiredLinearVelocity = new YoFrameVector(endEffectorName + "DesiredLinearVelocity", worldFrame, registry);
      yoCurrentLinearVelocity = new YoFrameVector(endEffectorName + "CurrentLinearVelocity", worldFrame, registry);
      
      yoFeedForwardLinearAcceleration = new YoFrameVector(endEffectorName + "FeedForwardLinearAcceleration", worldFrame, registry);
      yoDesiredLinearAcceleration = new YoFrameVector(endEffectorName + "DesiredLinearAcceleration", worldFrame, registry);

      yoDesiredOrientation = new YoFrameQuaternion(endEffectorName + "DesiredOrientation", worldFrame, registry);
      yoCurrentOrientation = new YoFrameQuaternion(endEffectorName + "CurrentOrientation", worldFrame, registry);

      yoDesiredRotationVector = new YoFrameVector(endEffectorName + "DesiredRotationVector", worldFrame, registry);
      yoCurrentRotationVector = new YoFrameVector(endEffectorName + "CurrentRotationVector", worldFrame, registry);

      yoDesiredAngularVelocity = new YoFrameVector(endEffectorName + "DesiredAngularVelocity", worldFrame, registry);
      yoCurrentAngularVelocity = new YoFrameVector(endEffectorName + "CurrentAngularVelocity", worldFrame, registry);

      yoFeedForwardAngularAcceleration = new YoFrameVector(endEffectorName + "FeedForwardAngularAcceleration", worldFrame, registry);
      yoDesiredAngularAcceleration = new YoFrameVector(endEffectorName + "DesiredAngularAcceleration", worldFrame, registry);

      weightForSolver = new DoubleYoVariable(endEffectorName + "SpatialWeight", registry);
      weightForSolver.set(Double.POSITIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(SpatialFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();

      output.set(base, endEffector);
      jacobianForSingularityEscapeId = command.getJacobianForNullspaceId();
      nullspaceMultipliers.set(command.getNullspaceMultipliers());

      setGains(command.getGains());
      setWeightForSolver(command.getWeightForSolver());
      selectionMatrix.set(command.getSelectionMatrix());

      command.getControlFramePoseIncludingFrame(tempPosition, tempOrientation);
      accelerationControlModule.setBodyFixedControlFrame(tempPosition, tempOrientation);
      
      command.getIncludingFrame(tempPosition, tempLinearVelocity, feedForwardLinearAcceleration);
      yoDesiredPosition.setAndMatchFrame(tempPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      yoFeedForwardLinearAcceleration.setAndMatchFrame(feedForwardLinearAcceleration);

      command.getIncludingFrame(tempOrientation, tempAngularVelocity, feedForwardAngularAcceleration);
      yoDesiredOrientation.setAndMatchFrame(tempOrientation);
      yoDesiredAngularVelocity.setAndMatchFrame(tempAngularVelocity);
      yoFeedForwardAngularAcceleration.setAndMatchFrame(feedForwardAngularAcceleration);
   }

   public void setGains(SE3PIDGainsInterface gains)
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

      yoDesiredOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      yoDesiredAngularVelocity.getFrameTupleIncludingFrame(tempAngularVelocity);
      yoFeedForwardAngularAcceleration.getFrameTupleIncludingFrame(feedForwardAngularAcceleration);

      accelerationControlModule.doPositionControl(tempPosition, tempOrientation, tempLinearVelocity, tempAngularVelocity, feedForwardLinearAcceleration, feedForwardAngularAcceleration, base);
      accelerationControlModule.getAcceleration(desiredSpatialAcceleration);
      desiredSpatialAcceleration.changeBodyFrameNoRelativeAcceleration(endEffectorFrame);
      desiredSpatialAcceleration.changeFrameNoRelativeMotion(endEffectorFrame);

      updatePositionVisualization();
      updateOrientationVisualization();

      output.set(desiredSpatialAcceleration, nullspaceMultipliers, selectionMatrix);
      output.setJacobianForNullspaceId(jacobianForSingularityEscapeId);
   }

   private void updatePositionVisualization()
   {
      desiredSpatialAcceleration.getAngularPart(desiredLinearAcceleration);
      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

      tempPosition.setToZero(accelerationControlModule.getTrackingFrame());
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      accelerationControlModule.getEndEffectorCurrentLinearVelocity(tempLinearVelocity);
      yoCurrentLinearVelocity.setAndMatchFrame(tempLinearVelocity);
   }

   private void updateOrientationVisualization()
   {
      desiredSpatialAcceleration.getAngularPart(desiredAngularAcceleration);
      yoDesiredAngularAcceleration.setAndMatchFrame(desiredAngularAcceleration);

      tempOrientation.setToZero(accelerationControlModule.getTrackingFrame());
      yoCurrentOrientation.setAndMatchFrame(tempOrientation);

      accelerationControlModule.getEndEffectorCurrentAngularVelocity(tempAngularVelocity);
      yoCurrentAngularVelocity.setAndMatchFrame(tempAngularVelocity);

      yoDesiredOrientation.get(tempAxisAngle);
      yoDesiredRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoDesiredRotationVector.scale(tempAxisAngle.getAngle());

      yoCurrentOrientation.get(tempAxisAngle);
      yoCurrentRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoCurrentRotationVector.scale(tempAxisAngle.getAngle());
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
