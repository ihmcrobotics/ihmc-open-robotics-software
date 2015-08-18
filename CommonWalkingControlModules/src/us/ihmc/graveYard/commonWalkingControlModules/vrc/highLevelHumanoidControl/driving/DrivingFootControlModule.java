package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.communication.packets.LowLevelDrivingAction;
import us.ihmc.communication.packets.LowLevelDrivingStatus;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.trajectories.providers.AverageVelocityTrajectoryTimeProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.yoUtilities.controllers.AxisAngleOrientationController;
import us.ihmc.yoUtilities.controllers.EuclideanPositionController;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoPositionProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;


/**
 * @author twan
 *         Date: 6/6/13
 */
public class DrivingFootControlModule
{
   private final YoVariableRegistry registry;
   private final int footJacobianId;
   private final FramePoint toePoint;
   private final EuclideanPositionController toePointPositionController;
   private final MomentumBasedController momentumBasedController;

   private final GlobalDataProducer statusProducer;

   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredVelocity = new FrameVector();
   private final FrameVector feedForward = new FrameVector();

   private final YoFramePoint desiredPositionYoFramePoint;
   private final YoFrameVector desiredVelocityYoFrameVector;

   private final FrameVector currentVelocity = new FrameVector();
   private final FrameVector currentAngularVelocity = new FrameVector();

   private final PositionTrajectoryGenerator positionTrajectoryGenerator;
   private final YoFramePoint initialToePointPosition;
   private final YoFramePoint finalToePointPosition;
   private final DoubleYoVariable footPitch;
   private final DoubleYoVariable footRoll;

   private final DoubleYoVariable trajectoryInitializationTime;
   private final DoubleYoVariable time;

   private final AxisAngleOrientationController orientationController;
   private final DenseMatrix64F footOrientationSelectionMatrix;
   private final DenseMatrix64F footOrientationNullspaceMultipliers = new DenseMatrix64F(0, 1);

   private final SpatialAccelerationVector footOrientationControlSpatialAcceleration;
   private final TaskspaceConstraintData footOrientationTaskspaceConstraintData = new TaskspaceConstraintData();

   private final FrameOrientation desiredOrientation;
   private final FrameVector desiredAngularVelocity;
   private final FrameVector feedForwardAngularAcceleration;

   private final TwistCalculator twistCalculator;
   private final Twist currentTwist = new Twist();
   private final FramePoint toePointInBase = new FramePoint();
   private final ReferenceFrame toePointFrame;

   private final DrivingReferenceFrames drivingReferenceFrames;

   private final RigidBody foot;

   private final TaskExecutor taskExecutor = new TaskExecutor();
   private final double pedalY = 0.0;

   private final FrameVector pedalForceToCompensateFor = new FrameVector();
   private final IntegerYoVariable nFootTasksRemaining;
   private final YoVariableDoubleProvider averageVelocityProvider;
   private final List<YoGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();
   
   private final RigidBody elevator;


   public DrivingFootControlModule(FullRobotModel fullRobotModel, ContactablePlaneBody contactablePlaneFoot, MomentumBasedController momentumBasedController,
                                   DrivingReferenceFrames drivingReferenceFrames, double dt, DoubleYoVariable yoTime, TwistCalculator twistCalculator,
                                   YoVariableRegistry parentRegistry, GlobalDataProducer statusProducer,
                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.statusProducer = statusProducer;
      this.foot = contactablePlaneFoot.getRigidBody();
      registry = new YoVariableRegistry(foot.getName() + getClass().getSimpleName());
      elevator = fullRobotModel.getElevator();
      footJacobianId = momentumBasedController.getOrCreateGeometricJacobian(elevator, foot, elevator.getBodyFixedFrame());
      toePoint = getCenterToePoint(contactablePlaneFoot);
//      toePoint = getLeftFrontToePoint(contactablePlaneFoot);
      String toePointName = foot.getName() + "ToePoint";
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationAndIdentityRotation(toePoint.getVectorCopy());
      toePointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(toePointName, toePoint.getReferenceFrame(), transform);
      this.drivingReferenceFrames = drivingReferenceFrames;
      toePointPositionController = new EuclideanPositionController(toePointName, toePointFrame, dt, registry);
      this.momentumBasedController = momentumBasedController;
      this.time = yoTime;
      trajectoryInitializationTime = new DoubleYoVariable(toePointName + "InitializationTime", registry);
      footPitch = new DoubleYoVariable("footPitch", registry);
      footRoll = new DoubleYoVariable("footRoll", registry);

      desiredPositionYoFramePoint = new YoFramePoint("desiredFootPointPosition", ReferenceFrame.getWorldFrame(), registry);
      desiredVelocityYoFrameVector = new YoFrameVector("desiredFootPointVelocity", ReferenceFrame.getWorldFrame(), registry);

      ReferenceFrame vehicleFrame = drivingReferenceFrames.getVehicleFrame();

//      desiredOrientation = new FrameOrientation(drivingReferenceFrames.getObjectFrame(VehicleObject.GAS_PEDAL));    // should be the same for brake pedal

      desiredOrientation = new FrameOrientation(vehicleFrame);

      desiredAngularVelocity = new FrameVector(vehicleFrame);
      feedForwardAngularAcceleration = new FrameVector(vehicleFrame);

      initialToePointPosition = new YoFramePoint(toePointName + "Initial", vehicleFrame, registry);
      finalToePointPosition = new YoFramePoint(toePointName + "Final", vehicleFrame, registry);

      PositionProvider initialPositionProvider = new YoPositionProvider(initialToePointPosition);
      PositionProvider finalPositionProvider = new YoPositionProvider(finalToePointPosition);
      averageVelocityProvider = new YoVariableDoubleProvider("drivingFootAverageVelocity", registry);
      DoubleProvider trajectoryTimeProvider = new AverageVelocityTrajectoryTimeProvider(initialPositionProvider, finalPositionProvider,
                                                 averageVelocityProvider, 0.1);
      this.positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(toePointName + "Trajectory", vehicleFrame, trajectoryTimeProvider,
              initialPositionProvider, finalPositionProvider, registry);

      double kP = 300.0;
      double dampingRatio = 1.0;
      double kD = GainCalculator.computeDerivativeGain(kP, dampingRatio);
      toePointPositionController.setProportionalGains(kP, kP, kP);
      toePointPositionController.setDerivativeGains(kD, kD, kD);


      orientationController = new AxisAngleOrientationController(foot.getName() + "PD", foot.getBodyFixedFrame(), dt, registry);
      double kPOrientationYZ = 50.0;
      double kDOrientationYZ = GainCalculator.computeDerivativeGain(kPOrientationYZ, dampingRatio);

      double kPOrientationX = 50.0;
      double kDOrientationX = GainCalculator.computeDerivativeGain(kPOrientationX, dampingRatio);

      orientationController.setProportionalGains(kPOrientationX, kPOrientationYZ, kPOrientationYZ);
      orientationController.setDerivativeGains(kDOrientationX, kDOrientationYZ, kDOrientationYZ);

      nFootTasksRemaining = new IntegerYoVariable("nFootTasksRemaining", registry);

      footOrientationSelectionMatrix = new DenseMatrix64F(3, SpatialMotionVector.SIZE);
      footOrientationSelectionMatrix.zero();
      footOrientationSelectionMatrix.set(0, 0, 1.0);
      footOrientationSelectionMatrix.set(1, 1, 1.0);
      footOrientationSelectionMatrix.set(2, 2, 1.0);

      footPitch.set(0.3);
      footRoll.set(0.0);

      footOrientationControlSpatialAcceleration = new SpatialAccelerationVector();

      this.twistCalculator = twistCalculator;

//      taskExecutor.setPrintDebugStatements(true);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicReferenceFrame toePointFrameViz = new YoGraphicReferenceFrame(toePointFrame, registry, 0.1);
         YoGraphicsList list = new YoGraphicsList("drivingFootControlModule");
         dynamicGraphicReferenceFrames.add(toePointFrameViz);
         list.add(toePointFrameViz);
         yoGraphicsListRegistry.registerYoGraphicsList(list);
      }

   }

// public void addTaskCompletedNotifier(LowLevelDrivingAction action)
// {
//    if(statusProducer != null)
//    {
//       taskExecutor.submit(new NotifyStatusListenerTask<LowLevelDrivingStatus>(statusProducer, new LowLevelDrivingStatus(action, true)));
//    }
// }

   public void reset()
   {
      orientationController.reset();
      toePointPositionController.reset();
   }
   public void holdPosition()
   {
      FramePoint target = new FramePoint(toePoint);
      pedalForceToCompensateFor.setToZero(toePointFrame);
      double averageVelocity = 1.0;    // arbitrary positive number
      moveToPosition(target, pedalForceToCompensateFor, averageVelocity, false, null);
   }

   public void moveToPositionInGasPedalFrame(double z, double averageVelocity, boolean notifyIfDone)
   {
      FramePoint target = new FramePoint(drivingReferenceFrames.getObjectFrame(VehicleObject.GAS_PEDAL), 0.0, pedalY, z);

      pedalForceToCompensateFor.setToZero(drivingReferenceFrames.getObjectFrame(VehicleObject.GAS_PEDAL));
      pedalForceToCompensateFor.setZ(computePedalForceToCompensateFor(z));

      moveToPosition(target, pedalForceToCompensateFor, averageVelocity, notifyIfDone, LowLevelDrivingAction.GASPEDAL);
   }

   public void moveToPositionInBrakePedalFrame(double z, double averageVelocity, boolean notifyIfDone)
   {
      FramePoint target = new FramePoint(drivingReferenceFrames.getObjectFrame(VehicleObject.BRAKE_PEDAL), 0.0, pedalY, z);

      pedalForceToCompensateFor.setToZero(drivingReferenceFrames.getObjectFrame(VehicleObject.BRAKE_PEDAL));
      pedalForceToCompensateFor.setZ(computePedalForceToCompensateFor(z));

      moveToPosition(target, pedalForceToCompensateFor, averageVelocity, notifyIfDone, LowLevelDrivingAction.FOOTBRAKE);
   }

   private double computePedalForceToCompensateFor(double z)
   {
      return MathTools.clipToMinMax(-800.0 * z, 0.0, 10.0);    // from DRCVehiclePlugin.cc
   }

   public void initialize()
   {
      FramePoint toePointInFrame = new FramePoint(toePoint);
      toePointInFrame.changeFrame(initialToePointPosition.getReferenceFrame());
      initialToePointPosition.set(toePointInFrame);
   }

   public void doControl()
   {
      updateVisualizers();
      taskExecutor.doControl();
      updateCurrentVelocity();
      doToePositionControl();
      doFootOrientationControl();
   }

   private void updateVisualizers()
   {
      for (YoGraphicReferenceFrame dynamicGraphicReferenceFrame : dynamicGraphicReferenceFrames)
      {
         dynamicGraphicReferenceFrame.update();
      }
   }

   private void moveToPosition(FramePoint target, FrameVector forceToCompensateFor, double averageVelocity, boolean notifyIfDone, LowLevelDrivingAction action)
   {
      Task task = new FootControlTask(target, forceToCompensateFor, averageVelocity, notifyIfDone, action);
      nFootTasksRemaining.increment();
      taskExecutor.submit(task);
   }

   private void doToePositionControl()
   {
      positionTrajectoryGenerator.compute(time.getDoubleValue() - trajectoryInitializationTime.getDoubleValue());
      positionTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, feedForward);

      FrameVector output = new FrameVector(toePointFrame);
      toePointPositionController.compute(output, desiredPosition, desiredVelocity, currentVelocity, feedForward);
      momentumBasedController.setDesiredPointAcceleration(footJacobianId, toePoint, output);

      desiredPosition.changeFrame(desiredPositionYoFramePoint.getReferenceFrame());
      desiredPositionYoFramePoint.set(desiredPosition);

      desiredVelocity.changeFrame(desiredVelocityYoFrameVector.getReferenceFrame());
      desiredVelocityYoFrameVector.set(desiredVelocity);
   }

   private void doFootOrientationControl()
   {
//      Matrix3d rotationMatrix = new Matrix3d();
//      rotationMatrix.setColumn(0, 0.0, 0.0, -1.0);
//      rotationMatrix.setColumn(1, -1.0, 0.0, 0.0);
//      rotationMatrix.setColumn(2, 0.0, 1.0, 0.0);
//
//      Matrix3d postRotation = new Matrix3d();
//      RotationFunctions.setYawPitchRoll(postRotation, 0.0, 0.0, 0.0);
//      rotationMatrix.mul(postRotation);
//
//      desiredOrientation.set(drivingReferenceFrames.getObjectFrame(VehicleObject.GAS_PEDAL), rotationMatrix);

      desiredOrientation.setToZero(drivingReferenceFrames.getVehicleFrame());
      desiredOrientation.setYawPitchRoll(0.0, footPitch.getDoubleValue(), footRoll.getDoubleValue());
      desiredAngularVelocity.setToZero(toePointFrame);
      feedForwardAngularAcceleration.setToZero(toePointFrame);

      FrameVector output = new FrameVector(toePointFrame);
      orientationController.compute(output, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);

      footOrientationControlSpatialAcceleration.setToZero(foot.getBodyFixedFrame(), elevator.getBodyFixedFrame(), foot.getBodyFixedFrame());
      footOrientationControlSpatialAcceleration.setAngularPart(output.getVector());
      footOrientationTaskspaceConstraintData.set(elevator, foot);
      footOrientationTaskspaceConstraintData.set(footOrientationControlSpatialAcceleration, footOrientationNullspaceMultipliers,
              footOrientationSelectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(footJacobianId, footOrientationTaskspaceConstraintData);
   }

   private void updateCurrentVelocity()
   {
      twistCalculator.packRelativeTwist(currentTwist, elevator, foot);
      currentTwist.packAngularPart(currentAngularVelocity);
      currentTwist.changeFrame(elevator.getBodyFixedFrame());
      toePointInBase.setIncludingFrame(toePoint);
      toePointInBase.changeFrame(elevator.getBodyFixedFrame());
      currentTwist.packLinearVelocityOfPointFixedInBodyFrame(currentVelocity, toePointInBase);
   }

   private static FramePoint getCenterToePoint(ContactablePlaneBody foot)
   {
      FrameVector forward = new FrameVector(foot.getSoleFrame(), 1.0, 0.0, 0.0);
      int nToePoints = 2;
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(foot.getContactPointsCopy(), forward, nToePoints);
      FramePoint centerToePoint = FramePoint.average(toePoints);

      return centerToePoint;
   }

   private class FootControlTask implements Task
   {
      private final FramePoint targetPosition;
      private final FrameVector forceToCompensate;
      private final FrameVector tempVector = new FrameVector();
      private final Wrench wrench = new Wrench();
      private final LowLevelDrivingAction action;
      private final double averageVelocity;
      private boolean notifyIfDone;

      public FootControlTask(FramePoint targetPosition, FrameVector forceToCompensate, double averageVelocity, boolean notifyIfDone,
                             LowLevelDrivingAction action)
      {
         this.targetPosition = targetPosition;
         this.forceToCompensate = new FrameVector(forceToCompensate);
         this.averageVelocity = averageVelocity;
         this.action = action;
         this.notifyIfDone = notifyIfDone;
      }

      public void doTransitionIntoAction()
      {
         averageVelocityProvider.set(averageVelocity);
         FramePoint targetPositionInFrame = new FramePoint(targetPosition);
         targetPositionInFrame.changeFrame(finalToePointPosition.getReferenceFrame());
         finalToePointPosition.set(targetPositionInFrame);
         positionTrajectoryGenerator.initialize();
         trajectoryInitializationTime.set(time.getDoubleValue());
         nFootTasksRemaining.decrement();
      }

      public void doAction()
      {
         wrench.setToZero(foot.getBodyFixedFrame(), toePointFrame);
         tempVector.setIncludingFrame(forceToCompensate);
         tempVector.changeFrame(toePointFrame);
         wrench.setLinearPart(tempVector.getVector());
         wrench.changeFrame(foot.getBodyFixedFrame());
         momentumBasedController.setExternalWrenchToCompensateFor(foot, wrench);
      }

      public void doTransitionOutOfAction()
      {
         FramePoint currentDesired = new FramePoint();
         positionTrajectoryGenerator.get(currentDesired);
         currentDesired.changeFrame(initialToePointPosition.getReferenceFrame());
         initialToePointPosition.set(currentDesired);
      }

      public boolean isDone()
      {
         // this is so that the TaskExecutor keeps doing our doAction and doesn't transition into a NullTask, where it doesn't set the external wrench to compensate for.
         boolean newTasksAreAvailable = nFootTasksRemaining.getIntegerValue() > 0;

         if (positionTrajectoryGenerator.isDone() && notifyIfDone)
         {
            statusProducer.queueDataToSend(new LowLevelDrivingStatus(action, true));
            notifyIfDone = false;
         }

         return positionTrajectoryGenerator.isDone() && newTasksAreAvailable;
      }

      @Override
      public String toString()
      {
         return getClass().getSimpleName() + ": targetPosition: " + targetPosition + ", forceToCompensate: " + forceToCompensate;
      }

      @Override
      public void pause()
      {
         // TODO Auto-generated method stub
         
      }

      @Override
      public void resume()
      {
         // TODO Auto-generated method stub
         
      }

      @Override
      public void stop()
      {
         // TODO Auto-generated method stub
         
      }
   }
}
