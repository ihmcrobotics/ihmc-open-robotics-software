package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.manipulationBehaviors;

import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformReferenceFrame;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.controllers.EuclideanPositionController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.trajectories.CirclePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;


/**
 * @author twan
 *         Date: 6/12/13
 */
public class PointPositionRotateSteeringWheelBehavior
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoVariableDoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider("rotateSteeringWheelTrajectoryTime", registry);
   private final YoVariableDoubleProvider desiredRotationAngleProvider = new YoVariableDoubleProvider("rotateSteeringWheelDeltaAngle", registry);

//   private final CylindricalCoordinatesPositionController positionController;
   private final EuclideanPositionController positionController;

   private final double averageAngularVelocity = 0.7;

   private final HandControlModule individualHandControlModule;
   private final ReferenceFrame creepyGripHandPositionControlFrame;

   private final CirclePositionTrajectoryGenerator trajectoryGenerator;
   private final RobotSide robotSide;
   private final int jacobianId;

   private final ReferenceFrame steeringWheelFrame;
   private final TransformReferenceFrame xTangentialFrame;
   private final YoGraphicReferenceFrame xTangentialFrameViz;
   private final MomentumBasedController momentumBasedController;

   private final RigidBody hand;
   private final RigidBody elevator;

   public PointPositionRotateSteeringWheelBehavior(double dt, RobotSide robotSide, HandControlModule individualHandControlModule,
                                                   ReferenceFrame creepyGripHandPositionControlFrame, FullRobotModel fullRobotModel,
                                                   ReferenceFrame steeringWheelFrame, MomentumBasedController momentumBasedController,
                                                   YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.individualHandControlModule = individualHandControlModule;
      this.creepyGripHandPositionControlFrame = creepyGripHandPositionControlFrame;
      this.robotSide = robotSide;

      hand = fullRobotModel.getHand(robotSide);
      elevator = fullRobotModel.getElevator();
      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(elevator, hand, elevator.getBodyFixedFrame());

      positionController = new EuclideanPositionController("pointRotateSteeringWheel", creepyGripHandPositionControlFrame, dt, registry);
      positionController.setProportionalGains(100.0, 100.0, 100.0);
      positionController.setDerivativeGains(20.0, 20.0, 20.0);

      this.steeringWheelFrame = steeringWheelFrame;

      this.xTangentialFrame = new TransformReferenceFrame("rotateSteeringWheelGainFrame", positionController.getBodyFrame());

      this.momentumBasedController = momentumBasedController;

//      SE3ConfigurationProvider currentDesiredConfigurationProvider =
//         individualHandControlModule.getCurrentDesiredConfigurationProvider(creepyGripHandPositionControlFrame);
      PositionProvider initialPositionProvider = null; // currentDesiredConfigurationProvider;
      trajectoryGenerator = new CirclePositionTrajectoryGenerator("rotateSteeringWheelTrajectory", steeringWheelFrame, trajectoryTimeProvider,
              initialPositionProvider, registry, desiredRotationAngleProvider);

      if (yoGraphicsListRegistry != null)
      {
         xTangentialFrameViz = new YoGraphicReferenceFrame(xTangentialFrame, registry, 0.1);
         yoGraphicsListRegistry.registerYoGraphic("rotateSteeringWheelBehavior", xTangentialFrameViz);
      }
      else
      {
         xTangentialFrameViz = null;
      }

      parentRegistry.addChild(registry);
   }

   public Task getTask(double relativeRotationAngle, double absoluteRotationAngle)
   {
      return new RotateTask(relativeRotationAngle, absoluteRotationAngle);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   private class RotateTask implements Task
   {
      private final double relativeRotationAngle;
      private final double absoluteRotationAngle;

      private final FramePoint tempPoint = new FramePoint(ReferenceFrame.getWorldFrame());

      private final FrameVector x = new FrameVector();    // direction of motion
      private final FrameVector y = new FrameVector();    // radial direction
      private final FrameVector z = new FrameVector();    // z direction

      private final Matrix3d tempMatrix = new Matrix3d();
      private final FrameOrientation rotationFromGainOrientationToBody = new FrameOrientation(steeringWheelFrame);
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final FrameVector force = new FrameVector();
      private final Wrench wrench = new Wrench();

      private RotateTask(double relativeRotationAngle, double absoluteRotationAngle)
      {
         this.relativeRotationAngle = relativeRotationAngle;
         this.absoluteRotationAngle = absoluteRotationAngle;
      }

      public void doTransitionIntoAction()
      {
         tempPoint.setToZero(creepyGripHandPositionControlFrame);

         double minTrajectoryTime = 0.1;
         double trajectoryTime = Math.max(minTrajectoryTime, Math.abs(relativeRotationAngle / averageAngularVelocity));
         trajectoryTimeProvider.set(trajectoryTime);
         desiredRotationAngleProvider.set(relativeRotationAngle);
         // TODO Reimplement the following
//         individualHandControlModule.executePointPositionTrajectory(trajectoryGenerator, positionController, tempPoint, jacobianId);
      }

      public void doAction()
      {
         updateXTangentialFrame();
         setExternalWrench();
      }

      private void setExternalWrench()
      {
         double steeringWheelExternalTorqueMagnitude = 1.0;    // from DRCVehiclePlugin.cc, handWheelForce (100 Nm / rad P control, clipped to 1)
         double steeringWheelExternalTorque = Math.signum(absoluteRotationAngle) * steeringWheelExternalTorqueMagnitude;

         tempPoint.setToZero(xTangentialFrame);
         tempPoint.changeFrame(steeringWheelFrame);
         double radius = Math.hypot(tempPoint.getX(), tempPoint.getY());
         double steeringWheelTangentialForce = steeringWheelExternalTorque / radius;
         force.setIncludingFrame(xTangentialFrame, steeringWheelTangentialForce, 0.0, 0.0);

         wrench.setToZero(hand.getBodyFixedFrame(), force.getReferenceFrame());
         wrench.setLinearPart(force.getVector());
         wrench.changeFrame(hand.getBodyFixedFrame());
         momentumBasedController.setExternalWrenchToCompensateFor(hand, wrench);
      }

      private void updateXTangentialFrame()
      {
         z.setIncludingFrame(steeringWheelFrame, 0.0, 0.0, 1.0);

         tempPoint.setToZero(creepyGripHandPositionControlFrame);
         tempPoint.changeFrame(steeringWheelFrame);
         y.setIncludingFrame(tempPoint);

         x.setToZero(steeringWheelFrame);
         x.cross(y, z);
         x.normalize();

         y.cross(z, x);
         y.normalize();

         tempMatrix.setColumn(0, x.getVector());
         tempMatrix.setColumn(1, y.getVector());
         tempMatrix.setColumn(2, z.getVector());

         rotationFromGainOrientationToBody.setIncludingFrame(steeringWheelFrame, tempMatrix);
         rotationFromGainOrientationToBody.changeFrame(positionController.getBodyFrame());

         // R^B_S
         rotationFromGainOrientationToBody.getMatrix3d(tempMatrix);
         transform.setRotationAndZeroTranslation(tempMatrix);
         xTangentialFrame.setTransformAndUpdate(transform);

         if (xTangentialFrameViz != null)
            xTangentialFrameViz.update();
      }

      public void doTransitionOutOfAction()
      {
      }

      public boolean isDone()
      {
         return individualHandControlModule.isDone();
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
