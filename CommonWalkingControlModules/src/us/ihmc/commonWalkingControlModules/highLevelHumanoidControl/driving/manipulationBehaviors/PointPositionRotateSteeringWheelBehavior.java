package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving.manipulationBehaviors;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.EuclideanPositionController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.Task;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.*;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

/**
 * @author twan
 *         Date: 6/12/13
 */
public class PointPositionRotateSteeringWheelBehavior
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoVariableDoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider("rotateSteeringWheelTrajectoryTime", registry);
   private final YoVariableDoubleProvider desiredRotationAngleProvider = new YoVariableDoubleProvider("rotateSteeringWheelDeltaAngle", registry);

   private final EuclideanPositionController positionController;

   private final double averageAngularVelocity = 1.0;

   private final IndividualHandControlModule individualHandControlModule;
   private final ReferenceFrame creepyGripHandPositionControlFrame;

   private final CirclePositionTrajectoryGenerator trajectoryGenerator;
   private final RobotSide robotSide;
   private final GeometricJacobian jacobian;

   private final double kpRadial = 100.0;
   private final double kpZ = 100.0;
   private final double kpTangential = 100.0;
   private final double zeta = 1.0;
   private final ReferenceFrame steeringWheelFrame;
   private final TransformReferenceFrame xTangentialFrame;
   private final DynamicGraphicReferenceFrame gainFrameViz;
   private final MomentumBasedController momentumBasedController;


   public PointPositionRotateSteeringWheelBehavior(RobotSide robotSide, IndividualHandControlModule individualHandControlModule,
           ReferenceFrame creepyGripHandPositionControlFrame, FullRobotModel fullRobotModel, ReferenceFrame steeringWheelFrame, MomentumBasedController momentumBasedController,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.individualHandControlModule = individualHandControlModule;
      this.creepyGripHandPositionControlFrame = creepyGripHandPositionControlFrame;
      this.robotSide = robotSide;

      RigidBody hand = fullRobotModel.getHand(robotSide);
      jacobian = new GeometricJacobian(fullRobotModel.getElevator(), hand, fullRobotModel.getElevator().getBodyFixedFrame());

      positionController = new EuclideanPositionController("pointRotateSteeringWheel", creepyGripHandPositionControlFrame, registry);

      this.steeringWheelFrame = steeringWheelFrame;

      this.xTangentialFrame = new TransformReferenceFrame("rotateSteeringWheelGainFrame", positionController.getBodyFrame());

      this.momentumBasedController = momentumBasedController;

      SE3ConfigurationProvider currentDesiredConfigurationProvider =
         individualHandControlModule.getCurrentDesiredConfigurationProvider(creepyGripHandPositionControlFrame);
      PositionProvider initialPositionProvider = currentDesiredConfigurationProvider;
      trajectoryGenerator = new CirclePositionTrajectoryGenerator("rotateSteeringWheelTrajectory", steeringWheelFrame, trajectoryTimeProvider,
              initialPositionProvider, registry, desiredRotationAngleProvider);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         gainFrameViz = new DynamicGraphicReferenceFrame(xTangentialFrame, registry, 0.1);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("rotateSteeringWheelBehavior", gainFrameViz);
      }
      else
      {
         gainFrameViz = null;
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
      private final Matrix3d proportionalGainMatrix = new Matrix3d();
      private final Matrix3d derivativeGainMatrix = new Matrix3d();
      private final FrameOrientation rotationFromGainOrientationToBody = new FrameOrientation(steeringWheelFrame);
      private final Transform3D transform = new Transform3D();
      private final FrameVector force = new FrameVector();
      private final Wrench wrench = new Wrench();

      private RotateTask(double relativeRotationAngle, double absoluteRotationAngle)
      {
         this.relativeRotationAngle = relativeRotationAngle;
         this.absoluteRotationAngle = absoluteRotationAngle;
      }

      public void doTransitionIntoAction()
      {
//         Transform3D transform3D = creepyGripHandPositionControlFrame.getTransformToDesiredFrame(steeringWheelFrame);
//         Matrix3d rotationMatrix = new Matrix3d();
//         transform3D.get(rotationMatrix);
//         System.out.println(RotationFunctions.getYaw(rotationMatrix) + ", " + RotationFunctions.getPitch(rotationMatrix) + ", " + RotationFunctions.getRoll(rotationMatrix));

         tempPoint.setToZero(creepyGripHandPositionControlFrame);

         double minTrajectoryTime = 0.1;
         double trajectoryTime = Math.max(minTrajectoryTime, Math.abs(relativeRotationAngle / averageAngularVelocity));
         trajectoryTimeProvider.set(trajectoryTime);
         desiredRotationAngleProvider.set(relativeRotationAngle);
         double kp = 100.0;
         double kd = GainCalculator.computeDerivativeGain(kp, 1.0);
         positionController.setProportionalGains(kp, kp, kp);
         positionController.setDerivativeGains(kd, kd, kd);
         individualHandControlModule.executePointPositionTrajectory(trajectoryGenerator, positionController, tempPoint, jacobian);
      }

      public void doAction()
      {
//         updateXTangentialFrame();
//         setExternalWrench();
//         updateGains();
      }

      private void setExternalWrench()
      {
         double steeringWheelExternalTorqueMagnitude = 1.0; // from DRCVehiclePlugin.cc, handWheelForce (100 Nm / rad P control, clipped to 1)
         double steeringWheelExternalTorque = Math.signum(absoluteRotationAngle) * steeringWheelExternalTorqueMagnitude;

         tempPoint.setToZero(xTangentialFrame);
         tempPoint.changeFrame(steeringWheelFrame);
         double radius = Math.hypot(tempPoint.getX(), tempPoint.getY());
         double steeringWheelTangentialForce = steeringWheelExternalTorque / radius;
         force.set(xTangentialFrame, steeringWheelTangentialForce, 0.0, 0.0);

         wrench.setToZero(jacobian.getEndEffector().getBodyFixedFrame(), force.getReferenceFrame());
         wrench.setLinearPart(force.getVector());
         wrench.changeFrame(jacobian.getEndEffector().getBodyFixedFrame());
         momentumBasedController.setExternalWrenchToCompensateFor(jacobian.getEndEffector(), wrench);
      }

      private void updateGains()
      {
         xTangentialFrame.getTransformToDesiredFrame(transform, positionController.getBodyFrame());
         transform.get(tempMatrix);

         // K^S
         proportionalGainMatrix.setElement(0, 0, kpTangential);
         proportionalGainMatrix.setElement(1, 1, kpRadial);
         proportionalGainMatrix.setElement(2, 2, kpZ);

         // R^B_S * K_S * R^S_B
         proportionalGainMatrix.mul(tempMatrix, proportionalGainMatrix);
         proportionalGainMatrix.mulTransposeRight(proportionalGainMatrix, tempMatrix);
         positionController.setProportionalGains(proportionalGainMatrix);

         // B^S
         double kdTangential = GainCalculator.computeDerivativeGain(kpTangential, zeta);
         double kdRadial = GainCalculator.computeDerivativeGain(kpRadial, zeta);
         double kdZ = GainCalculator.computeDerivativeGain(kpZ, zeta);

         derivativeGainMatrix.setElement(0, 0, kdTangential);
         derivativeGainMatrix.setElement(1, 1, kdRadial);
         derivativeGainMatrix.setElement(2, 2, kdZ);

         // R^B_S * B_S * R^S_B
         derivativeGainMatrix.mul(tempMatrix, derivativeGainMatrix);
         derivativeGainMatrix.mulTransposeRight(derivativeGainMatrix, tempMatrix);
         positionController.setDerivativeGains(derivativeGainMatrix);
      }

      private void updateXTangentialFrame()
      {
         x.set(steeringWheelFrame, 1.0, 0.0, 0.0);
         tempMatrix.rotZ(getDesiredAngleFromYAxis());
         tempMatrix.transform(x.getVector());

         z.set(steeringWheelFrame, 0.0, 0.0, 1.0);

         y.setToZero(steeringWheelFrame);
         y.cross(z, x);
         y.normalize();

         z.cross(x, y);
         z.normalize(); // should not be necessary, but just to be sure

         tempMatrix.setColumn(0, x.getVector());
         tempMatrix.setColumn(1, y.getVector());
         tempMatrix.setColumn(2, z.getVector());

         rotationFromGainOrientationToBody.set(steeringWheelFrame, tempMatrix);
         rotationFromGainOrientationToBody.changeFrame(positionController.getBodyFrame());

         // R^B_S
         rotationFromGainOrientationToBody.getMatrix3d(tempMatrix);
         transform.set(tempMatrix);
         xTangentialFrame.updateTransform(transform);

         if (gainFrameViz != null)
            gainFrameViz.update();
      }

      public void doTransitionOutOfAction()
      {
      }

      public boolean isDone()
      {
         return individualHandControlModule.isDone();
      }
   }

   private double getDesiredAngleFromYAxis()
   {
      return trajectoryGenerator.getAngleFromXAxis() - Math.PI / 2.0;
   }
}
