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
import us.ihmc.commonWalkingControlModules.trajectories.*;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;

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
   private final TransformReferenceFrame gainFrame;
   private final DynamicGraphicReferenceFrame gainFrameViz;


   public PointPositionRotateSteeringWheelBehavior(RobotSide robotSide, IndividualHandControlModule individualHandControlModule, ReferenceFrame creepyGripHandPositionControlFrame,
                                      FullRobotModel fullRobotModel, ReferenceFrame steeringWheelFrame, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {

      this.individualHandControlModule = individualHandControlModule;
      this.creepyGripHandPositionControlFrame = creepyGripHandPositionControlFrame;
      this.robotSide = robotSide;

      RigidBody hand = fullRobotModel.getHand(robotSide);
      jacobian = new GeometricJacobian(fullRobotModel.getElevator(), hand, fullRobotModel.getElevator().getBodyFixedFrame());

      positionController = new EuclideanPositionController("pointRotateSteeringWheel", creepyGripHandPositionControlFrame, registry);

      this.steeringWheelFrame = steeringWheelFrame;

      this.gainFrame = new TransformReferenceFrame("rotateSteeringWheelGainFrame", positionController.getBodyFrame());

      SE3ConfigurationProvider currentDesiredConfigurationProvider =
            individualHandControlModule.getCurrentDesiredConfigurationProvider(creepyGripHandPositionControlFrame);
      PositionProvider initialPositionProvider = currentDesiredConfigurationProvider;
      trajectoryGenerator = new CirclePositionTrajectoryGenerator("rotateSteeringWheelTrajectory", steeringWheelFrame, trajectoryTimeProvider, initialPositionProvider, registry, desiredRotationAngleProvider);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         gainFrameViz = new DynamicGraphicReferenceFrame(gainFrame, registry, 0.1);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("rotateSteeringWheelBehavior", gainFrameViz);
      }
      else
      {
         gainFrameViz = null;
      }
      parentRegistry.addChild(registry);
   }

   public Task getTask(double relativeRotationAngle)
   {
      return new RotateTask(relativeRotationAngle);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   private class RotateTask implements Task
   {
      private final double rotationAngle;
      private final FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame());
      private final FrameVector x = new FrameVector(); // direction of motion
      private final FrameVector y = new FrameVector(); // radial direction
      private final FrameVector z = new FrameVector(); // z direction
      private final FrameVector tempVector = new FrameVector();

      private final Matrix3d tempMatrix = new Matrix3d();
      private final Matrix3d proportionalGainMatrix = new Matrix3d();
      private final Matrix3d derivativeGainMatrix = new Matrix3d();
      private final FrameOrientation rotationFromGainOrientationToBody = new FrameOrientation(steeringWheelFrame);
      private final Transform3D transform = new Transform3D();

      private RotateTask(double rotationAngle)
      {
         this.rotationAngle = rotationAngle;
      }

      public void doTransitionIntoAction()
      {
         point.setToZero(creepyGripHandPositionControlFrame);

         double trajectoryTime = Math.abs(rotationAngle / averageAngularVelocity);
         trajectoryTimeProvider.set(trajectoryTime);
         desiredRotationAngleProvider.set(rotationAngle);
         individualHandControlModule.executePointPositionTrajectory(trajectoryGenerator, positionController, point, jacobian);
      }

      public void doAction()
      {
         trajectoryGenerator.packVelocity(tempVector);

         x.set(steeringWheelFrame, 1.0, 0.0, 0.0);
         tempMatrix.rotZ(trajectoryGenerator.getAngleFromXAxis() - Math.PI / 2.0);
         tempMatrix.transform(x.getVector());

         z.set(steeringWheelFrame, 0.0, 0.0, 1.0);

         y.setToZero(steeringWheelFrame);
         y.cross(z, x);
         y.normalize();

         tempMatrix.setColumn(0, x.getVector());
         tempMatrix.setColumn(1, y.getVector());
         tempMatrix.setColumn(2, z.getVector());

         rotationFromGainOrientationToBody.set(steeringWheelFrame, tempMatrix);
         rotationFromGainOrientationToBody.changeFrame(positionController.getBodyFrame());

         // R^B_S
         rotationFromGainOrientationToBody.getMatrix3d(tempMatrix);
         transform.set(tempMatrix);
         gainFrame.updateTransform(transform);

         if (gainFrameViz != null)
            gainFrameViz.update();

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

      public void doTransitionOutOfAction()
      {
      }

      public boolean isDone()
      {
         return individualHandControlModule.isDone();
      }
   }
}
