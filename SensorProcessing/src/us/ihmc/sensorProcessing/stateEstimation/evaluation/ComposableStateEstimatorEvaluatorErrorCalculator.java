package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableStateEstimatorEvaluatorErrorCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OrientationEstimator orientationEstimator;
   private final Robot robot;
   private final Joint estimationJoint;

   private final DoubleYoVariable orientationError = new DoubleYoVariable("orientationError", registry);
   private final DoubleYoVariable angularVelocityError = new DoubleYoVariable("angularVelocityError", registry);
   private final DoubleYoVariable comPositionError = new DoubleYoVariable("comPositionError", registry);
   private final DoubleYoVariable comVelocityError = new DoubleYoVariable("comVelocityError", registry);

   public ComposableStateEstimatorEvaluatorErrorCalculator(Robot robot, Joint estimationJoint, OrientationEstimator orientationEstimator,
           YoVariableRegistry parentRegistry)
   {
      this.robot = robot;
      this.estimationJoint = estimationJoint;
      this.orientationEstimator = orientationEstimator;

      parentRegistry.addChild(registry);
   }


   private void computeOrientationError()
   {
      FrameOrientation estimatedOrientation = orientationEstimator.getEstimatedOrientation();
      Quat4d estimatedOrientationQuat4d = new Quat4d();
      estimatedOrientation.getQuaternion(estimatedOrientationQuat4d);

      Quat4d orientationErrorQuat4d = new Quat4d();
      estimationJoint.getRotationToWorld(orientationErrorQuat4d);
      orientationErrorQuat4d.mulInverse(estimatedOrientationQuat4d);

      AxisAngle4d orientationErrorAxisAngle = new AxisAngle4d();
      orientationErrorAxisAngle.set(orientationErrorQuat4d);

      double errorAngle = AngleTools.trimAngleMinusPiToPi(orientationErrorAxisAngle.getAngle());

      orientationError.set(Math.abs(errorAngle));
   }

   private void computeAngularVelocityError()
   {
      Vector3d estimatedAngularVelocity = orientationEstimator.getEstimatedAngularVelocity().getVectorCopy();
      Vector3d actualAngularVelocity = new Vector3d();
      estimationJoint.getAngularVelocityInBody(actualAngularVelocity);

      actualAngularVelocity.sub(estimatedAngularVelocity);
      angularVelocityError.set(actualAngularVelocity.length());
   }

   private void computeCoMPositionError()
   {
      Point3d comPoint = new Point3d();
      Vector3d linearVelocity = new Vector3d();
      Vector3d angularMomentum = new Vector3d();

      robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);

      Vector3d comError = new Vector3d();
      comError.set(orientationEstimator.getEstimatedCoMPosition().getPointCopy());
      comError.sub(comPoint);

      comPositionError.set(comError.length());
   }

   private void computeCoMVelocityError()
   {
      Point3d comPoint = new Point3d();
      Vector3d linearVelocity = new Vector3d();
      Vector3d angularMomentum = new Vector3d();

      double mass = robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);
      linearVelocity.scale(1.0 / mass);

      Vector3d estimatedCoMVelocity = orientationEstimator.getEstimatedCoMVelocity().getVectorCopy();

      estimatedCoMVelocity.sub(linearVelocity);
      comVelocityError.set(estimatedCoMVelocity.length());
   }


   public void computeErrors()
   {
      computeOrientationError();
      computeAngularVelocityError();
      computeCoMPositionError();
      computeCoMVelocityError();
   }

}
