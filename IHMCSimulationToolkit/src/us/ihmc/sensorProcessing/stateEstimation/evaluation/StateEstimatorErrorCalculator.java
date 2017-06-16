package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;


public class StateEstimatorErrorCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StateEstimator orientationEstimator;
   private final Robot robot;
   private final Joint estimationJoint;

   private final YoDouble orientationError = new YoDouble("orientationError", registry);
   private final YoDouble angularVelocityError = new YoDouble("angularVelocityError", registry);
   private final YoDouble comXYPositionError = new YoDouble("comXYPositionError", registry);
   private final YoDouble comZPositionError = new YoDouble("comZPositionError", registry);
   private final YoDouble comVelocityError = new YoDouble("comVelocityError", registry);

   private final YoDouble pelvisXYPositionError = new YoDouble("pelvisXYPositionError", registry);
   private final YoDouble pelvisZPositionError = new YoDouble("pelvisZPositionError", registry);
   private final YoDouble pelvisLinearVelocityError = new YoDouble("pelvisLinearVelocityError", registry);

   private final YoFrameQuaternion perfectOrientation = new YoFrameQuaternion("perfectOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector perfectAngularVelocity = new YoFrameVector("perfectAngularVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint perfectCoMPosition = new YoFramePoint("perfectCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector perfectCoMVelocity = new YoFrameVector("perfectCoMVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint perfectPelvisPosition = new YoFramePoint("perfectPelvisPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector perfectPelvisLinearVelocity = new YoFrameVector("perfectPelvisLinearVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final boolean assumePerfectIMU;
   private final boolean useSimplePelvisPositionEstimator;


   public StateEstimatorErrorCalculator(Robot robot, Joint estimationJoint, StateEstimator orientationEstimator, boolean assumePerfectIMU,
           boolean useSimplePelvisPositionEstimator, YoVariableRegistry parentRegistry)
   {
      this.robot = robot;
      this.estimationJoint = estimationJoint;
      this.orientationEstimator = orientationEstimator;
      this.assumePerfectIMU = assumePerfectIMU;
      this.useSimplePelvisPositionEstimator = useSimplePelvisPositionEstimator;
      parentRegistry.addChild(registry);
   }

   private final FrameOrientation estimatedOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
   
   private void computeOrientationError()
   {
      orientationEstimator.getEstimatedOrientation(estimatedOrientation);
     
      Quaternion estimatedOrientationQuat4d = new Quaternion();
      estimatedOrientation.getQuaternion(estimatedOrientationQuat4d);

      Quaternion actualOrientation = new Quaternion();
      estimationJoint.getRotationToWorld(actualOrientation);
      
      if (((estimatedOrientationQuat4d.getS() > 0.0) && (actualOrientation.getS() < 0.0)) || ((estimatedOrientationQuat4d.getS() < 0.0) && (actualOrientation.getS() > 0.0)))
      {
         actualOrientation.negate();
      }
      
      perfectOrientation.set(actualOrientation);
      
      Quaternion orientationErrorQuat4d = new Quaternion(actualOrientation);
      orientationErrorQuat4d.multiplyConjugateOther(estimatedOrientationQuat4d);

      AxisAngle orientationErrorAxisAngle = new AxisAngle();
      orientationErrorAxisAngle.set(orientationErrorQuat4d);

      double errorAngle = AngleTools.trimAngleMinusPiToPi(orientationErrorAxisAngle.getAngle());

      orientationError.set(Math.abs(errorAngle));
   }

   private final FrameVector estimatedAngularVelocityFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   
   private void computeAngularVelocityError()
   {
      orientationEstimator.getEstimatedAngularVelocity(estimatedAngularVelocityFrameVector);
      
      Vector3D estimatedAngularVelocity = estimatedAngularVelocityFrameVector.getVectorCopy();
      Vector3D actualAngularVelocity = new Vector3D();
      estimationJoint.physics.getAngularVelocityInBody(actualAngularVelocity);

      perfectAngularVelocity.set(actualAngularVelocity);
      
      actualAngularVelocity.sub(estimatedAngularVelocity);
      angularVelocityError.set(actualAngularVelocity.length());
   }

   private final FramePoint estimatedCoMPosition = new FramePoint();
   
   private void computeCoMPositionError()
   {
      Point3D comPoint = new Point3D();
      Vector3D linearVelocity = new Vector3D();
      Vector3D angularMomentum = new Vector3D();

      robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);
      perfectCoMPosition.set(comPoint);
      
      Vector3D comError = new Vector3D();
      orientationEstimator.getEstimatedCoMPosition(estimatedCoMPosition);
      comError.set(estimatedCoMPosition.getPointCopy());
      comError.sub(comPoint);

      comZPositionError.set(comError.getZ());

      comError.setZ(0.0);
      comXYPositionError.set(comError.length());
   }

   private final FrameVector estimatedCoMVelocityFrameVector = new FrameVector();

   private void computeCoMVelocityError()
   {
      Point3D comPoint = new Point3D();
      Vector3D linearVelocity = new Vector3D();
      Vector3D angularMomentum = new Vector3D();

      double mass = robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);
      linearVelocity.scale(1.0 / mass);
      perfectCoMVelocity.set(linearVelocity);
      
      orientationEstimator.getEstimatedCoMVelocity(estimatedCoMVelocityFrameVector);
      Vector3D estimatedCoMVelocity = estimatedCoMVelocityFrameVector.getVectorCopy();

      estimatedCoMVelocity.sub(linearVelocity);
      comVelocityError.set(estimatedCoMVelocity.length());
   }


   public void computeErrors()
   {
	   if(!assumePerfectIMU)
	   {
		   computeOrientationError();
		   computeAngularVelocityError();
	   }

	   computeCoMPositionError();
	   computeCoMVelocityError();
	   
	   if (useSimplePelvisPositionEstimator)
	   {
         computePelvisPositionError();
         computePelvisVelocityError();
	   }
   }

   private final FramePoint estimatedPelvisPosition = new FramePoint();
   
   private void computePelvisPositionError()
   {
      Vector3D actualPosition = new Vector3D();
      Vector3D positionError = new Vector3D();

      estimationJoint.getTranslationToWorld(actualPosition);
      perfectPelvisPosition.set(actualPosition);
      
      orientationEstimator.getEstimatedPelvisPosition(estimatedPelvisPosition);
      estimatedPelvisPosition.get(positionError);
      positionError.sub(actualPosition);

      pelvisZPositionError.set(positionError.getZ());

      positionError.setZ(0.0);
      pelvisXYPositionError.set(positionError.length());
   }

   private final FrameVector estimatedPelvisVelocityFrameVector = new FrameVector();

   private void computePelvisVelocityError()
   {
      Vector3D actualVelocity = new Vector3D();
      Vector3D linearVelocityError = new Vector3D();

      estimationJoint.physics.getLinearVelocityInWorld(actualVelocity, new Vector3D());
      perfectPelvisLinearVelocity.set(actualVelocity);
      
      orientationEstimator.getEstimatedPelvisLinearVelocity(estimatedPelvisVelocityFrameVector);
      estimatedPelvisVelocityFrameVector.get(linearVelocityError);
      linearVelocityError.sub(actualVelocity);

      pelvisLinearVelocityError.set(linearVelocityError.length());
   }
}
