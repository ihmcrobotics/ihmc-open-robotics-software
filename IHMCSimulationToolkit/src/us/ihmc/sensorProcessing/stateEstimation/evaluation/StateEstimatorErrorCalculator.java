package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class StateEstimatorErrorCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StateEstimator orientationEstimator;
   private final Robot robot;
   private final Joint estimationJoint;

   private final DoubleYoVariable orientationError = new DoubleYoVariable("orientationError", registry);
   private final DoubleYoVariable angularVelocityError = new DoubleYoVariable("angularVelocityError", registry);
   private final DoubleYoVariable comXYPositionError = new DoubleYoVariable("comXYPositionError", registry);
   private final DoubleYoVariable comZPositionError = new DoubleYoVariable("comZPositionError", registry);
   private final DoubleYoVariable comVelocityError = new DoubleYoVariable("comVelocityError", registry);

   private final DoubleYoVariable pelvisXYPositionError = new DoubleYoVariable("pelvisXYPositionError", registry);
   private final DoubleYoVariable pelvisZPositionError = new DoubleYoVariable("pelvisZPositionError", registry);
   private final DoubleYoVariable pelvisLinearVelocityError = new DoubleYoVariable("pelvisLinearVelocityError", registry);

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
     
      Quat4d estimatedOrientationQuat4d = new Quat4d();
      estimatedOrientation.getQuaternion(estimatedOrientationQuat4d);

      Quat4d actualOrientation = new Quat4d();
      estimationJoint.getRotationToWorld(actualOrientation);
      
      if (((estimatedOrientationQuat4d.getW() > 0.0) && (actualOrientation.getW() < 0.0)) || ((estimatedOrientationQuat4d.getW() < 0.0) && (actualOrientation.getW() > 0.0)))
      {
         actualOrientation.negate();
      }
      
      perfectOrientation.set(actualOrientation);
      
      Quat4d orientationErrorQuat4d = new Quat4d(actualOrientation);
      orientationErrorQuat4d.mulInverse(estimatedOrientationQuat4d);

      AxisAngle4d orientationErrorAxisAngle = new AxisAngle4d();
      orientationErrorAxisAngle.set(orientationErrorQuat4d);

      double errorAngle = AngleTools.trimAngleMinusPiToPi(orientationErrorAxisAngle.getAngle());

      orientationError.set(Math.abs(errorAngle));
   }

   private final FrameVector estimatedAngularVelocityFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   
   private void computeAngularVelocityError()
   {
      orientationEstimator.getEstimatedAngularVelocity(estimatedAngularVelocityFrameVector);
      
      Vector3d estimatedAngularVelocity = estimatedAngularVelocityFrameVector.getVectorCopy();
      Vector3d actualAngularVelocity = new Vector3d();
      estimationJoint.physics.getAngularVelocityInBody(actualAngularVelocity);

      perfectAngularVelocity.set(actualAngularVelocity);
      
      actualAngularVelocity.sub(estimatedAngularVelocity);
      angularVelocityError.set(actualAngularVelocity.length());
   }

   private final FramePoint estimatedCoMPosition = new FramePoint();
   
   private void computeCoMPositionError()
   {
      Point3d comPoint = new Point3d();
      Vector3d linearVelocity = new Vector3d();
      Vector3d angularMomentum = new Vector3d();

      robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);
      perfectCoMPosition.set(comPoint);
      
      Vector3d comError = new Vector3d();
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
      Point3d comPoint = new Point3d();
      Vector3d linearVelocity = new Vector3d();
      Vector3d angularMomentum = new Vector3d();

      double mass = robot.computeCOMMomentum(comPoint, linearVelocity, angularMomentum);
      linearVelocity.scale(1.0 / mass);
      perfectCoMVelocity.set(linearVelocity);
      
      orientationEstimator.getEstimatedCoMVelocity(estimatedCoMVelocityFrameVector);
      Vector3d estimatedCoMVelocity = estimatedCoMVelocityFrameVector.getVectorCopy();

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
      Vector3d actualPosition = new Vector3d();
      Vector3d positionError = new Vector3d();

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
      Vector3d actualVelocity = new Vector3d();
      Vector3d linearVelocityError = new Vector3d();

      estimationJoint.physics.getLinearVelocityInWorld(actualVelocity, new Vector3d());
      perfectPelvisLinearVelocity.set(actualVelocity);
      
      orientationEstimator.getEstimatedPelvisLinearVelocity(estimatedPelvisVelocityFrameVector);
      estimatedPelvisVelocityFrameVector.get(linearVelocityError);
      linearVelocityError.sub(actualVelocity);

      pelvisLinearVelocityError.set(linearVelocityError.length());
   }
}
