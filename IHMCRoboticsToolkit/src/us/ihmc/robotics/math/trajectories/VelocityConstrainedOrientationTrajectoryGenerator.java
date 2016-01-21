package us.ihmc.robotics.math.trajectories;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
/**
 * Currently under development. Trying to fix this trajectory properly but it is not usable yet.
 * Two useful references from the web:
 * <p> a) <a href="http://www.geometrictools.com/Documentation/Quaternions.pdf">(SLERP & SQUAD methods)</a> </p>
 * <p> b) <a href="http://www.cim.mcgill.ca/~mpersson/docs/quat_calc_notes.pdf">(Proper quaternion calculus)</a> </p>
 * 
 * I got inspired by the first reference with the SQUAD method, but watch out there the math is totally WRONG.
 * The second reference provides a proper approach to general quaternion calculus. I used it a lot to derive time derivatives with quaternion & angular velocity. Really useful ref.
 *
 * <p>
 * This class needs a total cleanup and probably to extract several methods to a "tool" class.
 * I'll do so as soon as the math is right and it works.
 * </p>
 *
 * @author Sylvain 
 */
public class VelocityConstrainedOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final YoPolynomial parameterPolynomial;

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameQuaternion initialOrientationDrifted;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameQuaternion finalOrientationDrifted;
   private final YoFrameVector finalAngularVelocity;

   private final YoFrameVector initialUnitRotationVector;
   private final YoFrameVector finalUnitRotationVector;

   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final DoubleYoVariable desiredAngularVelocityMagnitude;
   private final YoFrameVector desiredAngularVelocityDirection;
   private final YoFrameVector deltaAngularVelocityDirection;
   private final YoFrameVector intermediateAngularVelocityDirection;
   private final YoFrameVector desiredAngularAcceleration;

   private final DoubleProvider trajectoryTimeProvider;
   private final OrientationProvider initialOrientationProvider;
   private final VectorProvider initialAngularVelocityProvider;
   private final OrientationProvider finalOrientationProvider;
   private final VectorProvider finalAngularVelocityProvider;

   private final FrameOrientation tempInitialOrientation;
   private final FrameOrientation tempFinalOrientation;
   private final FrameVector tempInitialVelocity;
   private final FrameVector tempFinalVelocity;

   private final ReferenceFrame trajectoryFrame;

   public VelocityConstrainedOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
         OrientationProvider initialOrientationProvider, VectorProvider initialAngularVelocityProvider, OrientationProvider finalOrientationProvider,
         VectorProvider finalAngularVelocityProvider, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      this.parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      this.trajectoryFrame = referenceFrame;

      this.initialOrientation = new YoFrameQuaternion(namePrefix + "InitialOrientation", trajectoryFrame, registry);
      this.initialOrientationDrifted = new YoFrameQuaternion(namePrefix + "InitialOrientationDrifted", trajectoryFrame, registry);
      this.initialAngularVelocity = new YoFrameVector(namePrefix + "InitialAngularVelocity", trajectoryFrame, registry);
      this.finalOrientation = new YoFrameQuaternion(namePrefix + "FinalOrientation", trajectoryFrame, registry);
      this.finalOrientationDrifted = new YoFrameQuaternion(namePrefix + "FinalOrientationDrifted", trajectoryFrame, registry);
      this.finalAngularVelocity = new YoFrameVector(namePrefix + "FinalAngularVelocity", trajectoryFrame, registry);

      initialUnitRotationVector = new YoFrameVector(namePrefix + "InitialUnitRotationVector", trajectoryFrame, registry);
      finalUnitRotationVector = new YoFrameVector(namePrefix + "FinalUnitRotationVector", trajectoryFrame, registry);

      this.desiredOrientation = new YoFrameQuaternion(namePrefix + "DesiredOrientation", trajectoryFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector(namePrefix + "DesiredAngularVelocity", trajectoryFrame, registry);
      this.desiredAngularVelocityMagnitude = new DoubleYoVariable(namePrefix + "DesiredAngularVelocityMagnitude", registry);
      this.desiredAngularVelocityDirection = new YoFrameVector(namePrefix + "DesiredAngularVelocityDirection", trajectoryFrame, registry);
      this.deltaAngularVelocityDirection = new YoFrameVector(namePrefix + "DeltaAngularVelocityDirection", trajectoryFrame, registry);
      this.intermediateAngularVelocityDirection = new YoFrameVector(namePrefix + "IntermediateAngularVelocityDirection", trajectoryFrame, registry);
      this.desiredAngularAcceleration = new YoFrameVector(namePrefix + "DesiredAngularAcceleration", trajectoryFrame, registry);

      this.trajectoryTimeProvider = trajectoryTimeProvider;
      this.initialOrientationProvider = initialOrientationProvider;
      this.initialAngularVelocityProvider = initialAngularVelocityProvider;
      this.finalOrientationProvider = finalOrientationProvider;
      this.finalAngularVelocityProvider = finalAngularVelocityProvider;

      tempInitialOrientation = new FrameOrientation(trajectoryFrame);
      tempFinalOrientation = new FrameOrientation(trajectoryFrame);
      tempInitialVelocity = new FrameVector(trajectoryFrame);
      tempFinalVelocity = new FrameVector(trajectoryFrame);

      parentRegistry.addChild(registry);
   }

   private final Vector3d unitAngularVelocityVector0 = new Vector3d();
   private final Vector3d unitAngularVelocityVectorF = new Vector3d();

   private double angularVelocityMagnitude0 = 0.0;
   private double angularVelocityMagnitudeF = 0.0;
   private double driftAngleMagnitude0 = 0.0;
   private double driftAngleMagnitudeF = 0.0;
   private double driftAngleMagnitudeNotTrimmed0 = 0.0;
   private double driftAngleMagnitudeNotTrimmedF = 0.0;

   @Override
   public void initialize()
   {
      double trajectoryTime = trajectoryTimeProvider.getValue();
      MathTools.checkIfInRange(trajectoryTime, 0.0, Double.POSITIVE_INFINITY);
      this.trajectoryTime.set(trajectoryTime);
      currentTime.set(0.0);

      initialOrientationProvider.get(tempInitialOrientation);
      tempInitialOrientation.changeFrame(trajectoryFrame);
      initialOrientation.set(tempInitialOrientation);
      initialOrientation.checkQuaternionIsUnitMagnitude();

      initialAngularVelocityProvider.get(tempInitialVelocity);
      initialAngularVelocity.setAndMatchFrame(tempInitialVelocity);

      finalOrientationProvider.get(tempFinalOrientation);
      tempFinalOrientation.changeFrame(trajectoryFrame);
      finalOrientation.set(tempFinalOrientation);
      finalOrientation.checkQuaternionIsUnitMagnitude();

      finalAngularVelocityProvider.get(tempFinalVelocity);
      finalAngularVelocity.setAndMatchFrame(tempFinalVelocity);

      parameterPolynomial.setQuintic(0, trajectoryTime, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      desiredOrientation.set(initialOrientation);
      desiredAngularVelocity.setAndMatchFrame(tempInitialVelocity);
      desiredAngularAcceleration.setToZero();
   }

   private final AxisAngle4d deltaAxisAngle0 = new AxisAngle4d();
   private final AxisAngle4d deltaAxisAngleF = new AxisAngle4d();
   private final Quat4d quaternion0 = new Quat4d();
   private final Quat4d quaternionF = new Quat4d();
   private final Quat4d quaternionDrifted0 = new Quat4d();
   private final Quat4d quaternionDriftedF = new Quat4d();
   private final Quat4d deltaQuaternion0 = new Quat4d();
   private final Quat4d deltaQuaternionF = new Quat4d();

   private final Vector3d angularVelocity0 = new Vector3d();
   private final Vector3d angularVelocityF = new Vector3d();
   private final Vector3d interpolatedAngularVelocity = new Vector3d();

   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   /**
    * Note that this method is a total mess. I am using it to screw around and get a better feel of what's going on.
    * Main thing happening here is the integration of the initial and final angular velocities that then added to the initial and final orientations as the trajectory time is growing.
    * deltaQuaternion0 and deltaQuaternionF are the quaternions resulting from the angular velocity integration.
    * Note that deltaQuaternion0 = {int(w_0 * currentTime) dt} and deltaQuaternionF = {int(w_F * (currentTime - trajectoryTime)) dt}.
    *  quaternionDrifted0 and quaternionDriftedF are then used for the interpolation:
    *  quaternionDrifted0 = deltaQuaternion0 * quaternion0 and quaternionDriftedF = deltaQuaternionF * quaternionF
    *  
    *  The interpolation method for the orientation is {@link #computeInterpolatedOrientation(double, Quat4d, Quat4d, Quat4d)} and for the angular velocity {@link #computeInterpolatedAngularVelocity(double, double, Quat4d, Vector3d, Quat4d, Vector3d, Vector3d)}.
    *  Still some work needed for the angular velocity and need to implement the same for the angular acceleration.
    */
   @Override
   public void compute(double time)
   {
      this.currentTime.set(time);

      if (isDone())
      {
         tempFinalOrientation.changeFrame(trajectoryFrame);
         desiredOrientation.set(tempFinalOrientation);
         desiredAngularVelocity.setAndMatchFrame(tempFinalVelocity);
         desiredAngularAcceleration.setToZero();
      }

      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime.getDoubleValue());
      parameterPolynomial.compute(time);

      angularVelocityMagnitude0 = initialAngularVelocity.length();

      if (angularVelocityMagnitude0 > 1.0e-6)
      {
         initialOrientation.get(quaternion0);
         initialAngularVelocity.get(unitAngularVelocityVector0);
         unitAngularVelocityVector0.normalize();
         initialUnitRotationVector.set(unitAngularVelocityVector0);

         driftAngleMagnitudeNotTrimmed0 = angularVelocityMagnitude0 * time;
         driftAngleMagnitude0 = AngleTools.trimAngleMinusPiToPi(driftAngleMagnitudeNotTrimmed0);
         deltaAxisAngle0.set(unitAngularVelocityVector0, driftAngleMagnitude0);
         deltaQuaternion0.set(deltaAxisAngle0);
         quaternionDrifted0.mul(deltaQuaternion0, quaternion0);
      }
      else
      {
         initialOrientation.get(quaternionDrifted0);
      }

      angularVelocityMagnitudeF = finalAngularVelocity.length();

      if (angularVelocityMagnitudeF > 1.0e-6)
      {
         finalOrientation.get(quaternionF);
         finalAngularVelocity.get(unitAngularVelocityVectorF);
         unitAngularVelocityVectorF.normalize();
         if (unitAngularVelocityVector0.dot(unitAngularVelocityVectorF) < 0.0)
         {
            unitAngularVelocityVectorF.negate();
            angularVelocityMagnitudeF = -angularVelocityMagnitudeF;
         }

         driftAngleMagnitudeNotTrimmedF = angularVelocityMagnitudeF * (time - trajectoryTime.getDoubleValue());
         driftAngleMagnitudeF = AngleTools.trimAngleMinusPiToPi(driftAngleMagnitudeNotTrimmedF);


         finalUnitRotationVector.set(unitAngularVelocityVectorF);

         deltaAxisAngleF.set(unitAngularVelocityVectorF, driftAngleMagnitudeF);
         deltaQuaternionF.set(deltaAxisAngleF);
         quaternionDriftedF.mul(deltaQuaternionF, quaternionF);
      }
      else
      {
         finalOrientation.get(quaternionDriftedF);
      }

      initialAngularVelocity.get(angularVelocity0);
      finalAngularVelocity.get(angularVelocityF);

      initialOrientationDrifted.set(quaternionDrifted0);
      finalOrientationDrifted.set(quaternionDriftedF);

      boolean shouldBeZero = isDone() || currentTime.getDoubleValue() < 0.0;
      double alpha = shouldBeZero ? 1.0 : parameterPolynomial.getPosition();
      double alphaDot = shouldBeZero ? 0.0 : parameterPolynomial.getVelocity();
      double alphaDDot = shouldBeZero ? 0.0 : parameterPolynomial.getAcceleration();

      if (!isDone())
      {
         Quat4d qInterpolated = new Quat4d();
         computeInterpolatedOrientation(alpha, quaternionDrifted0, quaternionDriftedF, qInterpolated);
         desiredOrientation.set(qInterpolated);
         
         Vector3d wInterpolated = new Vector3d();
         Matrix3d matrix3d = new Matrix3d();
         matrix3d.set(quaternionDrifted0);
         matrix3d.transpose();
         matrix3d.transform(angularVelocity0);
         matrix3d.set(quaternionDriftedF);
         matrix3d.transpose();
         matrix3d.transform(angularVelocityF);
         computeInterpolatedAngularVelocity(alpha, alphaDot, quaternionDrifted0, angularVelocity0, quaternionDriftedF, angularVelocityF, wInterpolated);
         matrix3d.set(qInterpolated);
//         matrix3d.transpose();
         matrix3d.transform(wInterpolated);
         desiredAngularVelocity.set(wInterpolated);
         
//         double driftAngle = alpha * driftAngleMagnitudeNotTrimmedF + (1.0 - alpha) * driftAngleMagnitudeNotTrimmed0;
//         Vector3d unitRotationVector = new Vector3d();
//         unitRotationVector.interpolate(unitAngularVelocityVector0, unitAngularVelocityVectorF, alpha);
//         if (unitRotationVector.length() < 1.0e-7)
//         {
//            unitRotationVector.set(1.0, 0.0, 0.0);
//            driftAngle = 0.0;
//         }
//         else
//            unitRotationVector.normalize();
//         AxisAngle4d deltaAxisAngle = new AxisAngle4d(unitRotationVector, driftAngle);
//         
//         Quat4d deltaQuat = new Quat4d();
//         deltaQuat.set(deltaAxisAngle);
//         deltaQuat.interpolate(deltaQuaternion0, deltaQuaternionF, parameterPolynomial.getPosition());
//         Quat4d intermediateQuat = new Quat4d();
//         intermediateQuat.interpolate(quaternion0, quaternionF, parameterPolynomial.getPosition());
//         Quat4d desiredQuat = new Quat4d();
//         desiredQuat.mul(deltaQuat, intermediateQuat);
//         desiredOrientation.set(desiredQuat);
//         
//         Vector3d intermediateAngularVelocity = new Vector3d();
//         orientationInterpolationCalculator.computeAngularVelocity(intermediateAngularVelocity, quaternion0, quaternionF, alphaDot);
//         
//
//         
//         Vector3d intermediateAngularVelocity2 = new Vector3d();
//         orientationInterpolationCalculator.computeAngularVelocity(intermediateAngularVelocity2, deltaQuaternion0, deltaQuaternionF, alphaDot);
//
//         Vector3d deltaAngularVelocity = new Vector3d();
//         deltaAngularVelocity.interpolate(unitAngularVelocityVector0, unitAngularVelocityVectorF, alpha);
//         double omega = alpha * angularVelocityMagnitudeF + (1.0 - alpha) * angularVelocityMagnitude0;
//         deltaAngularVelocity.scale(omega);
//         desiredAngularVelocity.setToZero();
//         desiredAngularVelocity.add(deltaAngularVelocity);
////         desiredAngularVelocity.add(intermediateAngularVelocity);
//         desiredAngularVelocity.add(intermediateAngularVelocity2);
         desiredAngularVelocityMagnitude.set(desiredAngularVelocity.length());
         desiredAngularVelocityDirection.set(desiredAngularVelocity);
         desiredAngularVelocityDirection.scale(1.0 / desiredAngularVelocityMagnitude.getDoubleValue());
         
         orientationInterpolationCalculator.computeAngularAcceleration(desiredAngularAcceleration, initialOrientationDrifted, finalOrientationDrifted, alphaDDot);
      }
      else
      {
         desiredOrientation.set(finalOrientation);
         desiredAngularVelocity.setToZero();
         desiredAngularAcceleration.setToZero();
      }
   }

   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();

   /**
    * This computes the product: resultToPack = (q0^-1 q1)
    */
   private void computeQuaternionDifference(Quat4d q0, Quat4d q1, Quat4d resultToPack)
   {
      resultToPack.inverse(q0);
      resultToPack.mul(q1);
   }

   /**
    * This computes: resultToPack = log(q)
    * Assumes that q is a unit-quaternion and describes a orientation.
    */
   private void computeLogQuaternion(Quat4d q, Quat4d resultToPack)
   {
      tempAxisAngle.set(q);
      resultToPack.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ(), 0.0);
      resultToPack.scale(tempAxisAngle.getAngle());
   }

   /**
    * This computes: resultToPack = q^power.
    * Assumes that q is a unit-quaternion and describes a orientation.
    */
   private void computePowerQuaternion(Quat4d q, double power, Quat4d resultToPack)
   {
      tempAxisAngle.set(q);
      tempAxisAngle.setAngle(power * tempAxisAngle.getAngle());
      resultToPack.set(tempAxisAngle);
   }

   private final Quat4d tempLogQ = new Quat4d();
   private final Quat4d tempPoweredQ = new Quat4d();
   private final Quat4d tempQDot = new Quat4d();

   /**
    * Computes: resultToPack = d/dt (q^p)
    * - (power > 0) d/dt (q^p) = power qDot + powerDot log(q)
    * - (power < 0) d/dt (q^p) = power q^-power qDot q^power + powerDot log(q)
    * Assumes that q is a unit-quaternion and describes a orientation.
    * Assumes that qDot is a pure-quaternion and describes an angular velocity in the local frame of q.
    * Returns a pure-quaternion describing angular velocity in the local frame q^p.
    */
   private void computeDerivativePoweredQuaternion(Quat4d q, Quat4d qDot, double power, double powerDot, Quat4d resultToPack)
   {
      tempQDot.set(qDot);
      if (power < 0.0)
      {
         computePowerQuaternion(q, power, tempPoweredQ);
         invertTransform(tempPoweredQ, tempQDot);
      }
      
      computeLogQuaternion(q, tempLogQ);
      tempLogQ.scale(powerDot);
      resultToPack.set(tempQDot);
      resultToPack.scale(power);
      resultToPack.add(tempLogQ);
   }

   /**
    * Rotates qToTransform by the rotation described by q: qToTransform = q qToTransform q^-1
    * Assumes that q is a unit-quaternion and describes a orientation.
    * Assumes that qToTransform is a pure-quaternion.
    */
   private void transform(Quat4d q, Quat4d qToTransform)
   {
      qToTransform.mul(q, qToTransform);
      qToTransform.mulInverse(q);
   }

   private final Quat4d qInv = new Quat4d();

   /**
    * Rotates qToTransform by the inverse of the rotation described by q: qToTransform = q^-1 qToTransform q
    * Assumes that q is a unit-quaternion and describes a orientation.
    * Assumes that qToTransform is a pure-quaternion.
    */
   private void invertTransform(Quat4d q, Quat4d qToTransform)
   {
      qInv.inverse(q);
      transform(qInv, qToTransform);
   }

   /**
    * Computes: resultToPack = d/dt (q0 q1) = q1^-1 q0Dot q1 + q1Dot.
    * Assumes that q0 and q1 are unit-quaternions and describe orientations.
    * Assumes that q0Dot and q1Dot are pure-quaternion and describe angular velocities in the local frames of q0 and q1 respectively.
    * Returns a pure-quaternion describing angular velocity in the local frame (q0 q1).
    */
   private void computeDerivativeQuaternionProduct(Quat4d q0, Quat4d q0Dot, Quat4d q1, Quat4d q1Dot, Quat4d resultToPack)
   {
      resultToPack.set(q0Dot);
      invertTransform(q1, resultToPack);
      resultToPack.add(q1Dot);
   }

   /**
    * Computes: resultToPack = d/dt (q0^-1 q1) = q1^-1 q0 q0Dot q0^-1 q1 + q1Dot.
    * Assumes that q0 and q1 are unit-quaternions and describe orientations.
    * Assumes that q0Dot and q1Dot are pure-quaternion and describe angular velocities in the local frames of q0 and q1 respectively.
    * Returns a pure-quaternion describing angular velocity in the local frame (q0^-1 q1).
    */
   private void computeDerivativeQuaternionDifference(Quat4d q0, Quat4d q0Dot, Quat4d q1, Quat4d q1Dot, Quat4d resultToPack)
   {
      resultToPack.set(q0Dot);
      transform(q0, resultToPack);
      invertTransform(q1, resultToPack);
      resultToPack.sub(q1Dot, resultToPack);
   }

   /**
    * Interpolation from q0 to q1 for a given alpha = [0, 1] using SLERP.
    * Computes: resultToPack = q0 (q0^-1 q1)^alpha.
    */
   private void computeInterpolatedOrientation(double alpha, Quat4d q0, Quat4d q1, Quat4d resultToPack)
   {
      computeQuaternionDifference(q0, q1, resultToPack);
      computePowerQuaternion(resultToPack, alpha, resultToPack);
      resultToPack.mul(q0, resultToPack);
   }

   private final Quat4d q0Dot = new Quat4d();
   private final Quat4d q1Dot = new Quat4d();

   private final Quat4d tempResult3 = new Quat4d();

   /**
    * Interpolation of the angular velocity considering that the orientation is being interpolated using the SLERP method.
    * Computes: resultToPack = d/dt (q0 (q0^-1 q1)^alpha).
    * Almost there but still not working perfectly, probably a minus sign somewhere.
    */
   private void computeInterpolatedAngularVelocity(double alpha, double alphaDot, Quat4d q0, Vector3d w0, Quat4d q1, Vector3d w1, Vector3d resultToPack)
   {
      q0Dot.set(w0.getX(), w0.getY(), w0.getZ(), 0.0);
      q1Dot.set(w1.getX(), w1.getY(), w1.getZ(), 0.0);

      tempW.inverse(q0);
      tempW.mul(q1);
      computeDerivativeQuaternionDifference(q0, q0Dot, q1, q1Dot, tempWDot);
      
      computePowerQuaternion(tempW, alpha, tempWPowered);
      computeDerivativePoweredQuaternion(tempW, tempWDot, alpha, alphaDot, tempWPoweredDot);
      
      computeDerivativeQuaternionProduct(q0, q0Dot, tempWPowered, tempWPoweredDot, tempResult3);
      resultToPack.set(tempResult3.getX(), tempResult3.getY(), tempResult3.getZ());
   }
   
   private final Quat4d tempW = new Quat4d();
   private final Quat4d tempWDot = new Quat4d();
   private final Quat4d tempWPowered = new Quat4d();
   private final Quat4d tempWPoweredDot = new Quat4d();

   public void getDesiredAngularVelocityDirection(FrameVector angularVelocityDirectionToPack)
   {
      desiredAngularVelocityDirection.getFrameTupleIncludingFrame(angularVelocityDirectionToPack);
   }

   public void getInitialOrientationDrifted(FrameOrientation initialOrientationDriftedToPack)
   {
      initialOrientationDrifted.getFrameOrientationIncludingFrame(initialOrientationDriftedToPack);
   }

   public void getFinalOrientationDrifted(FrameOrientation finalOrientationDriftedToPack)
   {
      finalOrientationDrifted.getFrameOrientationIncludingFrame(finalOrientationDriftedToPack);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void get(FrameOrientation orientationToPack)
   {
      desiredOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   public void packAngularVelocity(FrameVector velocityToPack)
   {
      desiredAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void packAngularAcceleration(FrameVector accelerationToPack)
   {
      desiredAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      get(orientationToPack);
      packAngularVelocity(angularVelocityToPack);
      packAngularAcceleration(angularAccelerationToPack);
   }
}
