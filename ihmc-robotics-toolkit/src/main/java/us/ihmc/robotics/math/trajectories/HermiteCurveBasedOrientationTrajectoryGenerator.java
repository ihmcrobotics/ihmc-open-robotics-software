package us.ihmc.robotics.math.trajectories;

import static us.ihmc.commons.MathTools.square;

import org.apache.commons.math3.util.Precision;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameSO3TrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This trajectory generator aims at interpolating between two orientations qa and qb for given
 * angular velocities at the limits wa and wb. The method used here differs from the trajectory
 * generator implemented in {@link VelocityConstrainedOrientationTrajectoryGenerator}. It seems that
 * the approach used here is a better fit for interpolating between waypoints.
 * <p>
 * I basically implemented the method called Hermite Quaternion Curve that is presented in the
 * following paper: <a href="http://azrael.digipen.edu/MAT351/papers/Kim2.pdf"> PDF link 1</a>,
 * <a href="http://graphics.cs.cmu.edu/nsp/course/15-464/Fall05/papers/kimKimShin.pdf"> PDF link
 * 2</a>, <a href=
 * "https://www.researchgate.net/publication/2388093_A_General_Construction_Scheme_for_Unit_Quaternion_Curves_with_Simple_High_Order_Derivatives">
 * ResearchGate link</a>.
 * </p>
 * <p>
 * Because in the paper the authors are using a specific type of quaternions, the expressions had to
 * be redefined for the general definition of unit-quaternions. Also, the expression for the angular
 * acceleration had to be derived. See the following Word document for more details:
 * <a href="https://1drv.ms/b/s!AtjeMRpLgFtkiP0oG_0fhZEg3cPhhA">Hermite Quaternion Curve
 * Revisited</a>.
 * </p>
 *
 * @author Sylvain
 */
public class HermiteCurveBasedOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;
   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final YoInteger numberOfRevolutions;

   private final YoDouble[] cumulativeBeziers;
   private final YoDouble[] cumulativeBeziersDot;
   private final YoDouble[] cumulativeBeziersDDot;

   private final YoFrameVector[] controlRotations;

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameVector finalAngularVelocity;

   private final YoFrameQuaternion currentOrientation;
   private final YoFrameVector currentAngularVelocity;
   private final YoFrameVector currentAngularAcceleration;

   private final ReferenceFrame trajectoryFrame;

   public HermiteCurveBasedOrientationTrajectoryGenerator(String name, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(name, false, referenceFrame, parentRegistry);
   }

   public HermiteCurveBasedOrientationTrajectoryGenerator(String name, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
                                                          YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      registry = new YoVariableRegistry(name);
      trajectoryTime = new YoDouble(name + "TrajectoryTime", registry);
      numberOfRevolutions = new YoInteger(name + "NumberOfRevolutions", registry);
      currentTime = new YoDouble(name + "Time", registry);
      trajectoryFrame = referenceFrame;

      cumulativeBeziers = new YoDouble[4];
      cumulativeBeziersDot = new YoDouble[4];
      cumulativeBeziersDDot = new YoDouble[4];
      controlRotations = new YoFrameVector[4];

      for (int i = 1; i <= 3; i++)
      {
         cumulativeBeziers[i] = new YoDouble(name + "CumulativeBezier" + i, registry);
         cumulativeBeziersDot[i] = new YoDouble(name + "CumulativeBezierDot" + i, registry);
         cumulativeBeziersDDot[i] = new YoDouble(name + "CumulativeBezierDDot" + i, registry);
      }

      String initialOrientationName = "InitialOrientation";
      String initialAngularVelocityName = "InitialAngularVelocity";
      String finalOrientationName = "FinalOrientation";
      String finalAngularVelocityName = "FinalAngularVelocity";
      String currentOrientationName = "CurrentOrientation";
      String currentAngularVelocityName = "CurrentAngularVelocity";
      String currentAngularAccelerationName = "CurrentAngularAcceleration";
      String controlRotationsName = "ControlRotations";

      if (allowMultipleFrames)
      {
         YoFrameQuaternionInMultipleFrames initialOrientation = new YoFrameQuaternionInMultipleFrames(name + initialOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames initialAngularVelocity = new YoFrameVectorInMultipleFrames(name + initialAngularVelocityName, registry, trajectoryFrame);
         YoFrameQuaternionInMultipleFrames finalOrientation = new YoFrameQuaternionInMultipleFrames(name + finalOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames finalAngularVelocity = new YoFrameVectorInMultipleFrames(name + finalAngularVelocityName, registry, trajectoryFrame);

         YoFrameQuaternionInMultipleFrames currentOrientation = new YoFrameQuaternionInMultipleFrames(name + currentOrientationName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularVelocity = new YoFrameVectorInMultipleFrames(name + currentAngularVelocityName, registry, trajectoryFrame);
         YoFrameVectorInMultipleFrames currentAngularAcceleration = new YoFrameVectorInMultipleFrames(name + currentAngularAccelerationName, registry,
                                                                                                      trajectoryFrame);

         registerMultipleFramesHolders(initialOrientation, initialAngularVelocity);
         registerMultipleFramesHolders(finalOrientation, finalAngularVelocity);
         registerMultipleFramesHolders(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         this.initialOrientation = initialOrientation;
         this.initialAngularVelocity = initialAngularVelocity;
         this.finalOrientation = finalOrientation;
         this.finalAngularVelocity = finalAngularVelocity;
         this.currentOrientation = currentOrientation;
         this.currentAngularVelocity = currentAngularVelocity;
         this.currentAngularAcceleration = currentAngularAcceleration;

         for (int i = 1; i <= 3; i++)
         {
            YoFrameVectorInMultipleFrames controlRotation = new YoFrameVectorInMultipleFrames(name + controlRotationsName + i, registry, trajectoryFrame);
            registerMultipleFramesHolders(controlRotation);
            controlRotations[i] = controlRotation;
         }
      }
      else
      {
         initialOrientation = new YoFrameQuaternion(name + initialOrientationName, trajectoryFrame, registry);
         initialAngularVelocity = new YoFrameVector(name + initialAngularVelocityName, trajectoryFrame, registry);
         finalOrientation = new YoFrameQuaternion(name + finalOrientationName, trajectoryFrame, registry);
         finalAngularVelocity = new YoFrameVector(name + finalAngularVelocityName, trajectoryFrame, registry);

         currentOrientation = new YoFrameQuaternion(name + currentOrientationName, trajectoryFrame, registry);
         currentAngularVelocity = new YoFrameVector(name + currentAngularVelocityName, trajectoryFrame, registry);
         currentAngularAcceleration = new YoFrameVector(name + currentAngularAccelerationName, trajectoryFrame, registry);

         for (int i = 1; i <= 3; i++)
            controlRotations[i] = new YoFrameVector(name + controlRotationsName + i, trajectoryFrame, registry);
      }

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double duration)
   {
      MathTools.checkIntervalContains(duration, 0.0, Double.POSITIVE_INFINITY);
      trajectoryTime.set(duration);
   }

   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public void setInitialOrientation(FrameQuaternion initialOrientation)
   {
      tempOrientation.setIncludingFrame(initialOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.initialOrientation.set(tempOrientation);
   }

   public void setInitialOrientation(YoFrameQuaternion initialOrientation)
   {
      initialOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.initialOrientation.set(tempOrientation);
   }

   public void setFinalOrientation(FrameQuaternion finalOrientation)
   {
      tempOrientation.setIncludingFrame(finalOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.finalOrientation.set(tempOrientation);
   }

   public void setFinalOrientation(FramePose finalPose)
   {
      finalPose.getOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      finalOrientation.set(tempOrientation);
   }

   public void setFinalOrientation(YoFrameQuaternion finalOrientation)
   {
      finalOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      this.finalOrientation.set(tempOrientation);
   }

   public void setInitialAngularVelocity(FrameVector3D initialAngularVelocity)
   {
      this.initialAngularVelocity.setAndMatchFrame(initialAngularVelocity);
   }

   public void setInitialAngularVelocity(YoFrameVector initialAngularVelocity)
   {
      this.initialAngularVelocity.setAndMatchFrame(initialAngularVelocity);
   }

   public void setFinalAngularVelocity(FrameVector3D finalAngularVelocity)
   {
      this.finalAngularVelocity.setAndMatchFrame(finalAngularVelocity);
   }

   public void setFinalAngularVelocity(YoFrameVector finalAngularVelocity)
   {
      this.finalAngularVelocity.setAndMatchFrame(finalAngularVelocity);
   }

   public void setInitialVelocityToZero()
   {
      initialAngularVelocity.setToZero();
   }

   public void setFinalVelocityToZero()
   {
      finalAngularVelocity.setToZero();
   }

   /**
    * Sets an integer the desired number of revolutions to be achieved between the endpoints of this
    * trajectory.
    */
   public void setNumberOfRevolutions(int numberOfRevolutions)
   {
      this.numberOfRevolutions.set(numberOfRevolutions);
   }

   public void setInitialConditions(FrameQuaternion initialOrientation, FrameVector3D initialAngularVelocity)
   {
      setInitialOrientation(initialOrientation);
      setInitialAngularVelocity(initialAngularVelocity);
   }

   public void setInitialConditions(YoFrameQuaternion initialOrientation, YoFrameVector initialAngularVelocity)
   {
      setInitialOrientation(initialOrientation);
      setInitialAngularVelocity(initialAngularVelocity);
   }

   public void setFinalConditions(FrameQuaternion finalOrientation, FrameVector3D finalAngularVelocity)
   {
      setFinalOrientation(finalOrientation);
      setFinalAngularVelocity(finalAngularVelocity);
   }

   public void setFinalConditions(YoFrameQuaternion finalOrientation, YoFrameVector finalAngularVelocity)
   {
      setFinalOrientation(finalOrientation);
      setFinalAngularVelocity(finalAngularVelocity);
   }

   private final FrameQuaternion tempFrameOrientation = new FrameQuaternion();
   private final FrameVector3D tempFrameVector = new FrameVector3D();

   public void setTrajectoryParameters(FrameSO3TrajectoryPoint initialFrameSO3Waypoint, FrameSO3TrajectoryPoint finalFrameSO3Waypoint)
   {
      //TODO: Clean this up by making an initialSO3Waypoint, finalSO3Waypoint as YoFrameSO3Waypoints.
      setTrajectoryTime(finalFrameSO3Waypoint.getTime() - initialFrameSO3Waypoint.getTime());

      initialFrameSO3Waypoint.getOrientationIncludingFrame(tempFrameOrientation);
      initialFrameSO3Waypoint.getAngularVelocityIncludingFrame(tempFrameVector);

      initialOrientation.set(tempFrameOrientation);
      initialAngularVelocity.set(tempFrameVector);

      finalFrameSO3Waypoint.getOrientationIncludingFrame(tempFrameOrientation);
      finalFrameSO3Waypoint.getAngularVelocityIncludingFrame(tempFrameVector);

      finalOrientation.set(tempFrameOrientation);
      finalAngularVelocity.set(tempFrameVector);
   }

   public void setTrajectoryParameters(YoFrameSO3TrajectoryPoint initialYoFrameSO3Waypoint, YoFrameSO3TrajectoryPoint finalYoFrameSO3Waypoint)
   {
      setTrajectoryTime(finalYoFrameSO3Waypoint.getTime() - initialYoFrameSO3Waypoint.getTime());

      initialOrientation.set(initialYoFrameSO3Waypoint.getOrientation());
      initialAngularVelocity.set(initialYoFrameSO3Waypoint.getAngularVelocity());

      finalOrientation.set(finalYoFrameSO3Waypoint.getOrientation());
      finalAngularVelocity.set(finalYoFrameSO3Waypoint.getAngularVelocity());
   }

   /**
    * Before Initializing the isSolvable() method should be called to confirm that the limit
    * conditions are within the bounds
    */
   @Override
   public void initialize()
   {
      if (initialOrientation.dot(finalOrientation) < 0.0)
         finalOrientation.negate();

      updateControlQuaternions();
      compute(0.0);
   }

   private final Quaternion tempQuaternion = new Quaternion();
   private final Vector3D wa = new Vector3D();
   private final Vector3D wb = new Vector3D();
   private final Vector3D delta = new Vector3D();

   private void updateControlQuaternions()
   {
      double TOverThree = trajectoryTime.getDoubleValue() / 3.0;

      FrameQuaternion qa = initialOrientation.getFrameOrientation();
      FrameQuaternion qb = finalOrientation.getFrameOrientation();
      initialAngularVelocity.get(wa);
      finalAngularVelocity.get(wb);
      qa.inverseTransform(wa);
      qb.inverseTransform(wb);

      // delta1 = wa * T / 3.0
      delta.setAndScale(TOverThree, wa);
      controlRotations[1].set(delta);

      // delta2 = log( exp(-wa*T/3) * qa^-1 * qb * exp(-wb*T/3) )
      tempQuaternion.difference(qa, qb);
      tempQuaternion.preMultiply(exp(-TOverThree, wa));
      tempQuaternion.multiply(exp(-TOverThree, wb));
      controlRotations[2].set(log(tempQuaternion));

      if (numberOfRevolutions.getIntegerValue() != 0)
      {
         controlRotations[2].get(delta);
         if (delta.lengthSquared() > 1.0e-10)
         {
            delta.normalize();
            delta.scale(numberOfRevolutions.getIntegerValue() * 2.0 * Math.PI);
            controlRotations[2].add(delta);
         }
      }

      // delta3 = wb * T / 3.0
      delta.setAndScale(TOverThree, wb);
      controlRotations[3].set(delta);
   }

   private final Quaternion qInterpolated = new Quaternion();
   private final Vector3D angularVelocityInterpolated = new Vector3D();
   private final Vector3D angularAccelerationInterpolated = new Vector3D();

   @Override
   public void compute(double time)
   {
      if (Double.isNaN(time))
      {
         throw new RuntimeException("Can not call compute on trajectory generator with time NaN.");
      }

      currentTime.set(time);

      if (time < 0.0)
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
         return;
      }
      if (time > trajectoryTime.getDoubleValue())
      {
         currentOrientation.set(finalOrientation);
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
         return;
      }

      if (Precision.equals(0.0, trajectoryTime.getDoubleValue()))
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.set(initialAngularVelocity);
         currentAngularAcceleration.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      computeBezierBasedCurve(time, qInterpolated, angularVelocityInterpolated, angularAccelerationInterpolated);

      currentOrientation.set(qInterpolated);
      currentAngularVelocity.set(angularVelocityInterpolated);
      currentAngularAcceleration.set(angularAccelerationInterpolated);
   }

   private void updateBezierCoefficients(double t)
   {
      double T = trajectoryTime.getDoubleValue();
      double oneOverT = 1.0 / T;
      double tOverT = t * oneOverT;

      cumulativeBeziers[1].set(1.0 - cube(1.0 - tOverT));
      cumulativeBeziers[2].set(3.0 * square(tOverT) - 2.0 * cube(tOverT));
      cumulativeBeziers[3].set(cube(tOverT));

      cumulativeBeziersDot[1].set(3.0 * oneOverT * square(1.0 - tOverT));
      cumulativeBeziersDot[2].set(6.0 * tOverT * oneOverT * (1.0 - tOverT));
      cumulativeBeziersDot[3].set(3.0 * square(tOverT) * oneOverT);

      cumulativeBeziersDDot[1].set(-6.0 * square(oneOverT) * (1.0 - tOverT));
      cumulativeBeziersDDot[2].set(6.0 * square(oneOverT) * (1.0 - 2.0 * tOverT));
      cumulativeBeziersDDot[3].set(6.0 * t * cube(oneOverT));
   }

   private static double cube(double value)
   {
      return value * value * value;
   }

   private final Quaternion qProduct = new Quaternion();

   private final Vector4D qDot = new Vector4D();
   private final Vector4D qDot1 = new Vector4D();
   private final Vector4D qDot2 = new Vector4D();
   private final Vector4D qDot3 = new Vector4D();
   private final Vector4D qProductDot = new Vector4D();

   private final Vector4D qDDot = new Vector4D();
   private final Vector4D qDDot1 = new Vector4D();
   private final Vector4D qDDot2 = new Vector4D();
   private final Vector4D qDDot3 = new Vector4D();
   private final Vector4D qDDotTemp = new Vector4D();
   private final Vector4D qProductDDot = new Vector4D();

   private final Vector3D d1 = new Vector3D();
   private final Vector3D d2 = new Vector3D();
   private final Vector3D d3 = new Vector3D();

   private final Quaternion expD1B1 = new Quaternion();
   private final Quaternion expD2B2 = new Quaternion();
   private final Quaternion expD1B1_expD2B2 = new Quaternion();
   private final Quaternion expD3B3 = new Quaternion();

   private final Vector4D d1B1Dot = new Vector4D();
   private final Vector4D d2B2Dot = new Vector4D();
   private final Vector4D d3B3Dot = new Vector4D();

   private final Vector4D d1B1DDot = new Vector4D();
   private final Vector4D d2B2DDot = new Vector4D();
   private final Vector4D d3B3DDot = new Vector4D();

   private void computeBezierBasedCurve(double time, QuaternionBasics q, Vector3D angularVelocity, Vector3D angularAcceleration)
   {
      updateBezierCoefficients(time);

      // Changing naming convention to make expressions smaller
      QuaternionReadOnly q0 = initialOrientation.getFrameOrientation().getQuaternion();
      controlRotations[1].get(d1);
      controlRotations[2].get(d2);
      controlRotations[3].get(d3);

      // Update intermediate variables
      expD1B1.set(exp(cumulativeBeziers[1].getDoubleValue(), d1));
      expD2B2.set(exp(cumulativeBeziers[2].getDoubleValue(), d2));
      expD3B3.set(exp(cumulativeBeziers[3].getDoubleValue(), d3));
      expD1B1_expD2B2.set(expD1B1);
      expD1B1_expD2B2.multiply(expD2B2);

      // In page 1, the authors say they use a specific type of quaternion for which
      // exp(theta * u) = (cos(theta), u * sin(theta)) instead of:
      // exp(theta * u) = (cos(theta/2), u * sin(theta/2)).
      // Because of the latter, the quaternion derivatives in the paper are actually wrong.
      // When derivating the exponent of an exponential term, it should be as follows:
      // d/dt(exp(alpha(t) * u)) = 0.5 * alphaDot(t) * u * exp(alpha(t) * u)
      d1B1Dot.set(d1);
      d2B2Dot.set(d2);
      d3B3Dot.set(d3);
      d1B1Dot.scale(0.5 * cumulativeBeziersDot[1].getDoubleValue());
      d2B2Dot.scale(0.5 * cumulativeBeziersDot[2].getDoubleValue());
      d3B3Dot.scale(0.5 * cumulativeBeziersDot[3].getDoubleValue());

      d1B1DDot.set(d1);
      d2B2DDot.set(d2);
      d3B3DDot.set(d3);
      d1B1DDot.scale(0.5 * cumulativeBeziersDDot[1].getDoubleValue());
      d2B2DDot.scale(0.5 * cumulativeBeziersDDot[2].getDoubleValue());
      d3B3DDot.scale(0.5 * cumulativeBeziersDDot[3].getDoubleValue());

      // Calculate qStar = exp(w1*B1) * exp(w2*B2) * exp(w3*B3)
      qProduct.set(expD1B1);
      qProduct.multiply(expD2B2);
      qProduct.multiply(expD3B3);

      // Calculate qStarDot = qDot1 + qDot2 + qDot3, with:
      // qDot1 = ((w1*BDot1) * qStar
      QuaternionTools.multiply(d1B1Dot, qProduct, qDot1);
      // qDot2 = exp(w1*B1) * exp(w2*B2) * (w2*BDot2) * exp(w3*B3)
      QuaternionTools.multiply(expD1B1_expD2B2, d2B2Dot, qDot2);
      QuaternionTools.multiply(qDot2, expD3B3, qDot2);
      // qDot3 = qStar * (w3*BDot3)
      QuaternionTools.multiply(qProduct, d3B3Dot, qDot3);
      // Now qStarDot
      qProductDot.add(qDot1, qDot2);
      qProductDot.add(qDot3);

      // Calculate qStarDDot:
      // qStarDDot = qDDot1 + qDDot2 + qDDot3
      // qDDot1 = (w1*BDDot1) * qStar + (w1*BDot1) * qStarDot
      QuaternionTools.multiply(d1B1DDot, qProduct, qDDotTemp);
      QuaternionTools.multiply(d1B1Dot, qProductDot, qDDot1);
      qDDot1.add(qDDotTemp);

      // qDDot2 = exp(w1*B1)*exp(w2*B2)*{ (w2*BDDot2) + (w2*BDot2)*(w2*BDot2) }*exp(w3*B3) + (w1*BDot1)*qDot2 + qDot2*(w3*BDot3)
      QuaternionTools.multiply(d2B2Dot, d2B2Dot, qDDot2);
      qDDot2.add(d2B2DDot);
      QuaternionTools.multiply(expD1B1_expD2B2, qDDot2, qDDotTemp);
      QuaternionTools.multiply(qDDotTemp, expD3B3, qDDotTemp);
      qDDot2.set(qDDotTemp);
      QuaternionTools.multiply(d1B1Dot, qDot2, qDDotTemp);
      qDDot2.add(qDDotTemp);
      QuaternionTools.multiply(qDot2, d3B3Dot, qDDotTemp);
      qDDot2.add(qDDotTemp);

      // qDDot3 = qStar * (w3*BDDot3) + qStarDot * (w3*BDot3)
      QuaternionTools.multiply(qProduct, d3B3DDot, qDDotTemp);
      QuaternionTools.multiply(qProductDot, d3B3Dot, qDDot3);
      qDDot3.add(qDDotTemp);

      // Now qStarDDot
      qProductDDot.add(qDDot1, qDDot2);
      qProductDDot.add(qDDot3);

      q.multiply(q0, qProduct);
      QuaternionTools.multiply(q0, qProductDot, qDot);
      QuaternionTools.multiply(q0, qProductDDot, qDDot);

      angularVelocity.set(convertToAngularVelocity(q, qDot));
      angularAcceleration.set(convertToAngularAcceleration(q, qDot, qDDot));
   }

   private final Quaternion tempLogExpQuaternion = new Quaternion();
   private final Vector3D tempLogExpVector3D = new Vector3D();

   private QuaternionReadOnly exp(double alpha, Vector3DReadOnly rotation)
   {
      tempLogExpVector3D.setAndScale(alpha, rotation);
      tempLogExpQuaternion.set(tempLogExpVector3D);
      return tempLogExpQuaternion;
   }

   private Vector3DReadOnly log(QuaternionReadOnly q)
   {
      q.get(tempLogExpVector3D);
      return tempLogExpVector3D;
   }

   private final Vector3D tempConvertVector3D = new Vector3D();
   private final Vector4D tempConvertVector4D = new Vector4D();

   private Vector3DReadOnly convertToAngularVelocity(QuaternionReadOnly q, Vector4DReadOnly qDot)
   {// w = qDot * q^-1
      QuaternionTools.multiplyConjugateRight(qDot, q, tempConvertVector4D);
      tempConvertVector3D.setX(tempConvertVector4D.getX());
      tempConvertVector3D.setY(tempConvertVector4D.getY());
      tempConvertVector3D.setZ(tempConvertVector4D.getZ());
      tempConvertVector3D.scale(2.0);
      return tempConvertVector3D;
   }

   private Vector3DReadOnly convertToAngularAcceleration(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector4DReadOnly qDDot)
   { // w = qDDot * q^-1 + qDot * qDot^-1
      QuaternionTools.multiplyConjugateRight(qDDot, q, tempConvertVector4D);
      tempConvertVector3D.setX(tempConvertVector4D.getX());
      tempConvertVector3D.setY(tempConvertVector4D.getY());
      tempConvertVector3D.setZ(tempConvertVector4D.getZ());

      QuaternionTools.multiplyConjugateRight(qDot, qDot, tempConvertVector4D);
      tempConvertVector3D.addX(tempConvertVector4D.getX());
      tempConvertVector3D.addY(tempConvertVector4D.getY());
      tempConvertVector3D.addZ(tempConvertVector4D.getZ());
      tempConvertVector3D.scale(2.0);
      return tempConvertVector3D;
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      currentOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   public void getAngularVelocity(FrameVector3D velocityToPack)
   {
      currentAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D accelerationToPack)
   {
      currentAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public String toString()
   {
      String ret = "";

      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent orientation: " + currentOrientation.toString();
      ret += "\nCurrent angular velocity: " + currentAngularVelocity.toString();
      ret += "\nCurrent angular acceleration: " + currentAngularAcceleration.toString();
      return ret;
   }
}
