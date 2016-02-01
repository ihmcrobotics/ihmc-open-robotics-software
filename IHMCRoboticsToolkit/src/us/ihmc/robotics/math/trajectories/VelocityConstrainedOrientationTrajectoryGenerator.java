package us.ihmc.robotics.math.trajectories;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
/**
 * TODO: Setting final velocity constraints (angular and linear) does not work.
 * @author unknownid
 *
 */
public class VelocityConstrainedOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final YoPolynomial xPolynomial;
   private final YoPolynomial yPolynomial;
   private final YoPolynomial zPolynomial;

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameVector initialAngularVelocity;
   private final YoFrameQuaternion finalOrientation;
   private final YoFrameVector finalAngularVelocity;

   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector desiredAngularAcceleration;

   private final DoubleProvider trajectoryTimeProvider;
   private final OrientationProvider initialOrientationProvider;
   private final VectorProvider initialAngularVelocityProvider;
   private final OrientationProvider finalOrientationProvider;
   private final VectorProvider finalAngularVelocityProvider;

   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();
   private final Vector3d tempDeltaOrientation = new Vector3d();
   private final FrameOrientation tempOrientation;
   private final FrameOrientation tempInitialOrientation;
   private final FrameOrientation tempFinalOrientation;
   private final FrameVector tempInitialVelocity;
   private final FrameVector tempFinalVelocity;
   private final FrameVector tempVector;

   private final ReferenceFrame frameAtInitialOrientation;
   private final ReferenceFrame trajectoryFrame;

   public VelocityConstrainedOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
         OrientationProvider initialOrientationProvider, VectorProvider initialAngularVelocityProvider, OrientationProvider finalOrientationProvider,
         VectorProvider finalAngularVelocityProvider, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      this.xPolynomial = new YoPolynomial(namePrefix + "ParameterXPolynomial", 6, registry);
      this.yPolynomial = new YoPolynomial(namePrefix + "ParameterYPolynomial", 6, registry);
      this.zPolynomial = new YoPolynomial(namePrefix + "ParameterZPolynomial", 6, registry);
      this.trajectoryFrame = referenceFrame;

      this.initialOrientation = new YoFrameQuaternion(namePrefix + "InitialOrientation", trajectoryFrame, registry);
      this.initialAngularVelocity = new YoFrameVector(namePrefix + "InitialAngularVelocity", trajectoryFrame, registry);
      this.finalOrientation = new YoFrameQuaternion(namePrefix + "FinalOrientation", trajectoryFrame, registry);
      this.finalAngularVelocity = new YoFrameVector(namePrefix + "FinalAngularVelocity", trajectoryFrame, registry);

      this.desiredOrientation = new YoFrameQuaternion(namePrefix + "DesiredOrientation", trajectoryFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector(namePrefix + "DesiredAngularVelocity", trajectoryFrame, registry);
      this.desiredAngularAcceleration = new YoFrameVector(namePrefix + "DesiredAngularAcceleration", trajectoryFrame, registry);

      this.trajectoryTimeProvider = trajectoryTimeProvider;
      this.initialOrientationProvider = initialOrientationProvider;
      this.initialAngularVelocityProvider = initialAngularVelocityProvider;
      this.finalOrientationProvider = finalOrientationProvider;
      this.finalAngularVelocityProvider = finalAngularVelocityProvider;

      tempOrientation = new FrameOrientation(trajectoryFrame);
      tempInitialOrientation = new FrameOrientation(trajectoryFrame);
      tempFinalOrientation = new FrameOrientation(trajectoryFrame);
      tempInitialVelocity = new FrameVector(trajectoryFrame);
      tempFinalVelocity = new FrameVector(trajectoryFrame);
      tempVector = new FrameVector(trajectoryFrame);

      frameAtInitialOrientation = new ReferenceFrame("OrientationTrajectoryFrame", trajectoryFrame)
      {
         private static final long serialVersionUID = 8840430180170220404L;
         private final Matrix3d localMatrix = new Matrix3d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            initialOrientation.get(localMatrix);
            transformToParent.setRotation(localMatrix);
         }
      };

      parentRegistry.addChild(registry);
   }

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

      frameAtInitialOrientation.update();
      tempFinalOrientation.changeFrame(frameAtInitialOrientation);
      tempFinalOrientation.getAxisAngle(tempAxisAngle);
      tempInitialVelocity.changeFrame(frameAtInitialOrientation);
      tempFinalVelocity.changeFrame(frameAtInitialOrientation);

      tempDeltaOrientation.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      tempDeltaOrientation.scale(tempAxisAngle.getAngle());

      xPolynomial.setQuintic(0.0, trajectoryTime, 0.0, tempInitialVelocity.getX(), 0.0, tempDeltaOrientation.getX(), tempFinalVelocity.getX(), 0.0);
      yPolynomial.setQuintic(0.0, trajectoryTime, 0.0, tempInitialVelocity.getY(), 0.0, tempDeltaOrientation.getY(), tempFinalVelocity.getY(), 0.0);
      zPolynomial.setQuintic(0.0, trajectoryTime, 0.0, tempInitialVelocity.getZ(), 0.0, tempDeltaOrientation.getZ(), tempFinalVelocity.getZ(), 0.0);

      desiredOrientation.set(initialOrientation);
      desiredAngularVelocity.setAndMatchFrame(tempInitialVelocity);
      desiredAngularAcceleration.setToZero();
   }

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
      xPolynomial.compute(time);
      yPolynomial.compute(time);
      zPolynomial.compute(time);

      tempDeltaOrientation.set(xPolynomial.getPosition(), yPolynomial.getPosition(), zPolynomial.getPosition());

      double angle = tempDeltaOrientation.length();
      if (Math.abs(angle) < 1e-5)
         tempDeltaOrientation.set(1.0, 0.0, 0.0);
      else
         tempDeltaOrientation.scale(1.0 / angle);
      tempAxisAngle.set(tempDeltaOrientation, angle);
      tempOrientation.setIncludingFrame(frameAtInitialOrientation, tempAxisAngle);
      tempOrientation.changeFrame(trajectoryFrame);
      desiredOrientation.set(tempOrientation);

      tempVector.setIncludingFrame(frameAtInitialOrientation, xPolynomial.getVelocity(), yPolynomial.getVelocity(), zPolynomial.getVelocity());
      desiredAngularVelocity.setAndMatchFrame(tempVector);

      tempVector.setIncludingFrame(frameAtInitialOrientation, xPolynomial.getAcceleration(), yPolynomial.getAcceleration(), zPolynomial.getAcceleration());
      desiredAngularAcceleration.setAndMatchFrame(tempVector);
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
