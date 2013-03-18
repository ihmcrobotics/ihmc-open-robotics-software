package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class SimulatedLinearAccelerationSensor extends SimulatedSensor<Vector3d>
{
   private final RigidBody rigidBody;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FramePoint imuFramePoint = new FramePoint(worldFrame);

   private final FrameVector linearAccelerationFrameVector = new FrameVector(worldFrame);
   private final Vector3d linearAcceleration = new Vector3d();
   private final Twist twist = new Twist();
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();

   private final ReferenceFrame measurementFrame;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final ControlFlowOutputPort<Vector3d> linearAccelerationOutputPort = createOutputPort();

   public SimulatedLinearAccelerationSensor(RigidBody rigidBody, ReferenceFrame measurementFrame, TwistCalculator twistCalculator,
           SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this.rigidBody = rigidBody;
      this.measurementFrame = measurementFrame;
      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;
   }

   public void startComputation()
   {
      twistCalculator.packTwistOfBody(twist, rigidBody);
      spatialAccelerationCalculator.packAccelerationOfBody(spatialAcceleration, rigidBody);

      spatialAcceleration.changeFrame(worldFrame, twist, twist);
      twist.changeFrame(worldFrame);
      imuFramePoint.setToZero(measurementFrame);
      imuFramePoint.changeFrame(worldFrame);
      spatialAcceleration.packAccelerationOfPointFixedInBodyFrame(twist, imuFramePoint, linearAccelerationFrameVector);
      linearAccelerationFrameVector.changeFrame(measurementFrame);
      linearAccelerationFrameVector.getVector(linearAcceleration);

      corrupt(linearAcceleration);

      linearAccelerationOutputPort.setData(linearAcceleration);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Vector3d> getLinearAccelerationOutputPort()
   {
      return linearAccelerationOutputPort;
   }
}
