package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Vector3d;

import org.apache.commons.lang.ArrayUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.OriginAndPointFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.AxisAngleOrientationController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientationInMultipleFrames;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePointInMultipleFrames;

public class HeadOrientationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GeometricJacobian neckJacobian;
   private final TwistCalculator twistCalculator;
   private final RigidBody elevator;
   private final ReferenceFrame elevatorFrame;
   private final AxisAngleOrientationController headOrientationController;
   private final DenseMatrix64F selectionMatrix;
   private final SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector();

   private final YoFrameOrientationInMultipleFrames orientationToTrack;
   private final YoFramePointInMultipleFrames pointToTrack;
   private final ReferenceFrame[] framesToTrackIn;
   private final OriginAndPointFrame pointTrackingFrame;

   private final IntegerYoVariable trackingFrameIndex = new IntegerYoVariable("trackingFrameIndex", registry);
   private final EnumYoVariable<HeadTrackingMode> headTrackingMode = EnumYoVariable.create("headTrackingMode", HeadTrackingMode.class, registry);

   public HeadOrientationControlModule(GeometricJacobian neckJacobian, TwistCalculator twistCalculator, RigidBody chest,
           YoVariableRegistry parentRegistry)
   {
      this.neckJacobian = neckJacobian;
      this.twistCalculator = twistCalculator;
      elevator = twistCalculator.getRootBody();
      elevatorFrame = elevator.getBodyFixedFrame();

      selectionMatrix = new DenseMatrix64F(neckJacobian.getNumberOfColumns(), Twist.SIZE);

      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      pointTrackingFrame = new OriginAndPointFrame("headTrackingFrame", neckJacobian.getEndEffectorFrame());

      framesToTrackIn = new ReferenceFrame[] {chestFrame, elevatorFrame};

      orientationToTrack = new YoFrameOrientationInMultipleFrames("headOrientationToTrack", framesToTrackIn, registry);
      pointToTrack = new YoFramePointInMultipleFrames("headPointToTrack", framesToTrackIn, registry);

      headOrientationController = new AxisAngleOrientationController("headOrientationController", neckJacobian.getEndEffectorFrame(), registry);

      headTrackingMode.set(HeadTrackingMode.ORIENTATION);
      trackingFrameIndex.set(getTrackingFrameIndex(chestFrame));
      
      parentRegistry.addChild(registry);
   }

   public ReferenceFrame getElevatorFrame()
   {
      return elevatorFrame;
   }

   public void compute()
   {
      neckJacobian.compute();
      FrameOrientation desiredHeadOrientation = getDesiredFrameOrientation();

      Twist twistOfHeadWithRespectToElevator = new Twist();
      twistCalculator.packRelativeTwist(twistOfHeadWithRespectToElevator, elevator, neckJacobian.getEndEffector());
      FrameVector currentAngularVelocity = new FrameVector(neckJacobian.getEndEffectorFrame(), twistOfHeadWithRespectToElevator.getAngularPartCopy());

      ReferenceFrame frameToTrackIn = framesToTrackIn[trackingFrameIndex.getIntegerValue()];
      FrameVector desiredAngularVelocity = new FrameVector(frameToTrackIn);
      FrameVector desiredAngularAccelerationFeedForward = new FrameVector(frameToTrackIn);
      headOrientationController.compute(desiredAngularAccelerationFeedForward, desiredHeadOrientation, desiredAngularVelocity, currentAngularVelocity,
                                        new FrameVector(elevatorFrame));

      spatialAcceleration.set(neckJacobian.getEndEffectorFrame(), elevatorFrame, desiredAngularAccelerationFeedForward.getReferenceFrame(), new Vector3d(),
                              desiredAngularAccelerationFeedForward.getVector());

      computeSelectionMatrix(neckJacobian, selectionMatrix);
   }

   private FrameOrientation getDesiredFrameOrientation()
   {
      ReferenceFrame referenceFrame = framesToTrackIn[trackingFrameIndex.getIntegerValue()];
      switch (headTrackingMode.getEnumValue())
      {
         case ORIENTATION :
         {
            return orientationToTrack.getOrientationInFrame(referenceFrame).getFrameOrientationCopy();
         }

         case POINT :
         {
            FramePoint positionToPointAt = pointToTrack.getPointInFrame(referenceFrame).getFramePointCopy();
            pointTrackingFrame.setPositionToPointAt(positionToPointAt);
            pointTrackingFrame.update();

            return new FrameOrientation(pointTrackingFrame);
         }

         default :
            throw new RuntimeException("Case " + headTrackingMode.getEnumValue() + " not handled.");
      }
   }

   private static void computeSelectionMatrix(GeometricJacobian jacobian, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      CommonOps.pinv(jacobianMatrix, selectionMatrix);

      for (int i = 0; i < selectionMatrix.getNumRows(); i++)
      {
         Vector3d angularPart = new Vector3d(selectionMatrix.get(i, 0), selectionMatrix.get(i, 1), selectionMatrix.get(i, 2));
         angularPart.normalize();

         selectionMatrix.set(i, 0, angularPart.getX());
         selectionMatrix.set(i, 1, angularPart.getY());
         selectionMatrix.set(i, 2, angularPart.getZ());

         for (int j = 3; j < 6; j++)
         {
            selectionMatrix.set(i, j, 0.0);
         }
      }
   }

   public SpatialAccelerationVector getSpatialAcceleration()
   {
      return spatialAcceleration;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      headOrientationController.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      headOrientationController.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setOrientationToTrack(FrameOrientation orientation, ReferenceFrame frameToTrackIn)
   {
      this.trackingFrameIndex.set(getTrackingFrameIndex(frameToTrackIn));
      this.headTrackingMode.set(HeadTrackingMode.ORIENTATION);
      this.orientationToTrack.setFrameOrientation(orientation);
   }

   public void setPointToTrack(FramePoint point, ReferenceFrame frameToTrackIn)
   {
      this.trackingFrameIndex.set(getTrackingFrameIndex(frameToTrackIn));
      this.headTrackingMode.set(HeadTrackingMode.POINT);
      this.pointToTrack.setFramePoint(point);
   }

   private int getTrackingFrameIndex(ReferenceFrame frameToTrackIn)
   {
      int trackingFrameIndex = ArrayUtils.indexOf(framesToTrackIn, frameToTrackIn);
      if (trackingFrameIndex == -1)
         throw new RuntimeException("Frame to track in not found");

      return trackingFrameIndex;
   }

   private enum HeadTrackingMode {ORIENTATION, POINT;}
}
