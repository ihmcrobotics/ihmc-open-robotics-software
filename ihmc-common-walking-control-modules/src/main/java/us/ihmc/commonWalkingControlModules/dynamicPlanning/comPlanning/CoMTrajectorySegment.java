package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLarge;

public class CoMTrajectorySegment implements FixedFramePositionTrajectoryGenerator, TimeIntervalProvider, Settable<CoMTrajectorySegment>
{
   private final Point3D firstCoefficient = new Point3D();
   private final Point3D secondCoefficient = new Point3D();
   private final Point3D thirdCoefficient = new Point3D();
   private final Point3D fourthCoefficient = new Point3D();
   private final Point3D fifthCoefficient = new Point3D();
   private final Point3D sixthCoefficient = new Point3D();

   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();

   private final FramePoint3D dcmPosition = new FramePoint3D();
   private final FrameVector3D dcmVelocity = new FrameVector3D();
   private final FramePoint3D vrpPosition = new FramePoint3D();
   private final FrameVector3D vrpVelocity = new FrameVector3D();

   private double currentTime;
   private double omega = 3.0;
   private final TimeIntervalBasics timeInterval = new TimeInterval();

   public void reset()
   {
      currentTime = Double.NaN;
      firstCoefficient.setToNaN();
      secondCoefficient.setToNaN();
      thirdCoefficient.setToNaN();
      fourthCoefficient.setToNaN();
      fifthCoefficient.setToNaN();
      sixthCoefficient.setToNaN();

      timeInterval.reset();
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   @Override
   public void set(CoMTrajectorySegment other)
   {
      getTimeInterval().set(other.getTimeInterval());
      currentTime = other.currentTime;
      omega = other.omega;
      setCoefficients(other);

      comPosition.set(other.comPosition);
      comVelocity.set(other.comVelocity);
      comAcceleration.set(other.comAcceleration);

      dcmPosition.set(other.dcmPosition);
      dcmVelocity.set(other.dcmVelocity);
      vrpPosition.set(other.vrpPosition);
      vrpVelocity.set(other.vrpVelocity);
   }

   public void setCoefficients(DMatrixRMaj coefficients)
   {
      setCoefficients(coefficients, 0);
   }

   public void setCoefficients(DMatrixRMaj coefficients, int startRow)
   {
      setFirstCoefficient(coefficients.get(startRow, 0), coefficients.get(startRow, 1), coefficients.get(startRow, 2));
      setSecondCoefficient(coefficients.get(startRow + 1, 0), coefficients.get(startRow + 1, 1), coefficients.get(startRow + 1, 2));
      setThirdCoefficient(coefficients.get(startRow + 2, 0), coefficients.get(startRow + 2, 1), coefficients.get(startRow + 2, 2));
      setFourthCoefficient(coefficients.get(startRow + 3, 0), coefficients.get(startRow + 3, 1), coefficients.get(startRow + 3, 2));
      setFifthCoefficient(coefficients.get(startRow + 4, 0), coefficients.get(startRow + 4, 1), coefficients.get(startRow + 4, 2));
      setSixthCoefficient(coefficients.get(startRow + 5, 0), coefficients.get(startRow + 5, 1), coefficients.get(startRow + 5, 2));
   }

   public void setCoefficients(CoMTrajectorySegment other)
   {
      setCoefficients(other.firstCoefficient,
                      other.secondCoefficient,
                      other.thirdCoefficient,
                      other.fourthCoefficient,
                      other.fifthCoefficient,
                      other.sixthCoefficient);
   }

   public void setCoefficients(Point3DReadOnly firstCoefficient,
                               Point3DReadOnly secondCoefficient,
                               Point3DReadOnly thirdCoefficient,
                               Point3DReadOnly fourthCoefficient,
                               Point3DReadOnly fifthCoefficient,
                               Point3DReadOnly sixthCoefficient)
   {
      setFirstCoefficient(firstCoefficient);
      setSecondCoefficient(secondCoefficient);
      setThirdCoefficient(thirdCoefficient);
      setFourthCoefficient(fourthCoefficient);
      setFifthCoefficient(fifthCoefficient);
      setSixthCoefficient(sixthCoefficient);
   }

   public void setFirstCoefficient(Point3DReadOnly firstCoefficient)
   {
      setFirstCoefficient(firstCoefficient.getX(), firstCoefficient.getY(), firstCoefficient.getZ());
   }

   public void setFirstCoefficient(double x, double y, double z)
   {
      firstCoefficient.set(x, y, z);
   }

   public void setSecondCoefficient(Point3DReadOnly secondCoefficient)
   {
      this.secondCoefficient.set(secondCoefficient);
   }

   public void setSecondCoefficient(double x, double y, double z)
   {
      secondCoefficient.set(x, y, z);
   }

   public void setThirdCoefficient(Point3DReadOnly thirdCoefficient)
   {
      this.thirdCoefficient.set(thirdCoefficient);
   }

   public void setThirdCoefficient(double x, double y, double z)
   {
      thirdCoefficient.set(x, y, z);
   }

   public void setFourthCoefficient(Point3DReadOnly fourthCoefficient)
   {
      this.fourthCoefficient.set(fourthCoefficient);
   }

   public void setFourthCoefficient(double x, double y, double z)
   {
      fourthCoefficient.set(x, y, z);
   }

   public void setFifthCoefficient(Point3DReadOnly fifthCoefficient)
   {
      this.fifthCoefficient.set(fifthCoefficient);
   }

   public void setFifthCoefficient(double x, double y, double z)
   {
      fifthCoefficient.set(x, y, z);
   }

   public void setSixthCoefficient(Point3DReadOnly sixthCoefficient)
   {
      this.sixthCoefficient.set(sixthCoefficient);
   }

   public void setSixthCoefficient(double x, double y, double z)
   {
      sixthCoefficient.set(x, y, z);
   }

   private final FramePoint3D modifiedFourthCoefficient = new FramePoint3D();
   private final FramePoint3D modifiedFifthCoefficient = new FramePoint3D();
   private final FramePoint3D modifiedSixthCoefficient = new FramePoint3D();

   public void shiftStartOfSegment(double durationToShift)
   {
      double originalDuration = getTimeInterval().getDuration();
      if (durationToShift > originalDuration)
         throw new IllegalArgumentException("New start time " + durationToShift + " must be less than end time " + originalDuration);

      double d2 = durationToShift * durationToShift;
      double d3 = d2 * durationToShift;
      double startTime = getTimeInterval().getStartTime();
      getTimeInterval().setInterval(startTime + durationToShift, getTimeInterval().getEndTime());
      double exponential = Math.exp(omega * durationToShift);
      firstCoefficient.scale(exponential);
      secondCoefficient.scale(1.0 / exponential);
      modifiedFourthCoefficient.scaleAdd(3.0 * durationToShift, thirdCoefficient, fourthCoefficient);
      modifiedFifthCoefficient.scaleAdd(3.0 * d2, thirdCoefficient, fifthCoefficient);
      modifiedFifthCoefficient.scaleAdd(2.0 * durationToShift, fourthCoefficient, modifiedFifthCoefficient);
      modifiedSixthCoefficient.scaleAdd(d3, thirdCoefficient, sixthCoefficient);
      modifiedSixthCoefficient.scaleAdd(d2, fourthCoefficient, modifiedSixthCoefficient);
      modifiedSixthCoefficient.scaleAdd(durationToShift, fifthCoefficient, modifiedSixthCoefficient);

      fourthCoefficient.set(modifiedFourthCoefficient);
      fifthCoefficient.set(modifiedFifthCoefficient);
      sixthCoefficient.set(modifiedSixthCoefficient);
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void computeCoMPosition(double time, FixedFramePoint3DBasics comPositionToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMPosition(comPositionToPack,
                                                            firstCoefficient,
                                                            secondCoefficient,
                                                            thirdCoefficient,
                                                            fourthCoefficient,
                                                            fifthCoefficient,
                                                            sixthCoefficient,
                                                            time,
                                                            omega);
   }

   public void computeCoMVelocity(double time, FixedFrameVector3DBasics comVelocityToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(comVelocityToPack,
                                                            firstCoefficient,
                                                            secondCoefficient,
                                                            thirdCoefficient,
                                                            fourthCoefficient,
                                                            fifthCoefficient,
                                                            sixthCoefficient,
                                                            time,
                                                            omega);
   }

   public void computeCoMAcceleration(double time, FixedFrameVector3DBasics comAccelerationToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(comAccelerationToPack,
                                                                firstCoefficient,
                                                                secondCoefficient,
                                                                thirdCoefficient,
                                                                fourthCoefficient,
                                                                fifthCoefficient,
                                                                sixthCoefficient,
                                                                time,
                                                                omega);
   }

   public void computeVRPVelocity(double time, FixedFrameVector3DBasics vrpVelocityToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredVRPVelocity(vrpVelocityToPack,
                                                                firstCoefficient,
                                                                secondCoefficient,
                                                                thirdCoefficient,
                                                                fourthCoefficient,
                                                                fifthCoefficient,
                                                                sixthCoefficient,
                                                                time,
                                                                omega);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void compute(double time)
   {
      compute(time, comPosition, comVelocity, comAcceleration, dcmPosition, dcmVelocity, vrpPosition, vrpVelocity);
   }

   public void compute(double time,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFrameVector3DBasics vrpVelocityToPack)
   {
      currentTime = time;

      double exponential = Math.min(sufficientlyLarge, Math.exp(omega * time));
      double negativeExponential = 1.0 / exponential;
      double t = Math.min(sufficientlyLarge, time);
      double t2 = Math.min(sufficientlyLarge, time * time);
      double t3 = Math.min(sufficientlyLarge, time * t2);

      double omega2 = omega * omega;

      comPositionToPack.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      comPositionToPack.setAndScale(exponential, firstCoefficient);
      comPositionToPack.scaleAdd(negativeExponential, secondCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(t3, thirdCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(t2, fourthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(t, fifthCoefficient, comPositionToPack);
      comPositionToPack.add(sixthCoefficient);

      comVelocityToPack.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      comVelocityToPack.setAndScale(omega * exponential, firstCoefficient);
      comVelocityToPack.scaleAdd(-omega * negativeExponential, secondCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(3.0 * t2, thirdCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(2.0 * t, fourthCoefficient, comVelocityToPack);
      comVelocityToPack.add(fifthCoefficient);

      comAccelerationToPack.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      comAccelerationToPack.setAndScale(omega2 * exponential, firstCoefficient);
      comAccelerationToPack.scaleAdd(omega2 * negativeExponential, secondCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(6.0 * t, thirdCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(2.0, fourthCoefficient, comAccelerationToPack);

      vrpVelocityToPack.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      vrpVelocityToPack.setAndScale(t3 - 6.0 * t / omega2, thirdCoefficient);
      vrpVelocityToPack.scaleAdd(t2 - 2.0 / omega2, fourthCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(t, fifthCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.add(sixthCoefficient);

      CapturePointTools.computeCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPositionToPack);
      CapturePointTools.computeCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      CapturePointTools.computeCentroidalMomentumPivot(dcmPositionToPack, dcmVelocityToPack, omega, vrpPositionToPack);
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return comPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return comVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return comAcceleration;
   }

   public FramePoint3DReadOnly getDCMPosition()
   {
      return dcmPosition;
   }

   public FrameVector3DReadOnly getDCMVelocity()
   {
      return dcmVelocity;
   }

   public FramePoint3DReadOnly getVRPPosition()
   {
      return vrpPosition;
   }

   public FrameVector3DReadOnly getVRPVelocity()
   {
      return vrpVelocity;
   }

   public Point3DReadOnly getFirstCoefficient()
   {
      return firstCoefficient;
   }

   public Point3DReadOnly getSecondCoefficient()
   {
      return secondCoefficient;
   }

   public Point3DReadOnly getThirdCoefficient()
   {
      return thirdCoefficient;
   }

   public Point3DReadOnly getFourthCoefficient()
   {
      return fourthCoefficient;
   }

   public Point3DReadOnly getFifthCoefficient()
   {
      return fifthCoefficient;
   }

   public Point3DReadOnly getSixthCoefficient()
   {
      return sixthCoefficient;
   }

   @Override
   public boolean isDone()
   {
      return currentTime >= getTimeInterval().getEndTime();
   }

   @Override
   public void hideVisualization()
   {}

   @Override
   public void showVisualization()
   {
   }
}
