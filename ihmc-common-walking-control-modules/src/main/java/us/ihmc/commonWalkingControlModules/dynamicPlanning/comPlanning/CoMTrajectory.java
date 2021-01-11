package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalBasics;

public class CoMTrajectory implements Trajectory3DReadOnly, TimeIntervalBasics
{
   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();
   private final FramePoint3D fifthCoefficient = new FramePoint3D();
   private final FramePoint3D sixthCoefficient = new FramePoint3D();

   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();

   private final FramePoint3D dcmPosition = new FramePoint3D();
   private final FrameVector3D dcmVelocity = new FrameVector3D();
   private final FramePoint3D vrpPosition = new FramePoint3D();
   private final FrameVector3D vrpVelocity = new FrameVector3D();

   private double omega = 3.0;
   private double tInitial;
   private double tFinal;

   @Override
   public void reset()
   {
      firstCoefficient.setToNaN();
      secondCoefficient.setToNaN();
      thirdCoefficient.setToNaN();
      fourthCoefficient.setToNaN();
      fifthCoefficient.setToNaN();
      sixthCoefficient.setToNaN();

      tInitial = Double.NaN;
      tFinal = Double.NaN;
   }

   @Override
   public void setEndTime(double tFinal)
   {
      this.tFinal = tFinal;
   }

   @Override
   public void setStartTime(double t0)
   {
      this.tInitial = t0;
   }

   @Override
   public double getEndTime()
   {
      return this.tFinal;
   }

   @Override
   public double getStartTime()
   {
      return this.tInitial;
   }

   public void setCoefficients(DMatrixRMaj coefficients)
   {
      setCoefficients(coefficients, 0);
   }

   public void setCoefficients(DMatrixRMaj coefficients, int startRow)
   {
      setFirstCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow, 0), coefficients.get(startRow, 1), coefficients.get(startRow, 2));
      setSecondCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 1, 0), coefficients.get(startRow + 1, 1), coefficients.get(startRow + 1, 2));
      setThirdCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 2, 0), coefficients.get(startRow + 2, 1), coefficients.get(startRow + 2, 2));
      setFourthCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 3, 0), coefficients.get(startRow + 3, 1), coefficients.get(startRow + 3, 2));
      setFifthCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 4, 0), coefficients.get(startRow + 4, 1), coefficients.get(startRow + 4, 2));
      setSixthCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 5, 0), coefficients.get(startRow + 5, 1), coefficients.get(startRow + 5, 2));
   }

   public void setCoefficients(FramePoint3DReadOnly firstCoefficient,
                               FramePoint3DReadOnly secondCoefficient,
                               FramePoint3DReadOnly thirdCoefficient,
                               FramePoint3DReadOnly fourthCoefficient,
                               FramePoint3DReadOnly fifthCoefficient,
                               FramePoint3DReadOnly sixthCoefficient)
   {
      setFirstCoefficient(firstCoefficient);
      setSecondCoefficient(secondCoefficient);
      setThirdCoefficient(thirdCoefficient);
      setFourthCoefficient(fourthCoefficient);
      setFifthCoefficient(fifthCoefficient);
      setSixthCoefficient(sixthCoefficient);
   }

   public void setFirstCoefficient(FramePoint3DReadOnly firstCoefficient)
   {
      setFirstCoefficient(firstCoefficient.getReferenceFrame(), firstCoefficient.getX(), firstCoefficient.getY(), firstCoefficient.getZ());
   }

   public void setFirstCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      firstCoefficient.set(frame, x, y, z);
   }

   public void setSecondCoefficient(FramePoint3DReadOnly secondCoefficient)
   {
      this.secondCoefficient.set(secondCoefficient);
   }

   public void setSecondCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      secondCoefficient.set(frame, x, y, z);
   }

   public void setThirdCoefficient(FramePoint3DReadOnly thirdCoefficient)
   {
      this.thirdCoefficient.set(thirdCoefficient);
   }

   public void setThirdCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      thirdCoefficient.set(frame, x, y, z);
   }

   public void setFourthCoefficient(FramePoint3DReadOnly fourthCoefficient)
   {
      this.fourthCoefficient.set(fourthCoefficient);
   }

   public void setFourthCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      fourthCoefficient.set(frame, x, y, z);
   }

   public void setFifthCoefficient(FramePoint3DReadOnly fifthCoefficient)
   {
      this.fifthCoefficient.set(fifthCoefficient);
   }

   public void setFifthCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      fifthCoefficient.set(frame, x, y, z);
   }

   public void setSixthCoefficient(FramePoint3DReadOnly sixthCoefficient)
   {
      this.sixthCoefficient.set(sixthCoefficient);
   }

   public void setSixthCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      sixthCoefficient.set(frame, x, y, z);
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
                       FixedFrameVector3DBasics vrpVelocityTPack)
   {
      computeCoMPosition(time, comPositionToPack);
      computeCoMVelocity(time, comVelocityToPack);
      computeCoMAcceleration(time, comAccelerationToPack);

      CoMTrajectoryPlannerTools.constructDesiredVRPVelocity(vrpVelocityTPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                                            sixthCoefficient, time, omega);


      CapturePointTools.computeCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPosition);
      CapturePointTools.computeCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      CapturePointTools.computeCentroidalMomentumPivot(dcmPositionToPack, dcmVelocityToPack, omega, vrpPositionToPack);
   }

   @Override
   public Point3DReadOnly getPosition()
   {
      return comPosition;
   }

   @Override
   public Vector3DReadOnly getVelocity()
   {
      return comVelocity;
   }

   @Override
   public Vector3DReadOnly getAcceleration()
   {
      return comAcceleration;
   }

   public Point3DReadOnly getDCMPosition()
   {
      return dcmPosition;
   }
}
