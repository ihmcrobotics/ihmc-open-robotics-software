package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

public class WrenchTrajectorySegment implements TimeIntervalProvider, Settable<WrenchTrajectorySegment>
{
   private final WrenchBasics firstCoefficient = new Wrench();
   private final WrenchBasics secondCoefficient = new Wrench();
   private final WrenchBasics thirdCoefficient = new Wrench();
   private final WrenchBasics fourthCoefficient = new Wrench();

   private final WrenchBasics pointFirstCoefficient = new Wrench();
   private final WrenchBasics pointSecondCoefficient = new Wrench();
   private final WrenchBasics pointThirdCoefficient = new Wrench();
   private final WrenchBasics pointFourthCoefficient = new Wrench();

   private final FrameVector3D linear = new FrameVector3D();
   private final FrameVector3D angular = new FrameVector3D();

   private double currentTime;
   private double omega = 3.0;
   private final TimeIntervalBasics timeInterval = new TimeInterval();

   private final WrenchBasics desiredWrench = new Wrench();

   public void reset()
   {
      currentTime = Double.NaN;
      firstCoefficient.setToNaN();
      secondCoefficient.setToNaN();
      thirdCoefficient.setToNaN();
      fourthCoefficient.setToNaN();

      timeInterval.reset();
   }


   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   @Override
   public void set(WrenchTrajectorySegment other)
   {
      getTimeInterval().set(other.getTimeInterval());
      currentTime = other.currentTime;
      omega = other.omega;
      setCoefficients(other);

      desiredWrench.set(other.desiredWrench);
   }

   public void setCoefficients(MPCContactPlane mpcContactPlane)
   {
      firstCoefficient.setToZero();
      secondCoefficient.setToZero();
      thirdCoefficient.setToZero();
      fourthCoefficient.setToZero();

      for (int pointIdx = 0; pointIdx < mpcContactPlane.getNumberOfContactPoints(); pointIdx++)
      {
         MPCContactPoint contactPoint = mpcContactPlane.getContactPointHelper(pointIdx);
         DMatrixRMaj trajectoryCoeff = contactPoint.getContactForceCoefficientMatrix();

         for (int element = 0; element < 3; element++)
            linear.setElement(element, trajectoryCoeff.get(0, element));
         pointFirstCoefficient.set(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), linear, angular, contactPoint.getBasisVectorOrigin());
         for (int element = 0; element < 3; element++)
            linear.setElement(element, trajectoryCoeff.get(1, element));
         pointSecondCoefficient.set(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), linear, angular, contactPoint.getBasisVectorOrigin());
         for (int element = 0; element < 3; element++)
            linear.setElement(element, trajectoryCoeff.get(2, element));
         pointThirdCoefficient.set(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), linear, angular, contactPoint.getBasisVectorOrigin());
         for (int element = 0; element < 3; element++)
            linear.setElement(element, trajectoryCoeff.get(3, element));
         pointFourthCoefficient.set(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), linear, angular, contactPoint.getBasisVectorOrigin());

         firstCoefficient.add(pointFirstCoefficient);
         secondCoefficient.add(pointSecondCoefficient);
         thirdCoefficient.add(pointThirdCoefficient);
         fourthCoefficient.add(pointFourthCoefficient);
      }
   }

   public void setCoefficients(WrenchTrajectorySegment other)
   {
      setCoefficients(other.firstCoefficient,
                      other.secondCoefficient,
                      other.thirdCoefficient,
                      other.fourthCoefficient);
   }

   public void setCoefficients(WrenchReadOnly firstCoefficient,
                               WrenchReadOnly secondCoefficient,
                               WrenchReadOnly thirdCoefficient,
                               WrenchReadOnly fourthCoefficient)
   {
      this.firstCoefficient.setIncludingFrame(firstCoefficient);
      this.secondCoefficient.setIncludingFrame(secondCoefficient);
      this.thirdCoefficient.setIncludingFrame(thirdCoefficient);
      this.fourthCoefficient.setIncludingFrame(fourthCoefficient);
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   private final WrenchBasics tempWrench = new Wrench();

   public void compute(double time)
   {
      compute(time, desiredWrench);
   }

   public void compute(double time, WrenchBasics wrenchToPack)
   {
      wrenchToPack.setToZero();
      double omega2 = omega * omega;
      double exponential = Math.exp(omega * time);
      tempWrench.set(firstCoefficient);
      tempWrench.scale(omega2 * exponential);
      wrenchToPack.add(tempWrench);

      tempWrench.set(secondCoefficient);
      tempWrench.scale(omega2 / exponential);
      wrenchToPack.add(tempWrench);

      tempWrench.set(thirdCoefficient);
      tempWrench.scale(6.0 * time);
      wrenchToPack.add(tempWrench);

      tempWrench.set(fourthCoefficient);
      tempWrench.scale(2.0);
      wrenchToPack.add(tempWrench);
   }
}
