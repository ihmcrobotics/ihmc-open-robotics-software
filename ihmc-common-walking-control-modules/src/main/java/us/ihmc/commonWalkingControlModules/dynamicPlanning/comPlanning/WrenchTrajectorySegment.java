package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

import java.util.List;

public class WrenchTrajectorySegment implements TimeIntervalProvider, Settable<WrenchTrajectorySegment>, ReferenceFrameHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WrenchBasics[] coefficients = new WrenchBasics[]{new Wrench(worldFrame, worldFrame),
                                                                  new Wrench(worldFrame, worldFrame),
                                                                  new Wrench(worldFrame, worldFrame),
                                                                  new Wrench(worldFrame, worldFrame)};
   private final WrenchBasics pointCoefficient = new Wrench(worldFrame, worldFrame);
   private final FrameVector3D linearCoefficient = new FrameVector3D();
   private static final FrameVector3DReadOnly zero = new FrameVector3D();

   private double currentTime;
   private double mass = 1.0;
   private double omega = 3.0;
   private final TimeIntervalBasics timeInterval = new TimeInterval();

   private final WrenchBasics desiredWrench = new Wrench(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());

   public void reset()
   {
      currentTime = Double.NaN;
      for (WrenchBasics coefficient : coefficients)
         coefficient.setToNaN();

      timeInterval.reset();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return getWrench().getReferenceFrame();
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
      setCoefficients(other);

      currentTime = other.currentTime;
      omega = other.omega;
      mass = other.mass;
      desiredWrench.setIncludingFrame(other.desiredWrench);
   }

   public void setCoefficients(List<MPCContactPlane> mpcContactPlanes)
   {
      for (WrenchBasics coefficient : coefficients)
         coefficient.setToZero(ReferenceFrame.getWorldFrame());

      for (int planeIdx = 0; planeIdx < mpcContactPlanes.size(); planeIdx++)
      {
         addCoefficients(mpcContactPlanes.get(planeIdx));
      }
   }

   private void addCoefficients(MPCContactPlane mpcContactPlane)
   {
      for (int pointIdx = 0; pointIdx < mpcContactPlane.getNumberOfContactPoints(); pointIdx++)
      {
         MPCContactPoint contactPoint = mpcContactPlane.getContactPointHelper(pointIdx);
         DMatrixRMaj trajectoryCoeff = contactPoint.getContactForceCoefficientMatrix();

         for (int coefficientIdx = 0; coefficientIdx < coefficients.length; coefficientIdx++)
         {
            for (int element = 0; element < 3; element++)
            {
               linearCoefficient.setElement(element, trajectoryCoeff.get(element, coefficientIdx));
            }
            linearCoefficient.scale(mass);
            pointCoefficient.set(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), zero, linearCoefficient, contactPoint.getBasisVectorOrigin());
            coefficients[coefficientIdx].add(pointCoefficient);
         }
      }
   }

   public void setCoefficients(WrenchTrajectorySegment other)
   {
      setCoefficients(other.coefficients);
   }

   public void setCoefficients(WrenchReadOnly[] coefficients)
   {
      for (int coefficientIdx = 0; coefficientIdx < coefficients.length; coefficientIdx++)
         this.coefficients[coefficientIdx].setIncludingFrame(coefficients[coefficientIdx]);
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void compute(double time)
   {
      compute(time, desiredWrench);
   }

   public void compute(double time, WrenchBasics wrenchToPack)
   {
      wrenchToPack.setToZero(coefficients[0].getReferenceFrame());

      double omega2 = omega * omega;
      double exponential = Math.exp(omega * time);
      scaleAddWrench(wrenchToPack, omega2 * exponential, coefficients[0]);
      scaleAddWrench(wrenchToPack, omega2 / exponential, coefficients[1]);
      scaleAddWrench(wrenchToPack, 6.0 * time, coefficients[2]);
      scaleAddWrench(wrenchToPack, 2.0, coefficients[3]);
   }

   public WrenchReadOnly getWrench()
   {
      return desiredWrench;
   }

   private static void scaleAddWrench(WrenchBasics wrenchToPack, double scaleFactor, WrenchReadOnly wrenchToAdd)
   {
      wrenchToPack.getAngularPart().scaleAdd(scaleFactor, wrenchToAdd.getAngularPart(), wrenchToPack.getAngularPart());
      wrenchToPack.getLinearPart().scaleAdd(scaleFactor, wrenchToAdd.getLinearPart(), wrenchToPack.getLinearPart());
   }
}
