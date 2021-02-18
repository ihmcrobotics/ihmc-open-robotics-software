package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class DiscreteOrientationCommand implements MPCCommand<DiscreteOrientationCommand>
{
   private int commandId;

   private final FrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion();
   private final Vector3DBasics desiredBodyAngularVelocity = new Vector3D();

   private final Vector3D desiredNetAngularMomentum = new Vector3D();
   private final Vector3D desiredInternalAngularMomentum = new Vector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();

   private final Matrix3DBasics momentOfInertiaInBodyFrame = new Matrix3D();

   private final FramePoint3D desiredCoMPosition = new FramePoint3D();
   private final FrameVector3D desiredCoMVelocity = new FrameVector3D();

   private final List<MPCContactPlane> contactPlaneHelpers = new ArrayList<>();

   private final Vector3D currentAxisAngleError = new Vector3D();
   private final FrameVector3D currentBodyAngularMomentumAboutFixedPoint = new FrameVector3D();

   private int segmentNumber;
   private int endDiscreteTickId;
   private double durationOfHold;
   private double timeOfConstraint;
   private double omega;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_DYNAMICS;
   }

   public void clear()
   {
      segmentNumber = -1;
      endDiscreteTickId = -1;
      timeOfConstraint = Double.NaN;
      durationOfHold = Double.NaN;

      desiredBodyOrientation.setToNaN();
      desiredBodyAngularVelocity.setToNaN();

      desiredNetAngularMomentum.setToNaN();
      desiredInternalAngularMomentum.setToNaN();
      desiredInternalAngularMomentumRate.setToNaN();

      momentOfInertiaInBodyFrame.setToNaN();

      desiredCoMPosition.setToNaN();
      desiredCoMVelocity.setToNaN();

      contactPlaneHelpers.clear();

      currentAxisAngleError.setToNaN();
      currentBodyAngularMomentumAboutFixedPoint.setToNaN();
   }

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public void setEndingDiscreteTickId(int tickId)
   {
      this.endDiscreteTickId = tickId;
   }

   public void setDurationOfHold(double durationOfHold)
   {
      this.durationOfHold = durationOfHold;
   }

   public void setTimeOfConstraint(double timeOfConstraint)
   {
      this.timeOfConstraint = timeOfConstraint;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void setDesiredBodyOrientation(FrameOrientation3DReadOnly desiredBodyOrientation)
   {
      this.desiredBodyOrientation.setIncludingFrame(desiredBodyOrientation);
   }

   public void setDesiredBodyAngularVelocityInBodyFrame(Vector3DReadOnly desiredBodyAngularVelocity)
   {
      this.desiredBodyAngularVelocity.set(desiredBodyAngularVelocity);
   }

   public void setDesiredNetAngularMomentum(Vector3DReadOnly desiredNetAngularMomentum)
   {
      this.desiredNetAngularMomentum.set(desiredNetAngularMomentum);
   }

   public void setDesiredInternalAngularMomentum(Vector3DReadOnly desiredInternalAngularMomentum)
   {
      this.desiredInternalAngularMomentum.set(desiredInternalAngularMomentum);
   }

   public void setDesiredInternalAngularMomentumRate(Vector3DReadOnly desiredInternalAngularMomentumRate)
   {
      this.desiredInternalAngularMomentumRate.set(desiredInternalAngularMomentumRate);
   }

   public void setMomentOfInertiaInBodyFrame(Matrix3DReadOnly momentOfInertiaInBodyFrame)
   {
      this.momentOfInertiaInBodyFrame.set(momentOfInertiaInBodyFrame);
   }

   public void setDesiredCoMPosition(FramePoint3DReadOnly desiredCoMPosition)
   {
      this.desiredCoMPosition.setIncludingFrame(desiredCoMPosition);
   }

   public void setDesiredCoMVelocity(FrameVector3DReadOnly desiredCoMVelocity)
   {
      this.desiredCoMVelocity.setIncludingFrame(desiredCoMVelocity);
   }

   public void addContactPlaneHelper(MPCContactPlane contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
   }

   public void setCurrentAxisAngleError(Vector3DReadOnly currentAxisAngleError)
   {
      this.currentAxisAngleError.set(currentAxisAngleError);
   }

   public void setCurrentBodyAngularMomentumAboutFixedPoint(FrameVector3DReadOnly currentBodyAngularMomentumAboutFixedPoint)
   {
      this.currentBodyAngularMomentumAboutFixedPoint.setIncludingFrame(currentBodyAngularMomentumAboutFixedPoint);
   }

   @Override
   public void set(DiscreteOrientationCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setTimeOfConstraint(other.getTimeOfConstraint());
      setDurationOfHold(other.getDurationOfHold());

      setDesiredBodyOrientation(other.getDesiredBodyOrientation());
      setDesiredBodyAngularVelocityInBodyFrame(other.getDesiredBodyAngularVelocity());

      setDesiredNetAngularMomentum(other.getDesiredNetAngularMomentum());
      setDesiredInternalAngularMomentum(other.getDesiredInternalAngularMomentum());
      setDesiredInternalAngularMomentumRate(other.getDesiredInternalAngularMomentumRate());

      setMomentOfInertiaInBodyFrame(other.getMomentOfInertiaInBodyFrame());

      setDesiredCoMPosition(other.getDesiredCoMPosition());
      setDesiredCoMVelocity(other.getDesiredCoMVelocity());

      setCurrentAxisAngleError(other.getCurrentAxisAngleError());
      setCurrentBodyAngularMomentumAboutFixedPoint(other.getCurrentBodyAngularMomentumAboutFixedPoint());

      for (int i = 0; i < other.getNumberOfContacts(); i++)
         addContactPlaneHelper(other.getContactPlaneHelper(i));
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   public Vector3DReadOnly getDesiredNetAngularMomentum()
   {
      return desiredNetAngularMomentum;
   }

   public Vector3DReadOnly getDesiredInternalAngularMomentum()
   {
      return desiredInternalAngularMomentum;
   }

   public Vector3DReadOnly getDesiredInternalAngularMomentumRate()
   {
      return desiredInternalAngularMomentumRate;
   }

   public FrameOrientation3DReadOnly getDesiredBodyOrientation()
   {
      return desiredBodyOrientation;
   }

   public Vector3DReadOnly getDesiredBodyAngularVelocity()
   {
      return desiredBodyAngularVelocity;
   }

   public Matrix3DReadOnly getMomentOfInertiaInBodyFrame()
   {
      return momentOfInertiaInBodyFrame;
   }

   public Vector3DReadOnly getCurrentAxisAngleError()
   {
      return currentAxisAngleError;
   }

   public FrameVector3DReadOnly getCurrentBodyAngularMomentumAboutFixedPoint()
   {
      return currentBodyAngularMomentumAboutFixedPoint;
   }

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public double getDurationOfHold()
   {
      return durationOfHold;
   }

   public double getTimeOfConstraint()
   {
      return timeOfConstraint;
   }

   public double getOmega()
   {
      return omega;
   }

   public int getEndDiscreteTickId()
   {
      return endDiscreteTickId;
   }

   public int getNumberOfContacts()
   {
      return contactPlaneHelpers.size();
   }

   public MPCContactPlane getContactPlaneHelper(int i)
   {
      return contactPlaneHelpers.get(i);
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof DiscreteOrientationCommand)
      {
         DiscreteOrientationCommand other = (DiscreteOrientationCommand) object;
         if (commandId != other.commandId)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (endDiscreteTickId != other.endDiscreteTickId)
            return false;
         if (durationOfHold != other.durationOfHold)
            return false;
         if (timeOfConstraint != other.timeOfConstraint)
            return false;
         if (omega != other.omega)
            return false;
         if (!desiredBodyOrientation.equals(other.desiredBodyOrientation))
            return false;
         if (!desiredBodyAngularVelocity.equals(other.desiredBodyAngularVelocity))
            return false;
         if (!desiredNetAngularMomentum.equals(other.desiredNetAngularMomentum))
            return false;
         if (!desiredInternalAngularMomentum.equals(other.desiredInternalAngularMomentum))
            return false;
         if (!desiredInternalAngularMomentumRate.equals(other.desiredInternalAngularMomentumRate))
            return false;
         if (!momentOfInertiaInBodyFrame.equals(other.momentOfInertiaInBodyFrame))
            return false;
         if (!desiredCoMPosition.equals(other.desiredCoMPosition))
            return false;
         if (!desiredCoMVelocity.equals(other.desiredCoMVelocity))
            return false;
         if (endDiscreteTickId == 0)
         {
            if (!currentAxisAngleError.equals(other.currentAxisAngleError))
               return false;
            if (!currentBodyAngularMomentumAboutFixedPoint.equals(other.currentBodyAngularMomentumAboutFixedPoint))
               return false;
         }
         if (contactPlaneHelpers.size() != other.contactPlaneHelpers.size())
            return false;
         for (int i = 0; i < contactPlaneHelpers.size(); i++)
         {
            if (!contactPlaneHelpers.get(i).equals(other.contactPlaneHelpers.get(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }

   /*
   @Override
   public String toString()
   {
      String string = getClass().getSimpleName() + ": value: " + getValueType() + ", derivative order: " + getDerivativeOrder() + ", segment number: "
                      + segmentNumber + ", constraint type: " + constraintType + ", time of objective: " + timeOfObjective + ", omega: " + omega + ", weight: "
                      + weight + ", objective: " + objective + ".";
      for (int i = 0; i < getNumberOfContacts(); i++)
      {
         string += "\ncontact " + i + ": " + contactPlaneHelpers.get(i);
      }
      return string;
   }

    */
}
