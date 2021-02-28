package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class DiscreteAngularVelocityOrientationCommand implements MPCCommand<DiscreteAngularVelocityOrientationCommand>
{
   private int commandId;

   private final FrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion();
   private final Vector3DBasics desiredBodyAngularVelocity = new Vector3D();

   private final Vector3D desiredNetAngularMomentumRate = new Vector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();

   private final Matrix3DBasics momentOfInertiaInBodyFrame = new Matrix3D();

   private final FramePoint3D desiredCoMPosition = new FramePoint3D();
   private final FrameVector3D desiredCoMAcceleration = new FrameVector3D();

   private final List<MPCContactPlane> contactPlaneHelpers = new ArrayList<>();

   private final Vector3D currentAxisAngleError = new Vector3D();
   private final Vector3D currentBodyAngularVelocityError = new Vector3D();

   private int segmentNumber;
   private int endDiscreteTickId;
   private double durationOfHold;
   private double timeOfConstraint;
   private double omega;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_VELOCITY_DYNAMICS;
   }

   public void clear()
   {
      segmentNumber = -1;
      endDiscreteTickId = -1;
      timeOfConstraint = Double.NaN;
      durationOfHold = Double.NaN;

      desiredBodyOrientation.setToNaN();
      desiredBodyAngularVelocity.setToNaN();

      desiredNetAngularMomentumRate.setToNaN();
      desiredInternalAngularMomentumRate.setToNaN();

      momentOfInertiaInBodyFrame.setToNaN();

      desiredCoMPosition.setToNaN();
      desiredCoMAcceleration.setToNaN();

      contactPlaneHelpers.clear();

      currentAxisAngleError.setToNaN();
      currentBodyAngularVelocityError.setToNaN();
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

   public void setDesiredNetAngularMomentumRate(Vector3DReadOnly desiredNetAngularMomentumRate)
   {
      this.desiredNetAngularMomentumRate.set(desiredNetAngularMomentumRate);
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

   public void setDesiredCoMAcceleration(FrameVector3DReadOnly desiredCoMAcceleration)
   {
      this.desiredCoMAcceleration.setIncludingFrame(desiredCoMAcceleration);
   }

   public void addContactPlaneHelper(MPCContactPlane contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
   }

   public void setCurrentAxisAngleError(Vector3DReadOnly currentAxisAngleError)
   {
      this.currentAxisAngleError.set(currentAxisAngleError);
   }

   public void setCurrentBodyAngularVelocityErrorInBodyFrame(Vector3DReadOnly currentBodyAngularVelocityError)
   {
      this.currentBodyAngularVelocityError.set(currentBodyAngularVelocityError);
   }

   @Override
   public void set(DiscreteAngularVelocityOrientationCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setTimeOfConstraint(other.getTimeOfConstraint());
      setDurationOfHold(other.getDurationOfHold());

      setDesiredBodyOrientation(other.getDesiredBodyOrientation());
      setDesiredBodyAngularVelocityInBodyFrame(other.getDesiredBodyAngularVelocity());

      setDesiredNetAngularMomentumRate(other.getDesiredNetAngularMomentumRate());
      setDesiredInternalAngularMomentumRate(other.getDesiredInternalAngularMomentumRate());

      setMomentOfInertiaInBodyFrame(other.getMomentOfInertiaInBodyFrame());

      setDesiredCoMPosition(other.getDesiredCoMPosition());
      setDesiredCoMAcceleration(other.getDesiredCoMAcceleration());

      setCurrentAxisAngleError(other.getCurrentAxisAngleError());
      setCurrentBodyAngularVelocityErrorInBodyFrame(other.getCurrentBodyAngularVelocityError());

      for (int i = 0; i < other.getNumberOfContacts(); i++)
         addContactPlaneHelper(other.getContactPlaneHelper(i));
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   public Vector3DReadOnly getDesiredNetAngularMomentumRate()
   {
      return desiredNetAngularMomentumRate;
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

   public Vector3DReadOnly getCurrentBodyAngularVelocityError()
   {
      return currentBodyAngularVelocityError;
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
      else if (object instanceof DiscreteAngularVelocityOrientationCommand)
      {
         DiscreteAngularVelocityOrientationCommand other = (DiscreteAngularVelocityOrientationCommand) object;
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
         if (!desiredNetAngularMomentumRate.equals(other.desiredNetAngularMomentumRate))
            return false;
         if (!desiredInternalAngularMomentumRate.equals(other.desiredInternalAngularMomentumRate))
            return false;
         if (!momentOfInertiaInBodyFrame.equals(other.momentOfInertiaInBodyFrame))
            return false;
         if (!desiredCoMPosition.equals(other.desiredCoMPosition))
            return false;
         if (!desiredCoMAcceleration.equals(other.desiredCoMAcceleration))
            return false;
         if (endDiscreteTickId == 0)
         {
            if (!currentAxisAngleError.equals(other.currentAxisAngleError))
               return false;
            if (!currentBodyAngularVelocityError.equals(other.currentBodyAngularVelocityError))
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
