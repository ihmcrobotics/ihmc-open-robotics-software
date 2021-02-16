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
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class DiscreteOrientationCommand
{
   private final FrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion();
   private final FrameVector3DBasics desiredBodyAngularVelocity = new FrameVector3D();

   private final FrameVector3DBasics desiredNetAngularMomentum = new FrameVector3D();
   private final FrameVector3DBasics desiredInternalAngularMomentum = new FrameVector3D();
   private final FrameVector3DBasics desiredInternalAngularMomentumRate = new FrameVector3D();

   private final Matrix3DBasics momentOfInertiaInBodyFrame = new Matrix3D();

   private final FramePoint3D desiredCoMPosition = new FramePoint3D();
   private final FrameVector3D desiredCoMVelocity = new FrameVector3D();

   private final List<MPCContactPlane> contactPlaneHelpers = new ArrayList<>();

   private final Vector3D currentAxisAngleError = new Vector3D();
   private final FrameVector3D currentBodyAngularMomentumAboutFixedPoint = new FrameVector3D();

   private int segmentId;
   private int endDiscreteTickId;
   private double durationOfHold;
   private double timeOfConstraint;
   private double omega;

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   public FrameVector3DReadOnly getDesiredNetAngularMomentum()
   {
      return desiredNetAngularMomentum;
   }

   public FrameVector3DReadOnly getDesiredInternalAngularMomentum()
   {
      return desiredInternalAngularMomentum;
   }

   public FrameVector3DReadOnly getDesiredInternalAngularMomentumRate()
   {
      return desiredInternalAngularMomentumRate;
   }

   public FrameOrientation3DReadOnly getDesiredBodyOrientation()
   {
      return desiredBodyOrientation;
   }

   public FrameVector3DReadOnly getDesiredBodyAngularVelocity()
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

   public int getSegmentId()
   {
      return segmentId;
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

}
