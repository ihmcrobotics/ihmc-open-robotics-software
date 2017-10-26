package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface PlaneContactState
{
   public abstract RigidBody getRigidBody();

   public abstract ReferenceFrame getFrameAfterParentJoint();

   public abstract ReferenceFrame getPlaneFrame();

   public abstract boolean inContact();

   public abstract FrameVector3D getContactNormalFrameVectorCopy();

   public abstract void getContactNormalFrameVector(FrameVector3D frameVectorToPack);

   public abstract List<FramePoint3D> getContactFramePointsInContactCopy();

   public abstract void getContactFramePointsInContact(List<FramePoint3D> contactPointListToPack);

   public abstract List<FramePoint2D> getContactFramePoints2dInContactCopy();

   public abstract double getCoefficientOfFriction();

   public abstract int getNumberOfContactPointsInContact();

   public abstract int getTotalNumberOfContactPoints();

   public abstract List<? extends ContactPointInterface> getContactPoints();

   public abstract void updateFromPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommand);

   public abstract void getPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommandToPack);

   public abstract void notifyContactStateHasChanged();

   public abstract boolean pollContactHasChangedNotification();
}
