package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface PlaneContactState
{
   public abstract RigidBodyBasics getRigidBody();

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

   public abstract boolean peekContactHasChangedNotification();

   /**
    * Will modify the provided contact state such that all contact points with the largest
    * x position value will be in contact and all other contact points will be deactivated.
    *
    * @param contactState to enable the toe contacts on.
    */
   public static void enableToeContacts(PlaneContactState contactState)
   {
      double maxX = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         maxX = Math.max(maxX, x);
      }
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         contactState.getContactPoints().get(i).setInContact(Precision.equals(x, maxX));
      }
      contactState.notifyContactStateHasChanged();
   }

   /**
    * Will modify the provided contact state such that all contact points with the smallest
    * x position value will be in contact and all other contact points will be deactivated.
    *
    * @param contactState to enable the heel contacts on.
    */
   public static void enableHeelContacts(PlaneContactState contactState)
   {
      double minX = Double.POSITIVE_INFINITY;
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         minX = Math.min(minX, x);
      }
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         double x = contactState.getContactPoints().get(i).getPosition().getX();
         contactState.getContactPoints().get(i).setInContact(Precision.equals(x, minX));
      }
      contactState.notifyContactStateHasChanged();
   }

   /**
    * Enables all contact points in the provided contact state.
    *
    * @param contactState to enable all contacts on.
    */
   public static void enableAllContacts(PlaneContactState contactState)
   {
      for (int i = 0; i < contactState.getContactPoints().size(); i++)
      {
         contactState.getContactPoints().get(i).setInContact(true);
      }
      contactState.notifyContactStateHasChanged();
   }
}
