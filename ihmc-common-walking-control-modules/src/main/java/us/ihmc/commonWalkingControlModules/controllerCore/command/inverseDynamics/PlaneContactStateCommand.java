package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.lists.FrameTupleArrayList;

public class PlaneContactStateCommand implements InverseDynamicsCommand<PlaneContactStateCommand>, VirtualModelControlCommand<PlaneContactStateCommand>
{
   private static final int initialSize = 8;
   private RigidBodyBasics rigidBody;
   private double coefficientOfFriction = Double.NaN;
   private final FrameTupleArrayList<FramePoint3D> contactPoints = FrameTupleArrayList.createFramePointArrayList(initialSize);
   private final FrameVector3D contactNormal = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);

   private boolean useHighCoPDamping = false;
   private boolean hasContactStateChanged;

   private boolean hasMaxContactPointNormalForce = false;
   private final TDoubleArrayList maxContactPointNormalForces = new TDoubleArrayList(initialSize);

   /**
    * Holds on to the rho weights, which are described in
    * {@code ControllerCoreOptimizationSettings.getRhoWeight()} If a rho weight is to Double.NaN, the
    * controller core will use the default rho weight set in the MomentumOptimizationSetting
    */
   private final TDoubleArrayList rhoWeights = new TDoubleArrayList(initialSize);

   /**
    * Defines the pose of the contact frame expressed in the contacting body fixed frame.
    */
   private final RigidBodyTransform contactFramePoseInBodyFixedFrame = new RigidBodyTransform();

   public PlaneContactStateCommand()
   {
      clearContactPoints();
   }

   @Override
   public void set(PlaneContactStateCommand other)
   {
      rigidBody = other.rigidBody;
      coefficientOfFriction = other.coefficientOfFriction;
      contactPoints.copyFromListAndTrimSize(other.contactPoints);
      contactNormal.setIncludingFrame(other.contactNormal);
      useHighCoPDamping = other.useHighCoPDamping;
      hasContactStateChanged = other.hasContactStateChanged;

      hasMaxContactPointNormalForce = other.hasMaxContactPointNormalForce;

      maxContactPointNormalForces.reset();
      rhoWeights.reset();

      for (int i = 0; i < other.contactPoints.size(); i++)
      {
         maxContactPointNormalForces.add(other.maxContactPointNormalForces.get(i));
         rhoWeights.add(other.rhoWeights.get(i));
      }

      contactFramePoseInBodyFixedFrame.set(other.contactFramePoseInBodyFixedFrame);
   }

   public void setHasContactStateChanged(boolean hasContactStateChanged)
   {
      this.hasContactStateChanged = hasContactStateChanged;
   }

   public void setContactingRigidBody(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void clearContactPoints()
   {
      contactPoints.clear();
      maxContactPointNormalForces.reset();
      rhoWeights.reset();
      contactFramePoseInBodyFixedFrame.setToNaN();
   }

   /**
    * Optional: Use this method to redefine the pose of the contact frame to use with this contacting
    * rigid-body.
    * 
    * @param contactFrame the reference frame to use for updating the contact frame used in the
    *           controller core.
    */
   public void setContactFramePose(ReferenceFrame contactFrame)
   {
      contactFrame.getTransformToDesiredFrame(contactFramePoseInBodyFixedFrame, rigidBody.getBodyFixedFrame());
   }

   public FramePoint3DBasics addPointInContact()
   {
      maxContactPointNormalForces.add(Double.POSITIVE_INFINITY);
      rhoWeights.add(Double.NaN);
      return contactPoints.add();
   }

   public void addPointInContact(FramePoint3DReadOnly newPointInContact)
   {
      addPointInContact().setIncludingFrame(newPointInContact);
   }

   public void addPointInContact(FramePoint2DReadOnly newPointInContact)
   {
      addPointInContact().setIncludingFrame(newPointInContact, 0.0);
   }

   public void setPointsInContact(List<? extends FramePoint3DReadOnly> newPointsInContact)
   {
      clearContactPoints();
      for (int i = 0; i < newPointsInContact.size(); i++)
      {
         addPointInContact().setIncludingFrame(newPointsInContact.get(i));
      }
   }

   public void setPoint2dsInContact(ReferenceFrame contactFrame, List<? extends Point2DReadOnly> newPointsInContact)
   {
      clearContactPoints();

      for (int i = 0; i < newPointsInContact.size(); i++)
      {
         addPointInContact().setIncludingFrame(contactFrame, newPointsInContact.get(i), 0.0);
      }
   }

   public void setMaxContactPointNormalForce(int contactPointIndex, double maxNormalForce)
   {
      hasMaxContactPointNormalForce |= Double.isFinite(maxNormalForce);
      maxContactPointNormalForces.set(contactPointIndex, maxNormalForce);
   }

   public void setContactNormal(FrameVector3DReadOnly contactNormal)
   {
      this.contactNormal.setIncludingFrame(contactNormal);
   }

   public void setUseHighCoPDamping(boolean useHighCoPDamping)
   {
      this.useHighCoPDamping = useHighCoPDamping;
   }

   public boolean isEmpty()
   {
      return contactPoints.isEmpty();
   }

   public int getNumberOfContactPoints()
   {
      return contactPoints.size();
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public RigidBodyBasics getContactingRigidBody()
   {
      return rigidBody;
   }

   public FramePoint3DBasics getContactPoint(int index)
   {
      return contactPoints.get(index);
   }

   public FrameVector3DBasics getContactNormal()
   {
      return contactNormal;
   }

   /**
    * Sets the rho weights, which are described in
    * {@code ControllerCoreOptimizationSettings.getRhoWeight()} If a rho weight is to Double.NaN, the
    * controller core will use the default rho weight set in the MomentumOptimizationSetting
    */
   public void setRhoWeight(int pointIndex, double rhoWeight)
   {
      rhoWeights.set(pointIndex, rhoWeight);
   }

   public double getRhoWeight(int pointIndex)
   {
      return rhoWeights.get(pointIndex);
   }

   public boolean isUseHighCoPDamping()
   {
      return useHighCoPDamping;
   }

   public boolean hasMaxContactPointNormalForce()
   {
      return hasMaxContactPointNormalForce;
   }

   public boolean getHasContactStateChanged()
   {
      return hasContactStateChanged;
   }

   public double getMaxContactPointNormalForce(int pointIndex)
   {
      return maxContactPointNormalForces.get(pointIndex);
   }

   public RigidBodyTransform getContactFramePoseInBodyFixedFrame()
   {
      return contactFramePoseInBodyFixedFrame;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.PLANE_CONTACT_STATE;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof PlaneContactStateCommand)
      {
         PlaneContactStateCommand other = (PlaneContactStateCommand) object;

         if (rigidBody != other.rigidBody)
            return false;
         if (Double.compare(coefficientOfFriction, other.coefficientOfFriction) != 0)
            return false;
         if (getNumberOfContactPoints() != other.getNumberOfContactPoints())
            return false;
         if (!contactPoints.equals(other.contactPoints))
            return false;
         if (!contactNormal.equals(other.contactNormal))
            return false;
         if (useHighCoPDamping != other.useHighCoPDamping)
            return false;
         if (hasContactStateChanged != other.hasContactStateChanged)
            return false;
         if (hasMaxContactPointNormalForce != other.hasMaxContactPointNormalForce)
            return false;
         if (!maxContactPointNormalForces.equals(other.maxContactPointNormalForces))
            return false;
         for (int contactPointIndex = 0; contactPointIndex < getNumberOfContactPoints(); contactPointIndex++)
         {
            if (Double.compare(rhoWeights.get(contactPointIndex), other.rhoWeights.get(contactPointIndex)) != 0)
               return false;
         }

         if (contactFramePoseInBodyFixedFrame.containsNaN())
         {
            if (!other.contactFramePoseInBodyFixedFrame.containsNaN())
               return false;
         }
         else if (!contactFramePoseInBodyFixedFrame.equals(other.contactFramePoseInBodyFixedFrame))
         {
            return false;
         }

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": contacting rigid body = " + rigidBody + ", number of contact points = " + getNumberOfContactPoints();
   }
}
