package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PlaneContactStateCommand implements InverseDynamicsCommand<PlaneContactStateCommand>
{
   private RigidBody rigidBody;
   private String rigidBodyName;
   private double coefficientOfFriction = Double.NaN;
   private final int initialSize = 8;
   private final FrameTupleArrayList<FramePoint3D> contactPoints = FrameTupleArrayList.createFramePointArrayList(initialSize);
   private final FrameVector3D contactNormal = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);

   private boolean useHighCoPDamping = false;
   private boolean hasContactStateChanged;

   private boolean hasMaxContactPointNormalForce = false;
   private final RecyclingArrayList<MutableDouble> maxContactPointNormalForces = new RecyclingArrayList<>(initialSize, MutableDouble.class);
   
   /**
    * Holds on to the rho weights, which are described in {@code ControllerCoreOptimizationSettings.getRhoWeight()}
    * If a rho weight is to Double.NaN, the controller core will use the default rho weight set in the MomentumOptimizationSetting
    */
   private final RecyclingArrayList<MutableDouble> rhoWeights = new RecyclingArrayList<>(initialSize, MutableDouble.class);

   public PlaneContactStateCommand()
   {
      clearContactPoints();
   }

   public void setHasContactStateChanged(boolean hasContactStateChanged)
   {
      this.hasContactStateChanged = hasContactStateChanged;
   }

   public void setContactingRigidBody(RigidBody rigidBody)
   {
      this.rigidBody = rigidBody;
      rigidBodyName = rigidBody.getName();
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void clearContactPoints()
   {
      contactPoints.clear();
      maxContactPointNormalForces.clear();
      rhoWeights.clear();
   }

   public void addPointInContact(FramePoint3D newPointInContact)
   {
      contactPoints.add().setIncludingFrame(newPointInContact);
      maxContactPointNormalForces.add().setValue(Double.POSITIVE_INFINITY);
      rhoWeights.add().setValue(Double.NaN);
   }

   public void addPointInContact(FramePoint2D newPointInContact)
   {
      contactPoints.add().setIncludingFrame(newPointInContact, 0.0);
      maxContactPointNormalForces.add().setValue(Double.POSITIVE_INFINITY);
      rhoWeights.add().setValue(Double.NaN);
   }

   public void setPointsInContact(List<FramePoint3D> newPointsInContact)
   {
      contactPoints.copyFromListAndTrimSize(newPointsInContact);

      maxContactPointNormalForces.clear();
      for (int i = 0; i < contactPoints.size(); i++)
      {
         maxContactPointNormalForces.add().setValue(Double.POSITIVE_INFINITY);
         rhoWeights.add().setValue(Double.NaN);
      }
   }

   public void setPoint2dsInContact(ReferenceFrame contactFrame, List<Point2D> newPointsInContact)
   {
      clearContactPoints();
      for (int i = 0; i < newPointsInContact.size(); i++)
      {
         contactPoints.add().setIncludingFrame(contactFrame, newPointsInContact.get(i), 0.0);
         maxContactPointNormalForces.add().setValue(Double.POSITIVE_INFINITY);
         rhoWeights.add().setValue(Double.NaN);
      }
   }

   public void setMaxContactPointNormalForce(int contactPointIndex, double maxNormalForce)
   {
      hasMaxContactPointNormalForce = true;
      maxContactPointNormalForces.get(contactPointIndex).setValue(maxNormalForce);
   }

   public void setContactNormal(FrameVector3D contactNormal)
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

   public RigidBody getContactingRigidBody()
   {
      return rigidBody;
   }

   public String getContactingRigidBodyName()
   {
      return rigidBodyName;
   }

   public void getContactPoint(int index, FramePoint3D contactPointToPack)
   {
      contactPointToPack.setIncludingFrame(contactPoints.get(index));
   }

   public void getContactNormal(FrameVector3D contactNormalToPack)
   {
      contactNormalToPack.setIncludingFrame(contactNormal);
   }
   
   /**
    * Sets the rho weights, which are described in {@code ControllerCoreOptimizationSettings.getRhoWeight()}
    * If a rho weight is to Double.NaN, the controller core will use the default rho weight set in the MomentumOptimizationSetting
    */
   public void setRhoWeight(int pointIndex, double rhoWeight)
   {
      rhoWeights.get(pointIndex).setValue(rhoWeight);
   }

   public double getRhoWeight(int pointIndex)
   {
      return rhoWeights.get(pointIndex).doubleValue();
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
      return maxContactPointNormalForces.get(pointIndex).doubleValue();
   }

   @Override
   public void set(PlaneContactStateCommand other)
   {
      rigidBody = other.rigidBody;
      rigidBodyName = other.rigidBodyName;
      coefficientOfFriction = other.coefficientOfFriction;
      contactPoints.copyFromListAndTrimSize(other.contactPoints);
      contactNormal.setIncludingFrame(other.contactNormal);
      useHighCoPDamping = other.useHighCoPDamping;
      hasContactStateChanged = other.hasContactStateChanged;

      hasMaxContactPointNormalForce = other.hasMaxContactPointNormalForce;
      
      maxContactPointNormalForces.clear();
      rhoWeights.clear();
      
      for (int i = 0; i < other.contactPoints.size(); i++)
      {
         maxContactPointNormalForces.add().setValue(other.maxContactPointNormalForces.get(i));
         rhoWeights.add().setValue(other.rhoWeights.get(i));
      }
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.PLANE_CONTACT_STATE;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": contacting rigid body = " + rigidBody + ", number of contact points = " + getNumberOfContactPoints();
      return ret;
   }
}
