package us.ihmc.commonWalkingControlModules.controllerCore.command;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public class DesiredExternalWrenchHolder implements Settable<DesiredExternalWrenchHolder>
{
   private final List<RigidBodyBasics> bodiesWithExternalWrench = new ArrayList<>();
   private final RecyclingArrayList<Wrench> desiredExternalWrenches = new RecyclingArrayList<>(Wrench::new);

   /**
    * This is used for lookups only and is populated from the {@link #desiredExternalWrenches}. It should not be modified
    * directly and does not represent the state of the class.
    */
   private final transient TLongObjectMap<Wrench> desiredExternalWrenchMap = new TLongObjectHashMap<>();

   public DesiredExternalWrenchHolder()
   {
   }

   public DesiredExternalWrenchHolder(Collection<RigidBodyBasics> rigidBodies)
   {
      rigidBodies.forEach(this::registerRigidBody);
   }

   public void clear()
   {
      bodiesWithExternalWrench.clear();
      desiredExternalWrenches.clear();
      desiredExternalWrenchMap.clear();
   }

   public void registerRigidBody(RigidBodyBasics rigidBody)
   {
      if (bodiesWithExternalWrench.contains(rigidBody))
         throw new RuntimeException("The body: " + rigidBody.getName() + " has already been registered.");

      Wrench wrench = desiredExternalWrenches.add();
      wrench.setToNaN(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
      bodiesWithExternalWrench.add(rigidBody);
      desiredExternalWrenchMap.put(rigidBody.hashCode(), wrench);
   }

   public void registerRigidBody(RigidBodyBasics rigidBody, WrenchReadOnly desiredExternalWrench)
   {
      if (bodiesWithExternalWrench.contains(rigidBody))
         throw new RuntimeException("The body: " + rigidBody.getName() + " has already been registered.");

      Wrench wrench = desiredExternalWrenches.add();
      wrench.setIncludingFrame(desiredExternalWrench);
      wrench.changeFrame(rigidBody.getBodyFixedFrame());
      bodiesWithExternalWrench.add(rigidBody);
      desiredExternalWrenchMap.put(rigidBody.hashCode(), wrench);
   }

   public int getNumberOfBodiesWithDesiredExternalWrench()
   {
      return bodiesWithExternalWrench.size();
   }

   public RigidBodyBasics getRigidBody(int bodyIndex)
   {
      return bodiesWithExternalWrench.get(bodyIndex);
   }

   public void setDesiredExternalWrench(WrenchReadOnly desiredExternalWrench, RigidBodyBasics rigidBody)
   {
      Wrench wrench = desiredExternalWrenchMap.get(rigidBody.hashCode());
      wrench.setIncludingFrame(desiredExternalWrench);
      wrench.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public void setDesiredExternalWrench(ReferenceFrame referenceFrame, Vector3DReadOnly torque, Vector3DReadOnly force, RigidBodyBasics rigidBody)
   {
      Wrench wrench = desiredExternalWrenchMap.get(rigidBody.hashCode());
      if (torque != null)
         wrench.getAngularPart().set(torque);
      else
         wrench.getAngularPart().setToZero();
      if (force != null)
         wrench.getLinearPart().set(force);
      else
         wrench.getLinearPart().setToZero();
      wrench.setReferenceFrame(referenceFrame);
      wrench.setBodyFrame(rigidBody.getBodyFixedFrame());
      wrench.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public void setDesiredExternalWrench(ReferenceFrame referenceFrame, Vector3DReadOnly torque, Vector3DReadOnly force, int bodyIndex)
   {
      RigidBodyBasics rigidBody = getRigidBody(bodyIndex);
      Wrench wrench = desiredExternalWrenches.get(bodyIndex);
      if (torque != null)
         wrench.getAngularPart().set(torque);
      else
         wrench.getAngularPart().setToZero();
      if (force != null)
         wrench.getLinearPart().set(force);
      else
         wrench.getLinearPart().setToZero();
      wrench.setReferenceFrame(referenceFrame);
      wrench.setBodyFrame(rigidBody.getBodyFixedFrame());
      wrench.changeFrame(rigidBody.getBodyFixedFrame());
   }

   public boolean getDesiredExternalWrench(WrenchBasics desiredExternalWrenchToPack, RigidBodyBasics rigidBody)
   {
      Wrench wrench = desiredExternalWrenchMap.get(rigidBody.hashCode());
      if (wrench != null)
      {
         desiredExternalWrenchToPack.setIncludingFrame(wrench);
         return true;
      }
      else
      {
         return false;
      }
   }

   public Wrench getDesiredExternalWrench(RigidBodyBasics rigidBody)
   {
      return desiredExternalWrenchMap.get(rigidBody.hashCode());
   }

   public Wrench getDesiredExternalWrench(int bodyIndex)
   {
      return desiredExternalWrenches.get(bodyIndex);
   }

   @Override
   public void set(DesiredExternalWrenchHolder other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfBodiesWithDesiredExternalWrench(); i++)
      {
         RigidBodyBasics rigidBody = other.getRigidBody(i);
         registerRigidBody(rigidBody, other.getDesiredExternalWrench(i));
      }
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof DesiredExternalWrenchHolder)
      {
         DesiredExternalWrenchHolder other = (DesiredExternalWrenchHolder) obj;
         if (getNumberOfBodiesWithDesiredExternalWrench() != other.getNumberOfBodiesWithDesiredExternalWrench())
            return false;
         for (int i = 0; i < getNumberOfBodiesWithDesiredExternalWrench(); i++)
         {
            RigidBodyBasics rigidBody = getRigidBody(i);
            if (!getDesiredExternalWrench(i).equals(other.getDesiredExternalWrench(rigidBody)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }
}
