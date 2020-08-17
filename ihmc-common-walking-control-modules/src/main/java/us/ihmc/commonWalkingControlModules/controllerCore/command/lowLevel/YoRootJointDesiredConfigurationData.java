package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoRootJointDesiredConfigurationData implements RootJointDesiredConfigurationDataBasics
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameQuaternion orientation;
   private final YoFramePoint3D position;
   private final YoFrameVector3D linearVelocity;
   private final YoFrameVector3D angularVelocity;
   private final YoFrameVector3D linearAcceleration;
   private final YoFrameVector3D angularAcceleration;

   private final DMatrixRMaj desiredConfiguration = new DMatrixRMaj(7, 0);
   private final DMatrixRMaj desiredVelocity = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj desiredAcceleration = new DMatrixRMaj(6, 0);

   public YoRootJointDesiredConfigurationData(FloatingJointBasics rootJoint, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry("RootJointDesiredConfigurationData");
      parentRegistry.addChild(registry);
      ReferenceFrame frameAfterJoint = rootJoint.getFrameAfterJoint();

      String namePrefix = "rootJointLowLevel";
      orientation = new YoFrameQuaternion(namePrefix + "DesiredOrientation", worldFrame, registry);
      position = new YoFramePoint3D(namePrefix + "DesiredPosition", worldFrame, registry);
      linearVelocity = new YoFrameVector3D(namePrefix + "DesiredLinearVelocity", frameAfterJoint, registry);
      angularVelocity = new YoFrameVector3D(namePrefix + "DesiredAngularVelocity", frameAfterJoint, registry);
      linearAcceleration = new YoFrameVector3D(namePrefix + "DesiredLinearAcceleration", frameAfterJoint, registry);
      angularAcceleration = new YoFrameVector3D(namePrefix + "DesiredAngularAcceleration", frameAfterJoint, registry);

      clear();
   }

   @Override
   public void clear()
   {
      clearConfiguration();
      clearVelocity();
      clearAcceleration();
   }

   private void clearConfiguration()
   {
      orientation.setToNaN();
      position.setToNaN();
   }

   private void clearVelocity()
   {
      linearVelocity.setToNaN();
      angularVelocity.setToNaN();
   }

   private void clearAcceleration()
   {
      linearAcceleration.setToNaN();
      angularAcceleration.setToNaN();
   }

   @Override
   public void set(RootJointDesiredConfigurationDataReadOnly other)
   {
      setConfiguration(other);
      setVelocity(other);
      setAcceleration(other);
   }

   @Override
   public void completeWith(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!hasDesiredConfiguration())
         setConfiguration(other);

      if (!hasDesiredVelocity())
         setVelocity(other);

      if (!hasDesiredAcceleration())
         setAcceleration(other);
   }

   private void setConfiguration(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!other.hasDesiredConfiguration())
      {
         clearConfiguration();
         return;
      }

      setDesiredConfiguration(other.getDesiredConfiguration());
   }

   private void setVelocity(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!other.hasDesiredVelocity())
      {
         clearVelocity();
         return;
      }

      setDesiredVelocity(other.getDesiredVelocity());
   }

   private void setAcceleration(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!other.hasDesiredAcceleration())
      {
         clearAcceleration();
         return;
      }

      setDesiredAcceleration(other.getDesiredAcceleration());
   }

   @Override
   public void setDesiredAccelerationFromJoint(FloatingJointBasics sixDoFJoint)
   {
      desiredAcceleration.reshape(6, 1);
      sixDoFJoint.getJointAcceleration(0, desiredAcceleration);
      setDesiredAcceleration(desiredAcceleration);
   }

   @Override
   public void setDesiredConfiguration(FrameQuaternionReadOnly orientation, FramePoint3DReadOnly position)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   @Override
   public void setDesiredVelocity(FrameVector3DReadOnly angularVelocity, FrameVector3DReadOnly linearVelocity)
   {
      this.angularVelocity.set(angularVelocity);
      this.linearVelocity.set(linearVelocity);
   }

   @Override
   public void setDesiredAcceleration(FrameVector3DReadOnly angularAcceleration, FrameVector3DReadOnly linearAcceleration)
   {
      this.angularAcceleration.set(angularAcceleration);
      this.linearAcceleration.set(linearAcceleration);
   }

   @Override
   public void setDesiredConfiguration(DMatrixRMaj q, int startIndex)
   {
      orientation.set(startIndex, q);
      position.set(startIndex + 4, q);
   }

   @Override
   public void setDesiredVelocity(DMatrixRMaj qd, int startIndex)
   {
      angularVelocity.set(startIndex, qd);
      linearVelocity.set(startIndex + 3, qd);
   }

   @Override
   public void setDesiredAcceleration(DMatrixRMaj qdd, int startIndex)
   {
      angularAcceleration.set(startIndex, qdd);
      linearAcceleration.set(startIndex + 3, qdd);
   }

   @Override
   public boolean hasDesiredConfiguration()
   {
      return !orientation.containsNaN() && !position.containsNaN();
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return !angularVelocity.containsNaN() && !linearVelocity.containsNaN();
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return !angularAcceleration.containsNaN() && !linearAcceleration.containsNaN();
   }

   @Override
   public DMatrixRMaj getDesiredConfiguration()
   {
      if (!hasDesiredConfiguration())
      {
         desiredConfiguration.reshape(0, 0);
         return desiredConfiguration;
      }
      else
      {
         desiredConfiguration.reshape(7, 1);
         orientation.get(0, desiredConfiguration);
         position.get(4, desiredConfiguration);
         return desiredConfiguration;
      }
   }

   @Override
   public DMatrixRMaj getDesiredVelocity()
   {
      if (!hasDesiredVelocity())
      {
         desiredVelocity.reshape(0, 0);
         return desiredVelocity;
      }
      else
      {
         desiredVelocity.reshape(6, 1);
         angularVelocity.get(0, desiredVelocity);
         linearVelocity.get(3, desiredVelocity);
         return desiredVelocity;
      }
   }

   @Override
   public DMatrixRMaj getDesiredAcceleration()
   {
      if (!hasDesiredAcceleration())
      {
         desiredAcceleration.reshape(0, 0);
         return desiredAcceleration;
      }
      else
      {
         desiredAcceleration.reshape(6, 1);
         angularAcceleration.get(0, desiredAcceleration);
         linearAcceleration.get(3, desiredAcceleration);
         return desiredAcceleration;
      }
   }
}
