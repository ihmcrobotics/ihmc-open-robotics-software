package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class YoRootJointDesiredConfigurationData implements RootJointDesiredConfigurationDataReadOnly
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameQuaternion orientation;
   private final YoFramePoint3D position;
   private final YoFrameVector3D linearVelocity;
   private final YoFrameVector3D angularVelocity;
   private final YoFrameVector3D linearAcceleration;
   private final YoFrameVector3D angularAcceleration;

   private final DenseMatrix64F desiredConfiguration = new DenseMatrix64F(7, 0);
   private final DenseMatrix64F desiredVelocity = new DenseMatrix64F(6, 0);
   private final DenseMatrix64F desiredAcceleration = new DenseMatrix64F(6, 0);

   public YoRootJointDesiredConfigurationData(FloatingInverseDynamicsJoint rootJoint, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry("RootJointDesiredConfigurationData");
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

   public void set(RootJointDesiredConfigurationDataReadOnly other)
   {
      setConfiguration(other);
      setVelocity(other);
      setAcceleration(other);
   }

   public void completeWith(RootJointDesiredConfigurationDataReadOnly other)
   {
//      if(orientation.containsNaN())
//         PrintTools.debug("Orientation contains Nan");
//      if(position.containsNaN())
//         PrintTools.debug("Position contains Nan");
//      if(angularVelocity.containsNaN())
//         PrintTools.debug("Angular velocity contains Nan");
//      if(linearVelocity.containsNaN())
//         PrintTools.debug("Linear velocity contains Nan");
//      if(angularAcceleration.containsNaN())
//         PrintTools.debug("Angular acceleration contains Nan");
//      if(linearAcceleration.containsNaN())
//         PrintTools.debug("Linear acceleration contains Nan");
      
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
      
      DenseMatrix64F otherDesiredConfiguration = other.getDesiredConfiguration();
      MatrixTools.extractYoFrameQuaternionFromEJMLVector(orientation, otherDesiredConfiguration, 0);
      MatrixTools.extractFixedFrameTupleFromEJMLVector(position, otherDesiredConfiguration, 4);
   }

   private void setVelocity(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!other.hasDesiredVelocity())
      {
         clearVelocity();
         return;
      }

      DenseMatrix64F otherDesiredVelocity = other.getDesiredVelocity();
      MatrixTools.extractFixedFrameTupleFromEJMLVector(angularVelocity, otherDesiredVelocity, 0);
      MatrixTools.extractFixedFrameTupleFromEJMLVector(linearVelocity, otherDesiredVelocity, 3);
   }

   private void setAcceleration(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!other.hasDesiredAcceleration())
      {
         clearAcceleration();
         return;
      }

      DenseMatrix64F otherDesiredAcceleration = other.getDesiredAcceleration();
      MatrixTools.extractFixedFrameTupleFromEJMLVector(angularAcceleration, otherDesiredAcceleration, 0);
      MatrixTools.extractFixedFrameTupleFromEJMLVector(linearAcceleration, otherDesiredAcceleration, 3);
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
   public DenseMatrix64F getDesiredConfiguration()
   {
      if (!hasDesiredConfiguration())
      {
         desiredConfiguration.reshape(0, 0);
         return desiredConfiguration;
      }
      else
      {
         desiredConfiguration.reshape(7, 1);
         MatrixTools.insertFrameQuaternionIntoEJMLVector(orientation, desiredConfiguration, 0);
         MatrixTools.insertFrameTupleIntoEJMLVector(position, desiredConfiguration, 4);
         return desiredConfiguration;
      }
   }

   @Override
   public DenseMatrix64F getDesiredVelocity()
   {
      if (!hasDesiredVelocity())
      {
         desiredVelocity.reshape(0, 0);
         return desiredVelocity;
      }
      else
      {
         desiredVelocity.reshape(6, 1);
         MatrixTools.insertFrameTupleIntoEJMLVector(angularVelocity, desiredVelocity, 0);
         MatrixTools.insertFrameTupleIntoEJMLVector(linearVelocity, desiredVelocity, 3);
         return desiredVelocity;
      }
   }

   @Override
   public DenseMatrix64F getDesiredAcceleration()
   {
      if (!hasDesiredAcceleration())
      {
         desiredAcceleration.reshape(0, 0);
         return desiredAcceleration;
      }
      else
      {
         desiredAcceleration.reshape(6, 1);
         MatrixTools.insertFrameTupleIntoEJMLVector(angularAcceleration, desiredAcceleration, 0);
         MatrixTools.insertFrameTupleIntoEJMLVector(linearAcceleration, desiredAcceleration, 3);
         return desiredAcceleration;
      }
   }
}
