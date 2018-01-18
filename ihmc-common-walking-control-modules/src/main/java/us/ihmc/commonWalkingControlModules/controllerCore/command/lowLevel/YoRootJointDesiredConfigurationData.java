package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;

public class YoRootJointDesiredConfigurationData implements RootJointDesiredConfigurationDataReadOnly
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameQuaternion orientation;
   private final YoFramePoint position;
   private final YoFrameVector linearVelocity;
   private final YoFrameVector angularVelocity;
   private final YoFrameVector linearAcceleration;
   private final YoFrameVector angularAcceleration;

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
      position = new YoFramePoint(namePrefix + "DesiredPosition", worldFrame, registry);
      linearVelocity = new YoFrameVector(namePrefix + "DesiredLinearVelocity", frameAfterJoint, registry);
      angularVelocity = new YoFrameVector(namePrefix + "DesiredAngularVelocity", frameAfterJoint, registry);
      linearAcceleration = new YoFrameVector(namePrefix + "DesiredLinearAcceleration", frameAfterJoint, registry);
      angularAcceleration = new YoFrameVector(namePrefix + "DesiredAngularAcceleration", frameAfterJoint, registry);

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
      MatrixTools.extractYoFrameTupleFromEJMLVector(position, otherDesiredConfiguration, 4);
   }

   private void setVelocity(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!other.hasDesiredVelocity())
      {
         clearVelocity();
         return;
      }

      DenseMatrix64F otherDesiredVelocity = other.getDesiredVelocity();
      MatrixTools.extractYoFrameTupleFromEJMLVector(angularVelocity, otherDesiredVelocity, 0);
      MatrixTools.extractYoFrameTupleFromEJMLVector(linearVelocity, otherDesiredVelocity, 3);
   }

   private void setAcceleration(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!other.hasDesiredAcceleration())
      {
         clearAcceleration();
         return;
      }

      DenseMatrix64F otherDesiredAcceleration = other.getDesiredAcceleration();
      MatrixTools.extractYoFrameTupleFromEJMLVector(angularAcceleration, otherDesiredAcceleration, 0);
      MatrixTools.extractYoFrameTupleFromEJMLVector(linearAcceleration, otherDesiredAcceleration, 3);
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
