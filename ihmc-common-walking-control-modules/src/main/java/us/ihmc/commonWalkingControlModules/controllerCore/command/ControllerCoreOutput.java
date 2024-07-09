package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class ControllerCoreOutput implements ControllerCoreOutputReadOnly
{
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;
   private final DesiredExternalWrenchHolder desiredExternalWrenchHolder;
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentumRate = new FrameVector3D();
   private final FrameVector3D angularMomentumRate = new FrameVector3D();
   private final RootJointDesiredConfigurationData rootJointDesiredConfigurationData = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   @Deprecated
   private final transient JointDesiredOutputListBasics jointDesiredOutputList;

   /**
    * Do not use this constructor. It will break the barrier scheduler and the
    * {@link #set(ControllerCoreOutput)} and {@link #equals(Object)} methods will not work.
    * <p>
    * This is a simple command that is only holding on to data. Use the getters of this class to get
    * its data. This constructor should be removed after the thread refactor summer 2019.
    */
   @Deprecated
   public ControllerCoreOutput(CenterOfPressureDataHolder centerOfPressureDataHolder,
                               DesiredExternalWrenchHolder desiredExternalWrenchHolder,
                               OneDoFJointBasics[] controlledOneDoFJoints,
                               JointDesiredOutputList lowLevelControllerOutput)
   {
      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      this.desiredExternalWrenchHolder = desiredExternalWrenchHolder;
      linearMomentum.setToNaN(ReferenceFrame.getWorldFrame());
      angularMomentum.setToNaN(ReferenceFrame.getWorldFrame());
      linearMomentumRate.setToNaN(ReferenceFrame.getWorldFrame());
      angularMomentumRate.setToNaN(ReferenceFrame.getWorldFrame());
      if (lowLevelControllerOutput != null)
         jointDesiredOutputList = lowLevelControllerOutput;
      else
         jointDesiredOutputList = new JointDesiredOutputList(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder = null;
   }

   public ControllerCoreOutput()
   {
      centerOfPressureDataHolder = new CenterOfPressureDataHolder();
      desiredExternalWrenchHolder = new DesiredExternalWrenchHolder();
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
      jointDesiredOutputList = lowLevelOneDoFJointDesiredDataHolder;
   }

   public void setCenterOfPressureData(CenterOfPressureDataHolder centerOfPressureDataHolder)
   {
      this.centerOfPressureDataHolder.set(centerOfPressureDataHolder);
   }

   public CenterOfPressureDataHolder getCenterOfPressureData()
   {
      return centerOfPressureDataHolder;
   }

   public void setDesiredExternalWrenchData(DesiredExternalWrenchHolder desiredExternalWrenchHolder)
   {
      this.desiredExternalWrenchHolder.set(desiredExternalWrenchHolder);
   }

   public DesiredExternalWrenchHolder getDesiredExternalWrenchData()
   {
      return desiredExternalWrenchHolder;
   }

   public void setDesiredCenterOfPressure(FramePoint2DReadOnly cop, RigidBodyBasics rigidBody)
   {
      centerOfPressureDataHolder.setCenterOfPressure(cop, rigidBody);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint2DBasics copToPack, RigidBodyBasics rigidBody)
   {
      centerOfPressureDataHolder.getCenterOfPressure(copToPack, rigidBody);
   }

   public void setDesiredExternalWrench(WrenchReadOnly desiredExternalWrench, RigidBodyBasics rigidBody)
   {
      desiredExternalWrenchHolder.setDesiredExternalWrench(desiredExternalWrench, rigidBody);
   }

   @Override
   public boolean getDesiredExternalWrench(WrenchBasics desiredExternalWrenchToPack, RigidBodyBasics rigidBody)
   {
      return desiredExternalWrenchHolder.getDesiredExternalWrench(desiredExternalWrenchToPack, rigidBody);
   }

   public void setLinearMomentum(FrameVector3DReadOnly linearMomentum)
   {
      this.linearMomentum.setIncludingFrame(linearMomentum);
   }

   @Override
   public FrameVector3D getLinearMomentum()
   {
      return linearMomentum;
   }

   public void setAngularMomentum(FrameVector3DReadOnly angularMomentum)
   {
      this.angularMomentum.setIncludingFrame(angularMomentum);
   }

   @Override
   public FrameVector3D getAngularMomentum()
   {
      return angularMomentum;
   }

   public void setLinearMomentumRate(FrameVector3DReadOnly linearMomentumRate)
   {
      this.linearMomentumRate.setIncludingFrame(linearMomentumRate);
   }

   @Override
   public FrameVector3D getLinearMomentumRate()
   {
      return linearMomentumRate;
   }

   public void setAngularMomentumRate(FrameVector3DReadOnly angularMomentumRate)
   {
      this.angularMomentumRate.setIncludingFrame(angularMomentumRate);
   }

   @Override
   public FrameVector3D getAngularMomentumRate()
   {
      return angularMomentumRate;
   }

   public void setRootJointDesiredConfigurationData(RootJointDesiredConfigurationDataReadOnly rootJointDesiredConfigurationData)
   {
      this.rootJointDesiredConfigurationData.set(rootJointDesiredConfigurationData);
   }

   @Override
   public RootJointDesiredConfigurationDataReadOnly getRootJointDesiredConfigurationData()
   {
      return rootJointDesiredConfigurationData;
   }

   public void setLowLevelOneDoFJointDesiredDataHolder(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      this.jointDesiredOutputList.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }

   /**
    * This is depreceted and will be removed after a cleanup of this class. Use
    * {@link #getLowLevelOneDoFJointDesiredDataHolderPreferred()} instead.
    */
   @Override
   @Deprecated
   public JointDesiredOutputListBasics getLowLevelOneDoFJointDesiredDataHolder()
   {
      return jointDesiredOutputList;
   }

   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointDesiredDataHolderPreferred()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public void set(ControllerCoreOutput other)
   {
      if (lowLevelOneDoFJointDesiredDataHolder == null)
         throw new RuntimeException("You used the deprecated constructor set is not supported in that case.");

      centerOfPressureDataHolder.set(other.centerOfPressureDataHolder);
      desiredExternalWrenchHolder.set(other.desiredExternalWrenchHolder);
      linearMomentum.setIncludingFrame(other.linearMomentum);
      angularMomentum.setIncludingFrame(other.angularMomentum);
      linearMomentumRate.setIncludingFrame(other.linearMomentumRate);
      angularMomentumRate.setIncludingFrame(other.angularMomentumRate);
      rootJointDesiredConfigurationData.set(other.rootJointDesiredConfigurationData);
      lowLevelOneDoFJointDesiredDataHolder.set(other.lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (lowLevelOneDoFJointDesiredDataHolder == null)
         throw new RuntimeException("You used the deprecated constructor equals is not supported in that case.");

      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof ControllerCoreOutput)
      {
         ControllerCoreOutput other = (ControllerCoreOutput) obj;
         if (!centerOfPressureDataHolder.equals(other.centerOfPressureDataHolder))
            return false;
         if (!desiredExternalWrenchHolder.equals(other.desiredExternalWrenchHolder))
            return false;
         if (!linearMomentum.equals(other.linearMomentum))
            return false;
         if (!angularMomentum.equals(other.angularMomentum))
            return false;
         if (!linearMomentumRate.equals(other.linearMomentumRate))
            return false;
         if (!angularMomentumRate.equals(other.angularMomentumRate))
            return false;
         if (!rootJointDesiredConfigurationData.equals(other.rootJointDesiredConfigurationData))
            return false;
         if (!lowLevelOneDoFJointDesiredDataHolder.equals(other.lowLevelOneDoFJointDesiredDataHolder))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
