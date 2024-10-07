package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.DesiredExternalWrenchHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class ControllerCoreOutPutDataHolder implements ControllerCoreOutputReadOnly
{
   private boolean hasCenterOfPressure = false;
   private final CenterOfPressureDataHolder centerOfPressureDataHolder = new CenterOfPressureDataHolder();

   private boolean hasDesiredExternalWrench = false;
   private DesiredExternalWrenchHolder desiredExternalWrenchHolder = new DesiredExternalWrenchHolder();

   private boolean hasLinearMomentum = false;
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private boolean hasAngularMomentum = false;
   private final FrameVector3D angularMomentum = new FrameVector3D();

   private boolean hasLinearMomentumRate = false;
   private final FrameVector3D linearMomentumRate = new FrameVector3D();
   private boolean hasAngularMomentumRate = false;
   private final FrameVector3D angularMomentumRate = new FrameVector3D();

   private boolean hasRootJointDesiredConfiguration = false;
   private final RootJointDesiredConfigurationData rootJointDesiredConfigurationData = new RootJointDesiredConfigurationData();

   private boolean hasLowLevelOneDoFJointControllerCoreOutputDataHolder = false;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointControllerCoreOutputDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   public ControllerCoreOutPutDataHolder()
   {
      clear();
   }

   public void clear()
   {
      hasAngularMomentum = false;
      hasLowLevelOneDoFJointControllerCoreOutputDataHolder = false;
      hasAngularMomentumRate = false;
      hasLinearMomentum = false;
      hasLinearMomentumRate = false;
      hasCenterOfPressure = false;
      hasDesiredExternalWrench = false;
      hasRootJointDesiredConfiguration = false;

      angularMomentum.setToNaN();
      angularMomentumRate.setToNaN();
      linearMomentum.setToNaN();
      linearMomentumRate.setToNaN();

      centerOfPressureDataHolder.clear();
      desiredExternalWrenchHolder.clear();
      rootJointDesiredConfigurationData.clear();
      lowLevelOneDoFJointControllerCoreOutputDataHolder.clear();
   }
   public void setControllerCoreOutputDataHolder(ControllerCoreOutput controllerCoreOutput)
   {
      setDesiredCenterOfPressureDataHolder(controllerCoreOutput.getCenterOfPressureData());
      desiredExternalWrenchHolder = controllerCoreOutput.getDesiredExternalWrenchData();

      setAngularMomentum(controllerCoreOutput.getAngularMomentum());
      setAngularMomentumRate(controllerCoreOutput.getAngularMomentumRate());

      setLinearMomentum(controllerCoreOutput.getLinearMomentum());
      setLinearMomentumRate(controllerCoreOutput.getLinearMomentumRate());
      setRootJointDesiredConfigurationData(controllerCoreOutput.getRootJointDesiredConfigurationData());
      setLowLevelOneDoFJointControllerCoreOutputDataHolder(controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolderPreferred());
   }

   public void setDesiredCenterOfPressureDataHolder(CenterOfPressureDataHolder centerOfPressureDataHolder)
   {
      this.centerOfPressureDataHolder.set(centerOfPressureDataHolder);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint2DBasics copToPack, RigidBodyBasics rigidBody)
   {
      this.centerOfPressureDataHolder.getCenterOfPressure(copToPack, rigidBody);
   }

   public CenterOfPressureDataHolder getDesiredCenterOfPressureDataHolder()
   {
      return this.centerOfPressureDataHolder;
   }

   public void setDesiredExternalWrench(WrenchReadOnly desiredExternalWrench, RigidBodyBasics rigidBody)
   {
      this.desiredExternalWrenchHolder.setDesiredExternalWrench(desiredExternalWrench, rigidBody);
   }

   public DesiredExternalWrenchHolder getDesiredExternalWrenchHolder()
   {
      return this.desiredExternalWrenchHolder;
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

   @Override
   public JointDesiredOutputListReadOnly getLowLevelOneDoFJointDesiredDataHolder()
   {
      return null;
   }

   public void setLowLevelOneDoFJointControllerCoreOutputDataHolder(JointDesiredOutputListReadOnly lowLevelOneDoFJointControllerCoreOutputDataHolder)
   {
      this.lowLevelOneDoFJointControllerCoreOutputDataHolder.overwriteWith(lowLevelOneDoFJointControllerCoreOutputDataHolder);
   }

   //TODO this will be replace the getLowLevelOneDoFJointDesiredDataHolderPreferered. This one returns lowLevelOneDoFJointDesiredDataHolder
   //while getLowLevelOneDoFJointDesiredDataHolder returns jointDesiredOutputList.
   public JointDesiredOutputListReadOnly getLowLevelOneDoFJointControllerCoreDesiredDataHolder()
   {
      return lowLevelOneDoFJointControllerCoreOutputDataHolder;
   }

   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointControllerCoreOutPutDesiredDataHolder()
   {
      return lowLevelOneDoFJointControllerCoreOutputDataHolder;
   }

   public void set(ControllerCoreOutPutDataHolder other)
   {
      if (this.lowLevelOneDoFJointControllerCoreOutputDataHolder == null)
         throw new RuntimeException("You used the deprecated constructor set is not supported in that case. ");

      this.centerOfPressureDataHolder.set(other.centerOfPressureDataHolder);
      this.desiredExternalWrenchHolder.set(other.desiredExternalWrenchHolder);
      this.linearMomentum.setIncludingFrame(other.linearMomentum);
      this.linearMomentumRate.setIncludingFrame(other.linearMomentumRate);
      this.angularMomentum.setIncludingFrame(other.angularMomentum);
      this.angularMomentumRate.setIncludingFrame(other.angularMomentumRate);
      this.rootJointDesiredConfigurationData.set(other.rootJointDesiredConfigurationData);
      this.lowLevelOneDoFJointControllerCoreOutputDataHolder.set(other.lowLevelOneDoFJointControllerCoreOutputDataHolder);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (lowLevelOneDoFJointControllerCoreOutputDataHolder == null)
         throw new RuntimeException("You used the deprecated constructor equals is not supported in that case");

      if (obj == this)
      {
         return true;
      }

      else if (obj instanceof ControllerCoreOutPutDataHolder other)
      {
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
         return lowLevelOneDoFJointControllerCoreOutputDataHolder.equals(other.lowLevelOneDoFJointControllerCoreOutputDataHolder);
      }
      else
      {
         return false;
      }
   }
}
