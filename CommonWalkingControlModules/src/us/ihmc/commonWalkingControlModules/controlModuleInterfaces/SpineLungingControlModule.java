package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import java.util.ArrayList;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.screwTheory.Wrench;

public interface SpineLungingControlModule extends SpineControlModule
{
   public abstract void doMaintainDesiredChestOrientation();
   
   public abstract void getSpineTorques(SpineTorques spineTorquesToPack);
   
   public abstract void setWrench(Wrench wrench);
   
   public abstract void setGainsToZero(ArrayList<SpineJointName> spineJointsWithZeroGain);

   public abstract void setGains();
   
   public abstract void scaleGainsBasedOnLungeAxis(Vector2d lungeAxis);

   public abstract void setHipXYTorque(Vector3d desiredLungingTorqeicpRecoverDecelerateState);
   
   public abstract void doCMPControl(FramePoint2d desiredCMP, FrameVector2d lungeAxis);
   
   public abstract void doConstantTorqueAroundLungeAxis(FrameVector2d lungeAxis, double constantTorque);
}

