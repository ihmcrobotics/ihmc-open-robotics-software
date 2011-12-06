package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import java.util.ArrayList;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.utilities.screwTheory.Wrench;

public interface SpineLungingControlModule extends SpineControlModule
{
   public abstract void doMaintainDesiredChestOrientation();
   
   public abstract void getSpineTorques(SpineTorques spineTorquesToPack);
   
   public abstract void setWrench(Wrench wrench);
   
   public abstract void setGainsToZero(ArrayList<SpineJointName> spineJointsWithZeroGain);

   public abstract void setGains();
   
   public void scaleGainsBasedOnLungeAxis(Vector2d lungeAxis);

   public abstract void setHipXYTorque(Vector3d desiredLungingTorqeicpRecoverDecelerateState);
}

