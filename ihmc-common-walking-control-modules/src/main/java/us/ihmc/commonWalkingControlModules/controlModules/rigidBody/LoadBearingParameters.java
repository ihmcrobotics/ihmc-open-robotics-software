package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

public interface LoadBearingParameters
{
   default LoadBearingControlMode getDefaultControlMode()
   {
      return LoadBearingControlMode.JOINTSPACE;
   }

   
}
