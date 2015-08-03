package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotics.robotSide.RobotSide;


public interface PelvisHeightControlModule
{
   /**
    * Returns the z-component of the desired force on the pelvis, expressed in pelvis frame, given the desired height
    * @param desiredPelvisHeightInWorld the desired height of the origin of the pelvis frame in world frame
    * @param supportLeg is the supporting leg. If in double support, pass in null
    * @return
    */
   double doPelvisHeightControl(double desiredPelvisHeightInWorld, RobotSide supportLeg);
}
