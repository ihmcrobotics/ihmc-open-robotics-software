package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 4/25/13
 */
public interface MomentumControlModule
{
   // Initialization methods
   public abstract void initialize();
   public abstract void reset();
   public abstract void resetGroundReactionWrenchFilter();

   // Setting desired   
   public abstract void setDesiredRateOfChangeOfMomentum(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand);

   public abstract void setDesiredJointAcceleration(DesiredJointAccelerationCommand desiredJointAccelerationCommand);
   public abstract void setDesiredSpatialAcceleration(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand);
   public abstract void setDesiredPointAcceleration(DesiredPointAccelerationCommand desiredPointAccelerationCommand);

   // Known external wrenches
   public abstract void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench);

   // Solve
   public abstract MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportSide) throws MomentumControlModuleException;

}
