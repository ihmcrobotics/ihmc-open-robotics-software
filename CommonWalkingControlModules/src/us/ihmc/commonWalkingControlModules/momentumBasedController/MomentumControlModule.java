package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

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

   // One method for setting everything
//   public abstract void setMomentumModuleDataObject(MomentumModuleDataObject momentumModuleDataObject);

   // Setting desired   
   public abstract void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData);

   public abstract void setDesiredJointAcceleration(DesiredJointAccelerationCommand desiredJointAccelerationCommand);
   public abstract void setDesiredSpatialAcceleration(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand);
   public abstract void setDesiredPointAcceleration(DesiredPointAccelerationCommand desiredPointAccelerationCommand);

   // Known external wrenches
   public abstract void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench);

   // Solve
   public abstract MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, Map<ContactableCylinderBody, ? extends CylindricalContactState> cylinderContactStates, RobotSide upcomingSupportSide) throws NoConvergenceException;

}
