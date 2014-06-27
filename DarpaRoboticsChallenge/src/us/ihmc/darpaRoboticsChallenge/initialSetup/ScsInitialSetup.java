package us.ihmc.darpaRoboticsChallenge.initialSetup;

import com.yobotics.simulationconstructionset.DynamicIntegrationMethod;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;
import com.yobotics.simulationconstructionset.physics.ScsPhysics;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.steppingStones.SteppingStones;
import us.ihmc.graphics3DAdapter.GroundProfile;

/*
 * SCS Initial Setup
 * Implementations of this interface are designed to configure settings in SCS (the simulation environment).  
 * This includes physical things like gravity and terrain, as well as logical things like the simulation time step and the recording frequency for YoVariables.
 * 
 * Notice: To date, gravity and terrain are aspects of Robot, so the Robot object passed will be modified.
 */

public interface ScsInitialSetup
{
// FIXME: delete this method from this interface once ground contact profiles aren't guified via the first robot in the list of robots with which you create the sim...
   public abstract void initializeRobot(Robot robot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry);
   
   public abstract void initializeSimulation(SimulationConstructionSet scs);
   public abstract double getDT();
   public abstract boolean getDrawGroundProfile();
   public abstract int getSimulationDataBufferSize();
   public abstract int getRecordFrequency();
   public abstract double getGravity();
   public abstract ScsPhysics createPhysics( ScsCollisionConfigure collisionConfigure , YoVariableRegistry registry );
   public abstract GroundProfile getGroundProfile();
//   public abstract SteppingStones getSteppingStones();
   public abstract DynamicIntegrationMethod getDynamicIntegrationMethod();

   public abstract boolean getInitializeEstimatorToActual(); //TODO: Probably should be somewhere else, but where?
}
