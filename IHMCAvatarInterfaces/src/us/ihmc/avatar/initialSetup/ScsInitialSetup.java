package us.ihmc.avatar.initialSetup;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.DynamicIntegrationMethod;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

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
   public abstract void initializeRobot(Robot robot, YoGraphicsListRegistry yoGraphicsListRegistry);
   
   public abstract void initializeSimulation(SimulationConstructionSet scs);
   public abstract double getDT();
   public abstract boolean getDrawGroundProfile();
   public abstract int getSimulationDataBufferSize();
   public abstract int getRecordFrequency();
   public abstract double getGravity();
//   public abstract ScsPhysics createPhysics( ScsCollisionConfigure collisionConfigure , YoVariableRegistry registry );
   public abstract GroundProfile3D getGroundProfile3D();
//   public abstract SteppingStones getSteppingStones();
   public abstract DynamicIntegrationMethod getDynamicIntegrationMethod();

   public abstract boolean getInitializeEstimatorToActual(); //TODO: Probably should be somewhere else, but where?
}
