package us.ihmc.simulationToolkit.visualizers;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;

/*
 * Simple class that makes simpler to visualize a FullRobotModel inside the SimulationConstructionSet.
 * Basically it keeps in synch the whole body state of the FullRobotModel and the SDFRobot.
 * The user shall call the method update(...) when he wants the data to refreshed on the SDF side.
 * The main registry is added automatically by the constructor and can be accessed using getRobotRegistry().  
 */

public class FullRobotModelVisualizer implements RobotVisualizer
{
   private final String name;
   private final FloatingRootJointRobot robot;
   private final double updateDT;
   private final FullRobotModel fullRobot;

   private SimulationConstructionSet scs;
   private YoRegistry robotRegistry;
   private FloatingJointBasics rootJoint;
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJointBasics>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>>();
  
   private volatile long latestTimestamp = 0;
   
   public FullRobotModelVisualizer(SimulationConstructionSet scs, FullRobotModel fullRobotModel, double updateDT)
   {   
      fullRobot = fullRobotModel;
      this.scs = scs;
      robot = (FloatingRootJointRobot) scs.getRobots()[0];
      name = robot.getName() + "Simulated";    
      this.updateDT = updateDT;
      robotRegistry = robot.getRobotsYoRegistry();
      rootJoint = fullRobotModel.getRootJoint();
      revoluteJoints.clear();
      OneDoFJointBasics[] revoluteJointsArray = fullRobotModel.getOneDoFJoints();
      for (OneDoFJointBasics revoluteJoint : revoluteJointsArray)
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);
         
         ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJointBasics> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics>(oneDoFJoint, revoluteJoint);
         revoluteJoints.add(jointPair);
      }
      setMainRegistry(robotRegistry, null, null);
   }

   public FloatingRootJointRobot getSDFRobot()
   {
      return robot;
   }
   
   public YoRegistry getRobotRegistry()
   {
      return robotRegistry;
   }

   
   public void initialize()
   {
   }

   public YoRegistry getYoVariableRegistry()
   {
      return null;
   }
   
   @Override
   public void setMainRegistry(YoRegistry registry, RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
            
      if( robot.getRobotsYoRegistry() != registry)
      {
         robot.addYoRegistry(registry); 
      }
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   @Override
   public void update(long timestamp)
   {
      fullRobot.updateFrames();
      latestTimestamp = timestamp;
      
      if(rootJoint != null)
      {
         robot.setOrientation(rootJoint.getJointPose().getOrientation());
         robot.setPositionInWorld(rootJoint.getJointPose().getPosition());
      }
      
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJointBasics> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJointBasics revoluteJoint = jointPair.getRight();

         pinJoint.setQ(revoluteJoint.getQ());
         pinJoint.setQd(revoluteJoint.getQd());
         pinJoint.setTau(revoluteJoint.getTau());
      }
      robot.setTime(robot.getTime() + updateDT);
      if (scs != null)
      {
         scs.tickAndUpdate();
      }
   }

   @Override
   public void close()
   {
      // no-op
   }

   @Override
   public void update(long timestamp, YoRegistry registry)
   {
      update(timestamp);
   }

   @Override
   public void addRegistry(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      robot.addYoRegistry(registry);
   }

   @Override
   public long getLatestTimestamp()
   {
      return latestTimestamp;
   }

}
