package us.ihmc.simulationToolkit.visualizers;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

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
   private YoVariableRegistry robotRegistry;
   private FloatingInverseDynamicsJoint rootJoint;
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJoint>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
  
   public FullRobotModelVisualizer(SimulationConstructionSet scs, FullRobotModel fullRobotModel, double updateDT)
   {   
      this.fullRobot = fullRobotModel;
      this.scs = scs;
      this.robot = (FloatingRootJointRobot) scs.getRobots()[0];
      this.name = robot.getName() + "Simulated";    
      this.updateDT = updateDT;
      this.robotRegistry = robot.getRobotsYoVariableRegistry();
      this.setMainRegistry(robotRegistry, fullRobotModel, null);
   }

   public FloatingRootJointRobot getSDFRobot()
   {
      return robot;
   }
   
   public YoVariableRegistry getRobotRegistry()
   {
      return robotRegistry;
   }

   
   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }
   
   @Override
   public void setMainRegistry(YoVariableRegistry registry, FullRobotModel fullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.rootJoint = fullRobotModel.getRootJoint();
      
      revoluteJoints.clear();
      OneDoFJoint[] revoluteJointsArray = fullRobotModel.getOneDoFJoints();
      
      for (OneDoFJoint revoluteJoint : revoluteJointsArray)
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);
         
         ImmutablePair<OneDegreeOfFreedomJoint,OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, revoluteJoint);
         this.revoluteJoints.add(jointPair);
      }
      
      if( robot.getRobotsYoVariableRegistry() != registry)
      {
         robot.addYoVariableRegistry(registry); 
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

   
   private final Vector3d tempPosition = new Vector3d();
   private final Quat4d tempOrientation = new Quat4d();
   
   public void update(long timestamp)
   {
      fullRobot.updateFrames();
      
      if(rootJoint != null)
      {
         RigidBodyTransform rootTransform = rootJoint.getJointTransform3D();
         rootTransform.get(tempOrientation, tempPosition);
         robot.setOrientation(tempOrientation);
         robot.setPositionInWorld(tempPosition);
      }
      
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

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

   public void close()
   {
      // no-op
   }

   @Override
   public void update(long timestamp, YoVariableRegistry registry)
   {
      update(timestamp);
   }

   @Override
   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      robot.addYoVariableRegistry(registry);
   }

}
