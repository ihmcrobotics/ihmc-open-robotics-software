package us.ihmc.SdfLoader;

import java.util.ArrayList;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;

/*
 * Simple class that makes simpler to visualize a FullRobotModel inside the SimulationConstructionSet.
 * Basically it keeps in synch the whole body state of the FullRobotModel and the SDFRobot.
 * The user shall call the method update(...) when he wants the data to refreshed on the SDF side.
 * The main registry is added automatically by the constructor and can be accessed using getRobotRegistry().  
 */

public class FullRobotModelVisualizer implements RobotVisualizer
{
   private final String name;
   private final SDFRobot robot;
   private final double updateDT;
   private final FullRobotModel fullRobot;

   private SimulationConstructionSet scs;
   private YoVariableRegistry robotRegistry;
   private SixDoFJoint rootJoint;
   private final ArrayList<Pair<OneDegreeOfFreedomJoint,OneDoFJoint>> revoluteJoints = new ArrayList<Pair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
  
   public FullRobotModelVisualizer(SimulationConstructionSet scs, FullRobotModel fullRobotModel, double updateDT)
   {   
      this.fullRobot = fullRobotModel;
      this.scs = scs;
      this.robot = (SDFRobot) scs.getRobots()[0];
      this.name = robot.getName() + "Simulated";    
      this.updateDT = updateDT;
      this.robotRegistry = robot.getRobotsYoVariableRegistry();
      this.setMainRegistry(robotRegistry, fullRobotModel, null);
   }

   public SDFRobot getSDFRobot()
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
         
         Pair<OneDegreeOfFreedomJoint,OneDoFJoint> jointPair = new Pair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, revoluteJoint);
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
      
      for (Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();

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
