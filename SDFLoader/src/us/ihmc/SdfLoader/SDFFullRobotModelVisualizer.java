package us.ihmc.SdfLoader;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SDFFullRobotModelVisualizer implements RobotVisualizer
{
   private final String name;
   private final SDFRobot robot;
   private final double estimatorDT;

   private SimulationConstructionSet scs;
   
   private SixDoFJoint rootJoint;
   private final ArrayList<Pair<OneDegreeOfFreedomJoint,OneDoFJoint>> revoluteJoints = new ArrayList<Pair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
  
   public SDFFullRobotModelVisualizer(SDFRobot robot, int estimatorTicksPerSCSTickAndUpdate, double estimatorDT)
   {
	   
      this.name = robot.getName() + "SimulatedSensorReader";
      
      this.robot = robot;
      this.estimatorDT = estimatorDT;
   }

   public void registerSCS(SimulationConstructionSet scs)
   {
      this.scs = scs;
   }
   
   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }
   
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      this.rootJoint = fullRobotModel.getRootJoint();
      
      revoluteJoints.clear();
      OneDoFJoint[] revoluteJointsArray = fullRobotModel.getOneDoFJoints();
      
      for (OneDoFJoint revoluteJoint : revoluteJointsArray)
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDoFJoint(name);
         
         Pair<OneDegreeOfFreedomJoint,OneDoFJoint> jointPair = new Pair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, revoluteJoint);
         this.revoluteJoints.add(jointPair);
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
      if(rootJoint != null)
      {
         Transform3D rootTransform = rootJoint.getJointTransform3D();
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
      robot.setTime(robot.getTime() + estimatorDT);
      if (scs != null)
      {
         scs.tickAndUpdate();
      }
   }

   public void setRootRegistry(YoVariableRegistry registry)
   {
      // TODO Auto-generated method stub
      
   }

   public void close()
   {
      // no-op
   }
}
