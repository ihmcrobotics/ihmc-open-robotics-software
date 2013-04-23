package us.ihmc.SdfLoader;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RawOutputWriter;

public class SDFPerfectSimulatedOutputWriter implements RawOutputWriter
{
   private final String name;
   private final SDFRobot robot;
   private final ArrayList<Pair<OneDegreeOfFreedomJoint,OneDoFJoint>> revoluteJoints = new ArrayList<Pair<OneDegreeOfFreedomJoint, OneDoFJoint>>();

   public SDFPerfectSimulatedOutputWriter(SDFRobot robot)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
   }
   
   public SDFPerfectSimulatedOutputWriter(SDFRobot robot, SDFFullRobotModel fullRobotModel)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;

      setFullRobotModel(fullRobotModel);
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

   public void write()
   {
      for (Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();

         pinJoint.setTau(revoluteJoint.getTau());
      }
   }

}
