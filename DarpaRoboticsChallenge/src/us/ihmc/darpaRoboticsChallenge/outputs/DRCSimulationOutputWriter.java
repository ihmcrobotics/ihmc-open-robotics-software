package us.ihmc.darpaRoboticsChallenge.outputs;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotDampingParameters;
import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCSimulationOutputWriter extends SDFPerfectSimulatedOutputWriter implements DRCOutputWriter
{
   
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   private final RobotVisualizer robotVisualizer;
   
   double[] prevError;
   public DRCSimulationOutputWriter(SDFRobot robot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, RobotVisualizer robotVisualizer)
   {
      super(robot);
      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;
      this.robotVisualizer = robotVisualizer;
   }

   public void writeAfterController(long timestamp)
   {
      // Do not write here, because it will set the robot's torques while the simulation is running
   }

   public void writeAfterEstimator()
   {
      if (robotVisualizer != null)
      {
         robotVisualizer.update();
      }
   }

   public void writeAfterSimulationTick()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         
         Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = revoluteJoints.get(i);
         
         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();

         
         
         pinJoint.setTau(revoluteJoint.getTau());
         pinJoint.setKp(revoluteJoint.getKp());
         pinJoint.setKd(revoluteJoint.getKd());
         pinJoint.setqDesired(revoluteJoint.getqDesired());
         pinJoint.setQdDesired(revoluteJoint.getQdDesired());
         
      }
      
      if(dynamicGraphicObjectsListRegistry != null)
      {
         dynamicGraphicObjectsListRegistry.update();
      }
   }

   public void setFullRobotModel(SDFFullRobotModel fullRobotModel)
   {
      super.setFullRobotModel(fullRobotModel);
      
      OneDoFJoint[] joints = ROSAtlasJointMap.getJointMap(fullRobotModel.getOneDoFJointsAsMap());
      for(int i = 0; i < joints.length; i++)
      {
         joints[i].setDampingParameter(DRCRobotDampingParameters.getAtlasDamping(i));
      }
      
      prevError = new double[revoluteJoints.size()];
      

      if(robotVisualizer != null)
      {
         robotVisualizer.setFullRobotModel(fullRobotModel);
      }
      
   }

   public void setEstimatorModel(SDFFullRobotModel estimatorModel)
   {
      
   }

}
