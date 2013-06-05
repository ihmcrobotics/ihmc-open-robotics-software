package us.ihmc.darpaRoboticsChallenge.outputs;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotDampingParameters;
import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCSimulationOutputWriter extends SDFPerfectSimulatedOutputWriter implements DRCOutputWriter
{
   
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   
   double[] prevError;
   public DRCSimulationOutputWriter(SDFRobot robot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(robot);
      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;
   }

   public void writeAfterController(long timestamp)
   {
      // Do not write here, because it will set the robot's torques while the simulation is running
   }

   public void writeAfterEstimator()
   {
      // Nothing to do here
   }

   public void writeAfterSimulationTick()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         
         Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = revoluteJoints.get(i);
         
         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();

         double q = pinJoint.getQ().getDoubleValue();
         double qError = revoluteJoint.getqDesired() - q;
//         double qdError = (qError -  prevError[i]) / 0.001;
         double qdError = revoluteJoint.getQdDesired() - pinJoint.getQD().getDoubleValue();
         double control = (revoluteJoint.getKp() * qError)
               + (revoluteJoint.getKd() * qdError);
         double tau = revoluteJoint.getTau();
         
         prevError[i] = qError;
         pinJoint.setTau(control + tau);
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
      
   }

   public void setEstimatorModel(SDFFullRobotModel estimatorModel)
   {
      
   }

}
