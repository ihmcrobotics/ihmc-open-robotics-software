package us.ihmc.darpaRoboticsChallenge.outputs;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class DRCSimulationOutputWriter extends SDFPerfectSimulatedOutputWriter implements DRCOutputWriter
{
   
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
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
      for (Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();

         double control = (revoluteJoint.getKp() * (revoluteJoint.getqDesired() - pinJoint.getQ().getDoubleValue()))
               + (revoluteJoint.getKd() * (revoluteJoint.getQdDesired() - pinJoint.getQD().getDoubleValue()));
         double tau = revoluteJoint.getTau();
         
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
   }

   public void setEstimatorModel(SDFFullRobotModel estimatorModel)
   {
      
   }

}
