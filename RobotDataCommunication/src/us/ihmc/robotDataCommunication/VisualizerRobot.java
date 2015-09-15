package us.ihmc.robotDataCommunication;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class VisualizerRobot extends SDFHumanoidRobot
{
   private final YoVariableRegistry reducedRegistry;

   
   public VisualizerRobot(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap)
   {
      super(generalizedSDFRobotModel, null, sdfJointNameMap, false);
      this.reducedRegistry = new YoVariableRegistry(generalizedSDFRobotModel.getName());

      reducedRegistry.registerVariable(getRootJoint().getQx());
      reducedRegistry.registerVariable(getRootJoint().getQy());
      reducedRegistry.registerVariable(getRootJoint().getQz());

      reducedRegistry.registerVariable(getRootJoint().getQdx());
      reducedRegistry.registerVariable(getRootJoint().getQdy());
      reducedRegistry.registerVariable(getRootJoint().getQdz());


      reducedRegistry.registerVariable(getRootJoint().getQuaternionQs());
      reducedRegistry.registerVariable(getRootJoint().getQuaternionQx());
      reducedRegistry.registerVariable(getRootJoint().getQuaternionQy());
      reducedRegistry.registerVariable(getRootJoint().getQuaternionQz());

      reducedRegistry.registerVariable(getRootJoint().getAngularVelocityX());
      reducedRegistry.registerVariable(getRootJoint().getAngularVelocityY());
      reducedRegistry.registerVariable(getRootJoint().getAngularVelocityZ());
      
      for(OneDegreeOfFreedomJoint joint : getOneDoFJoints())
      {
         reducedRegistry.registerVariable(joint.getQ());
         reducedRegistry.registerVariable(joint.getQD());
      }
      
   }
   
   @Override
   public YoVariableRegistry getRobotsYoVariableRegistry()
   {
      if(this.reducedRegistry == null)
      {
         // Hack to avoid null registry errors on startup.
         return super.getRobotsYoVariableRegistry();
      }
      else
      {
         return this.reducedRegistry;
      }
   }
}
