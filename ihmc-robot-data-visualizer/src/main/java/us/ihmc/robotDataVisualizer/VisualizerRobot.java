package us.ihmc.robotDataVisualizer;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class VisualizerRobot extends FloatingRootJointRobot
{
   private final YoRegistry reducedRegistry;


   public VisualizerRobot(RobotDescription robotDescription)
   {
      super(robotDescription, false, false);
      this.reducedRegistry = new YoRegistry(robotDescription.getName());

      reducedRegistry.addVariable(t);
      reducedRegistry.addVariable(getRootJoint().getQx());
      reducedRegistry.addVariable(getRootJoint().getQy());
      reducedRegistry.addVariable(getRootJoint().getQz());

      reducedRegistry.addVariable(getRootJoint().getQdx());
      reducedRegistry.addVariable(getRootJoint().getQdy());
      reducedRegistry.addVariable(getRootJoint().getQdz());


      reducedRegistry.addVariable(getRootJoint().getQuaternionQs());
      reducedRegistry.addVariable(getRootJoint().getQuaternionQx());
      reducedRegistry.addVariable(getRootJoint().getQuaternionQy());
      reducedRegistry.addVariable(getRootJoint().getQuaternionQz());

      reducedRegistry.addVariable(getRootJoint().getAngularVelocityX());
      reducedRegistry.addVariable(getRootJoint().getAngularVelocityY());
      reducedRegistry.addVariable(getRootJoint().getAngularVelocityZ());

      for(OneDegreeOfFreedomJoint joint : getOneDegreeOfFreedomJoints())
      {
         reducedRegistry.addVariable(joint.getQYoVariable());
         reducedRegistry.addVariable(joint.getQDYoVariable());
      }

   }

   @Override
   public YoRegistry getRobotsYoRegistry()
   {
      if(this.reducedRegistry == null)
      {
         // Hack to avoid null registry errors on startup.
         return super.getRobotsYoRegistry();
      }
      else
      {
         return this.reducedRegistry;
      }
   }
}
