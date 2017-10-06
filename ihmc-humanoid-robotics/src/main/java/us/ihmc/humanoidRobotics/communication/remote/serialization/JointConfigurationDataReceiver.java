package us.ihmc.humanoidRobotics.communication.remote.serialization;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class JointConfigurationDataReceiver implements PacketConsumer<JointConfigurationData>
{
   private final InverseDynamicsJoint[] joints;
   
   

   public JointConfigurationDataReceiver(RigidBody rootBody)
   {
//      this.joints = ScrewTools.computeJointsInOrder(rootBody); //deprecated method
      this.joints = ScrewTools.computeSubtreeJoints(rootBody);
   }
   public synchronized void receivedPacket(JointConfigurationData jointConfigurationData)
   {
      jointConfigurationData.putConfigurationsIntoJoints(joints);
      notifyAll();
   }

}
