package us.ihmc.communication.remote.serialization;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.PacketProducer;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class JointConfigurationDataSender extends PacketProducer<JointConfigurationData>
{
   private final InverseDynamicsJoint[] joints;
   private final PacketCommunicator packetCommunicator;

   public JointConfigurationDataSender(RigidBody rootBody, PacketCommunicator packetCommunicator)
   {
//      this.joints = ScrewTools.computeJointsInOrder(rootBody); //deprecated method
      this.joints = ScrewTools.computeSubtreeJoints(rootBody);
      this.packetCommunicator = packetCommunicator;
   }

   public void send()
   {
      JointConfigurationData jointConfigurationData = new JointConfigurationData(joints);
      packetCommunicator.send(jointConfigurationData);
   }

   public void startUpdateThread(long updatePeriodInMilliseconds)
   {
      Thread thread = new Thread(new JointConfigurationUpdateThread(updatePeriodInMilliseconds));
      thread.setDaemon(true);
      thread.start();
   }

   private class JointConfigurationUpdateThread implements Runnable
   {
      private final long updatePeriodInMilliseconds;

      private JointConfigurationUpdateThread(long updatePeriodInMilliseconds)
      {
         this.updatePeriodInMilliseconds = updatePeriodInMilliseconds;
      }

      public void run()
      {
         while (true)
         {
            send();

            try
            {
               Thread.sleep(updatePeriodInMilliseconds);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      }
   }
}
