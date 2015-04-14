package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.dataobjects.HandJointName;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.utilities.GraphicsUpdatable;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class HandJointAngleProvider implements PacketConsumer<HandJointAnglePacket>
{
   private final SideDependentList<HashMap<HandJointName, OneDoFJoint>> handJoints = new SideDependentList<HashMap<HandJointName, OneDoFJoint>>();

   private final SideDependentList<AtomicReference<HandJointAnglePacket>> packets = new SideDependentList<AtomicReference<HandJointAnglePacket>>();
   private ArrayList<GraphicsUpdatable> graphicsToUpdate = new ArrayList<GraphicsUpdatable>();
   private HandModel handModel;

   public HandJointAngleProvider(SDFFullRobotModel fullRobotModel, HandModel handModel)
   {
      this.handModel = handModel;

      if (handModel != null)
      {
         for (RobotSide side : RobotSide.values())
         {
            final HashMap<HandJointName, OneDoFJoint> joints = new HashMap<HandJointName, OneDoFJoint>();
            packets.put(side, new AtomicReference<HandJointAnglePacket>());

            for (HandJointName jointName : handModel.getHandJointNames())
            {
               joints.put(jointName, fullRobotModel.getOneDoFJointByName(jointName.getJointName(side)));
            }

            handJoints.put(side, joints);
         }
      }
   }

   public void addGraphicsUpdateable(GraphicsRobot updateable)
   {
      graphicsToUpdate.add(updateable);
   }

   public void updateHandModel()
   {
      if (handModel != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            HandJointAnglePacket handJointAngles = packets.get(robotSide).getAndSet(null);
            HashMap<HandJointName, OneDoFJoint> joints = handJoints.get(robotSide);

            if (handJointAngles != null && joints != null)
            {
               for (HandJointName jointName : handModel.getHandJointNames())
               {
                  if (jointName != null)
                  {
                     OneDoFJoint oneDoFJoint = joints.get(jointName);
                     if (oneDoFJoint != null)
                     {
                        oneDoFJoint.setQ(handJointAngles.getJointAngle(jointName));
                     }
                  }
               }

               for (GraphicsUpdatable graphicsUpdatable : graphicsToUpdate)
               {
                  if (graphicsUpdatable != null)
                     graphicsUpdatable.update();
               }

            }
         }

      }
   }

   public void receivedPacket(HandJointAnglePacket object)
   {
      packets.get(object.robotSide).set(object);
   }
}
