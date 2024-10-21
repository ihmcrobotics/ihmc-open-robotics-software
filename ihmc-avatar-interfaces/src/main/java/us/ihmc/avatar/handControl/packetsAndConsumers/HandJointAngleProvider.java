package us.ihmc.avatar.handControl.packetsAndConsumers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;

public class HandJointAngleProvider implements PacketConsumer<HandJointAnglePacket>
{
   public static final SideDependentList<Boolean> HAND_ENABLED = new SideDependentList<>(true, true);
   private final SideDependentList<HashMap<HandJointName, OneDoFJointBasics>> handJoints = new SideDependentList<HashMap<HandJointName, OneDoFJointBasics>>();

   private final SideDependentList<AtomicReference<HandJointAnglePacket>> packets = new SideDependentList<AtomicReference<HandJointAnglePacket>>();
   private ArrayList<GraphicsUpdatable> graphicsToUpdate = new ArrayList<GraphicsUpdatable>();
   private SideDependentList<HandModel> handModels;

   public HandJointAngleProvider(FullRobotModel fullRobotModel, SideDependentList<HandModel> handModels)
   {
      this.handModels = handModels;

      if (handModels == null)
      {
         LogTools.warn("Hand models is null.");
      }
      else
      {
         for (RobotSide side : handModels.sides())
         {
            final HashMap<HandJointName, OneDoFJointBasics> joints = new HashMap<HandJointName, OneDoFJointBasics>();
            packets.put(side, new AtomicReference<HandJointAnglePacket>());

            for (HandJointName jointName : handModels.get(side).getHandJointNames())
            {
               joints.put(jointName, fullRobotModel.getOneDoFJointByName(jointName.getJointName(side)));
            }

            handJoints.put(side, joints);
         }
      }
   }

   public void addGraphicsUpdateable(GraphicsUpdatable updateable)
   {
      graphicsToUpdate.add(updateable);
   }

   public void updateHandModel()
   {
      if (handModels != null)
      {
         for (RobotSide robotSide : handModels.sides())
         {
            HandJointAnglePacket handJointAngles = packets.get(robotSide).getAndSet(null);
            HashMap<HandJointName, OneDoFJointBasics> joints = handJoints.get(robotSide);

            if (handJointAngles != null && joints != null)
            {
               for (HandJointName jointName : handModels.get(robotSide).getHandJointNames())
               {
                  if (jointName != null)
                  {
                     OneDoFJointBasics oneDoFJoint = joints.get(jointName);
                     if (oneDoFJoint != null)
                     {
                        if (HAND_ENABLED.get(robotSide))
                        {
                           double jointAngle = HumanoidMessageTools.unpackJointAngle(handJointAngles, jointName);
                           oneDoFJoint.setQ(jointAngle);
                        }
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
      if (handModels != null)
      {
         RobotSide side = RobotSide.fromByte(object.getRobotSide());
         if (handModels.containsKey(side))
         {
            packets.get(side).set(object);
         }
      }
   }
}
