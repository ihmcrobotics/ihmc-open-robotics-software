package us.ihmc.avatar.handControl.packetsAndConsumers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class HandJointAngleProvider implements PacketConsumer<HandJointAnglePacket>
{
   public static final SideDependentList<Boolean> HAND_ENABLED = new SideDependentList<>(true, true);
   private final SideDependentList<HashMap<HandJointName, OneDoFJointBasics>> handJoints = new SideDependentList<HashMap<HandJointName, OneDoFJointBasics>>();

   private final SideDependentList<AtomicReference<HandJointAnglePacket>> packets = new SideDependentList<AtomicReference<HandJointAnglePacket>>();
   private ArrayList<GraphicsUpdatable> graphicsToUpdate = new ArrayList<GraphicsUpdatable>();
   private HandModel handModel;

   public HandJointAngleProvider(FullRobotModel fullRobotModel, HandModel handModel)
   {
      this.handModel = handModel;

      if (handModel == null)
      {
         PrintTools.warn("Hand model is null.");
      }
      else
      {
         for (RobotSide side : RobotSide.values)
         {
            final HashMap<HandJointName, OneDoFJointBasics> joints = new HashMap<HandJointName, OneDoFJointBasics>();
            packets.put(side, new AtomicReference<HandJointAnglePacket>());

            for (HandJointName jointName : handModel.getHandJointNames())
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
      if (handModel != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            HandJointAnglePacket handJointAngles = packets.get(robotSide).getAndSet(null);
            HashMap<HandJointName, OneDoFJointBasics> joints = handJoints.get(robotSide);

            if (handJointAngles != null && joints != null)
            {
               for (HandJointName jointName : handModel.getHandJointNames())
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

      if (handModel != null)
         packets.get(RobotSide.fromByte(object.getRobotSide())).set(object);
   }
}
