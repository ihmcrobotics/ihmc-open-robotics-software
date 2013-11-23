package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.iRobot.iRobotHandModel.iRobotHandJointNameMinimal;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.graphics.GraphicsUpdatable;

public class HandJointAngleProvider implements ObjectConsumer<HandJointAnglePacket>
{
   private final SideDependentList<EnumMap<iRobotHandJointNameMinimal, OneDoFJoint>> handJoints = new SideDependentList<EnumMap<iRobotHandJointNameMinimal, OneDoFJoint>>();

   private final Object lock = new Object();
   private ArrayList<GraphicsUpdatable> graphicsToUpdate = new ArrayList<GraphicsUpdatable>();

   public HandJointAngleProvider(SDFFullRobotModel fullRobotModel)
   {
      for (RobotSide side : RobotSide.values())
      {
         final EnumMap<iRobotHandJointNameMinimal, OneDoFJoint> joints = new EnumMap<iRobotHandJointNameMinimal, OneDoFJoint>(iRobotHandJointNameMinimal.class);

         for (iRobotHandJointNameMinimal jointName : iRobotHandJointNameMinimal.values())
         {
            joints.put(jointName, fullRobotModel.getOneDoFJointByName(jointName.toLowerCase()));
         }
         handJoints.put(side, joints);
      }
   }

   public void addGraphicsUpdateable(GraphicsUpdatable updateable)
   {
      graphicsToUpdate.add(updateable);
   }

   public void consumeObject(HandJointAnglePacket object)
   {
      synchronized (lock)
      {
         EnumMap<iRobotHandJointNameMinimal, OneDoFJoint> joints = handJoints.get(object.getRobotSide());

         for (iRobotHandJointNameMinimal jointName : iRobotHandJointNameMinimal.values())
         {
            joints.get(jointName).setQ(object.getJointAngle(jointName));
         }

         for (GraphicsUpdatable graphicsUpdatable : graphicsToUpdate)
         {
            graphicsUpdatable.update();
         }
      }
   }
}
