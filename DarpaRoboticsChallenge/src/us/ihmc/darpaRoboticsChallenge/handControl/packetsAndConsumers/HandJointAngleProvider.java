package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.HandJointName;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.graphics3DAdapter.GraphicsUpdatable;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class HandJointAngleProvider implements ObjectConsumer<HandJointAnglePacket>
{
   private final SideDependentList<HashMap<HandJointName, OneDoFJoint>> handJoints = new SideDependentList<HashMap<HandJointName, OneDoFJoint>>();
   private final AtomicReference<HandJointAnglePacket> packet = new AtomicReference<HandJointAnglePacket>();
   private final Object lock = new Object();
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
	   HandJointAnglePacket handJointAngles = packet.getAndSet(null);
	   
	   if(handJointAngles == null)
		   return;
	   
	   synchronized (lock)
	   {
		   HashMap<HandJointName, OneDoFJoint> joints = handJoints.get(handJointAngles.getRobotSide());
		   if (joints != null)
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
				   if (graphicsUpdatable != null) graphicsUpdatable.update();
			   }
		   }
	   }
   }

   public void consumeObject(HandJointAnglePacket object)
   {
	   packet.set(object);
	   
	   updateHandModel();
   }
}
