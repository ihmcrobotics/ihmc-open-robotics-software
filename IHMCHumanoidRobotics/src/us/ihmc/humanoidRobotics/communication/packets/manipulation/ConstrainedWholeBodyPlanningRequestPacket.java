package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.manipulation.ConstrainedEndEffectorTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class ConstrainedWholeBodyPlanningRequestPacket extends Packet<ConstrainedWholeBodyPlanningRequestPacket>
{
   public ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;
   
   public FullHumanoidRobotModel toolboxFullRobotModel;
   public HumanoidReferenceFrames referenceFrames;
   
   
      
   public double tempInputValue;

   public ConstrainedWholeBodyPlanningRequestPacket()
   {

   }
   
   public void setConstrainedTrajectoryType(ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory)
   {
      this.constrainedEndEffectorTrajectory = constrainedEndEffectorTrajectory;
   }
   
   public void setFullRobotModel(FullHumanoidRobotModel toolboxFullRobotModel)
   {
      this.toolboxFullRobotModel = toolboxFullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(toolboxFullRobotModel);
      /*
       * solver, midZUpFrame, eeTrajectory will be put in controller.
       * random region.
       */
   }

   public void setTempValue(double value)
   {
      tempInputValue = value;
   }

   public void setTaskSpace()
   {

   }

   public void setEndEffectorTrajectory()
   {

   }

   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningRequestPacket other, double epsilon)
   {
      return false;
   }

}
