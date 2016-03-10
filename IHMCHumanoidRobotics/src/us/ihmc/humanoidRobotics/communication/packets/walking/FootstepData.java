package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.robotics.robotSide.RobotSide;

//To be deleted after the controller rewrite is done. 
//Keeping for right now in order to transform old script files.
public class FootstepData  extends IHMCRosApiMessage<FootstepDataListMessage>
{
   public RobotSide robotSide;
   @FieldDocumentation("Specifies the position of the footstep. It is expressed in world frame.")
   public Point3d location;
   @FieldDocumentation("Specifies the orientation of the footstep. It is expressed in world frame.")
   public Quat4d orientation;
   
   public double swingHeight;
   
   @Override
   public boolean epsilonEquals(FootstepDataListMessage other, double epsilon)
   {
      return false;
   }

}
