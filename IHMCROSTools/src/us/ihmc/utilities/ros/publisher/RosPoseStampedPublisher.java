package us.ihmc.utilities.ros.publisher;

import geometry_msgs.PoseStamped;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import std_msgs.Header;


public class RosPoseStampedPublisher extends RosTopicPublisher<PoseStamped>
{
   private final PoseStamped initialValue;
   private int seq = 0;

   public RosPoseStampedPublisher(boolean latched)
   {
      this(latched, null);
   }

   public RosPoseStampedPublisher(boolean latched, PoseStamped initialValue)
   {
      super(PoseStamped._TYPE, latched);
      this.initialValue = initialValue;
   }
   
   @Override
   public void connected()
   {
      if(initialValue != null)
      {
         publish(initialValue);
      }
   }

   public void publish(String frameID, Vector3d pos, Quat4d rot, Time t)
   {
      PoseStamped message = getMessage();

      Header header = message.getHeader();
      header.setStamp(t);
      header.setFrameId(frameID);
      header.setSeq(seq++);
      message.setHeader(header);

      if (pos != null) {
          message.getPose().getPosition().setX(pos.getX());
          message.getPose().getPosition().setY(pos.getY());
          message.getPose().getPosition().setZ(pos.getZ());
      } else {
          message.getPose().getPosition().setX(0.0);
          message.getPose().getPosition().setY(0.0);
          message.getPose().getPosition().setZ(0.0);
      }

      if (rot != null) {
          message.getPose().getOrientation().setX(rot.getX());
          message.getPose().getOrientation().setY(rot.getY());
          message.getPose().getOrientation().setZ(rot.getZ());
          message.getPose().getOrientation().setW(rot.getW());
      } else {
          message.getPose().getOrientation().setX(0.0);
          message.getPose().getOrientation().setY(0.0);
          message.getPose().getOrientation().setZ(0.0);
          message.getPose().getOrientation().setW(1.0);
      }

      publish(message);
   }
}
