package us.ihmc.utilities.ros.subscriber;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;

public abstract class RosPoseStampedSubscriber extends AbstractRosTopicSubscriber<geometry_msgs.PoseStamped>
{
   private long timeStamp;
   private String frameID;
   private Vector3D pos;
   private Quaternion rot;
      
   public RosPoseStampedSubscriber()
   {
      super(geometry_msgs.PoseStamped._TYPE);    
   }

   public synchronized void onNewMessage(geometry_msgs.PoseStamped received)
   {
      // get timestamp
      timeStamp = received.getHeader().getStamp().totalNsecs();
      
      // get Point
      Double posx = received.getPose().getPosition().getX();
      Double posy = received.getPose().getPosition().getY();
      Double posz = received.getPose().getPosition().getZ();
      
      pos = new Vector3D(posx, posy, posz);
      
      // get Rotation
      Double rotx = received.getPose().getOrientation().getX();
      Double roty = received.getPose().getOrientation().getY();
      Double rotz = received.getPose().getOrientation().getZ();
      Double rotw = received.getPose().getOrientation().getW();
      
      rot = new Quaternion(rotx, roty, rotz, rotw);
      
      // get frameID
      frameID = received.getHeader().getFrameId();

      TimeStampedTransform3D transform = new TimeStampedTransform3D();
      transform.getTransform3D().setTranslation(pos);
      transform.getTransform3D().setRotation(rot);
      transform.setTimeStamp(timeStamp);
      
      newPose(frameID, transform);
   }
   
   public synchronized Vector3D getPoint()
   {
      return pos;     
   }
   
   public synchronized Quaternion getRotation()
   {
      return rot;
   }
   
   public synchronized long getTimestamp()
   {
      return timeStamp;
   }
   
   public synchronized String getFrameID()
   {
      return frameID;
   }
   
   protected abstract void newPose(String frameID, TimeStampedTransform3D transform);
   
}
