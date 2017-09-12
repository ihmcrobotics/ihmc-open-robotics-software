package us.ihmc.utilities.ros.subscriber;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public abstract class  RosImuSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.Imu> 
{
   /*
    * header: 
        seq: 676
        stamp: 
          secs: 1424311090
          nsecs: 671066000
        frame_id: /multisense/accel
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
      orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      angular_velocity: 
        x: -0.00580321995013
        y: -0.00687223413779
        z: 0.00488692192639
      angular_velocity_covariance: [8.79376936e-06, -3.56007627e-07, 2.22611968e-07, -3.56007627e-07, 8.78939245e-06, -8.08367486e-07, 1.33981577e-05, 2.22611968e-07, -8.08367486e-07]
      linear_acceleration: 
        x: -9.45361105359
        y: -0.588399023381
        z: -3.76575363273
      linear_acceleration_covariance: [0.000698179077, 2.46789341e-06, -2.50549745e-06, 2.46789341e-06, 0.000502177646, 5.26265558e-05, -2.50549745e-06, 5.26265558e-05, 0.000922796937]
    */
   private long timeStamp = 0;
   private int seq_id;
   private String frameId;

   private final us.ihmc.euclid.tuple4D.Quaternion orientationEstimate = new us.ihmc.euclid.tuple4D.Quaternion();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D();
   
   public RosImuSubscriber()
   {
      super(sensor_msgs.Imu._TYPE);
   }

   @Override
   public synchronized void onNewMessage(sensor_msgs.Imu message)
   {
      this.timeStamp = message.getHeader().getStamp().totalNsecs();
      this.seq_id = message.getHeader().getSeq();
      this.frameId = message.getHeader().getFrameId();
      RosTools.packRosQuaternionToQuat4d(message.getOrientation(), this.orientationEstimate);
      RosTools.packRosVector3ToVector3d(message.getAngularVelocity(), this.angularVelocity);
      RosTools.packRosVector3ToVector3d(message.getLinearAcceleration(), this.linearAcceleration);
      onNewMessage(timeStamp, seq_id, orientationEstimate, angularVelocity, linearAcceleration);
   }
   
   public String getFrameId()
   {
      return frameId;
   }
   

   protected abstract void onNewMessage(long timeStamp, int seqId, Quaternion orientation, Vector3D angularVelocity, Vector3D linearAcceleration);
   
   public static void main(String[] arg) throws URISyntaxException
   {
      RosMainNode mainNode = new RosMainNode(new URI("http://localhost:11311"), "testImu");
      mainNode.attachSubscriber("/multisense/imu/imu_data", new RosImuSubscriber()
      {
         
         @Override
         protected void onNewMessage(long timeStamp, int seqId, Quaternion orientation, Vector3D angularVelocity, Vector3D linearAcceleration)
         {
            System.out.println("Gravity:"+ linearAcceleration);
         }
      });
      mainNode.execute();
   }

}


