package us.ihmc.utilities.ros.publisher;

import org.ros.message.Time;

import boofcv.struct.calib.IntrinsicParameters;
import sensor_msgs.CameraInfo;
import std_msgs.Header;

public class RosCameraInfoPublisher extends RosTopicPublisher<CameraInfo>
{
   private int seq = 0;

   private double[] D = new double[5];
   private double[] K = new double[9];
   private double[] R = new double[9];
   private double[] P = new double[12];

   public RosCameraInfoPublisher()
   {
       super(CameraInfo._TYPE, false);
   }

   @Override
   public void connected()
   {
   }

   public void publish(String frameID, IntrinsicParameters params, Time t)
   {
        CameraInfo message = getMessage();
        Header header = message.getHeader();

        header.setStamp(t);
        header.setFrameId(frameID);
        header.setSeq(seq++);

        //double f = videoSettings.getWidth()/2.0 / Math.tan(videoSettings.getFieldOfView()/2.0);
        //leftParamPacket = new IntrinsicCameraParametersPacket(f, f, 0, (videoSettings.getWidth()-1)/2f,(videoSettings.getHeight()-1)/2f;

        double Tx = 0.0;
        double Ty = 0.0;

        R[0] = 0.0;
        R[1] = 1.0;
        R[2] = 0.0;
        R[3] = 0.0;
        R[4] = 0.0;
        R[5] = 1.0;
        R[6] = 1.0;
        R[7] = 0.0;
        R[8] = 0.0;

/*
       R[0] = 1.0;
       R[1] = 0.0;
       R[2] = 0.0;
       R[3] = 0.0;
       R[4] = 1.0;
       R[5] = 0.0;
       R[6] = 0.0;
       R[7] = 0.0;
       R[8] = 1.0;
*/

        K[0] = params.getFx();
        K[1] = params.getSkew();
        K[2] = params.getCx();
        K[3] = 0.0;
        K[4] = params.getFy();
        K[5] = params.getCy();
        K[6] = 0.0;
        K[7] = 0.0;
        K[8] = 1.0;

        P[0] = params.getFx();
        P[1] = params.getSkew();
        P[2] = params.getCx();
        P[3] = Tx;
        P[4] = 0.0;
        P[5] = params.getFy();
        P[6] = params.getCy();
        P[7] = Ty;
        P[8] = 0.0;
        P[9] = 0.0;
        P[10] = 1.0;
        P[11] = 0.0;

        D[0] =  0.0;
        D[1] =  0.0;
        D[2] =  0.0;
        D[3] =  0.0;
        D[4] =  0.0;

        message.setWidth(params.getWidth());
        message.setHeight(params.getHeight());
        message.setBinningX(0);
        message.setBinningY(0);
        message.setDistortionModel("plumb_bob");
        message.setD(D);
        message.setK(K);
        message.setP(P);
        message.setR(R);

        publish(message);
   }
}
