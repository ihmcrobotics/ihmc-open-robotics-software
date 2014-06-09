package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.calib.IntrinsicParameters;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.so.Quaternion_F64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import sensor_msgs.CameraInfo;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.graphics3DAdapter.camera.IntrinsicCameraParametersPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.ros.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.RosImageSubscriber;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTopicPublisher;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.awt.image.BufferedImage;
import java.io.*;

/**
 * Grabs camera images, camera location, and intrinsic parameters and saves them to a log file
 */
// TODO grab camera to world transform
public class CameraLogger
{

   File outputDir;
   String logDirectory = "logCamera";
   long tick = 0;

   PrintStream logPose;

   public CameraLogger( String cameraName )
   {
      // see if the log directory exists, if it doesn't make a new one
      File directory = new File(logDirectory + "/" + cameraName);

      if (!directory.exists())
      {
         if (!directory.mkdirs())
            throw new RuntimeException("Failed to create camera logging directory");
      }

      outputDir = new File(directory, "" + System.currentTimeMillis());
      if (!outputDir.mkdir())
         throw new RuntimeException("Can't create log directory for this session");

      try
      {
         logPose = new PrintStream(new File(outputDir, "pose.txt"));
         logPose.println("# (Time Step) [Camera to world (quaternion x) (quaternion y) (quaternion z) (quaternion w) (X) (Y) (Z)]");
      } catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void log( BufferedImage image , long timeStamp )
   {
      DenseMatrix64F R = CommonOps.identity(3, 3);
      Vector3d T = new Vector3d();
//            Matrix3d Rm = new Matrix3d();
//            rosTransformFromHeadBaseToCamera.get(Rm);
//            MatrixTools.matrix3DToDenseMatrix(Rm,R,0,0);
      Quaternion_F64 quad = RotationMatrixGenerator.matrixToQuaternion(R, null);
//            rosTransformFromHeadBaseToCamera.get(T);

      logPose.printf("%d %15f %15f %15f %15f %15f %15f %15f\n", timeStamp, quad.x, quad.y, quad.z, quad.w, T.x, T.y, T.z);
      logPose.flush();

      UtilImageIO.saveImage(image, String.format("%s/image%06d.png", outputDir.getAbsolutePath(), tick));
      tick++;
   }

   public void log( IntrinsicParameters parameters )
   {
      BoofMiscOps.saveXML(parameters,outputDir.getAbsolutePath()+"/intrinsic.xml");
   }
}
