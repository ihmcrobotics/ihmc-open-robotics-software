package us.ihmc.perception.ros1.camera;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import boofcv.io.UtilIO;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.calib.CameraPinholeBrown;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.so.Quaternion_F64;
import us.ihmc.euclid.tuple3D.Vector3D;

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
      DMatrixRMaj R = CommonOps_DDRM.identity(3, 3);
      Vector3D T = new Vector3D();
//            Matrix3d Rm = new Matrix3d();
//            rosTransformFromHeadBaseToCamera.get(Rm);
//            MatrixTools.matrix3DToDenseMatrix(Rm,R,0,0);
      Quaternion_F64 quad = ConvertRotation3D_F64.matrixToQuaternion(R, null);
//            rosTransformFromHeadBaseToCamera.get(T);

      logPose.printf("%d %15f %15f %15f %15f %15f %15f %15f\n", timeStamp, quad.x, quad.y, quad.z, quad.w, T.getX(), T.getY(), T.getZ());
      logPose.flush();

      UtilImageIO.saveImage(image, String.format("%s/image%06d.png", outputDir.getAbsolutePath(), tick));
      tick++;
   }

   public void log( CameraPinholeBrown parameters )
   {
      UtilIO.save(parameters, outputDir.getAbsolutePath() + "/intrinsic.xml");
   }
}
