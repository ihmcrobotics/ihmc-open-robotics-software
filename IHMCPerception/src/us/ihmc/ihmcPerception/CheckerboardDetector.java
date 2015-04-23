package us.ihmc.ihmcPerception;

import georegression.struct.se.Se3_F64;

import java.awt.image.BufferedImage;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.fiducial.FiducialDetector;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.ImageFloat32;

public class CheckerboardDetector
{
   private final FiducialDetector<ImageFloat32> detector;
   private boolean intrinsicSet = false;
   
   public CheckerboardDetector(int gridCollumns, int gridRows, double gridSize)
   {
      detector = FactoryFiducial.calibChessboard(new ConfigChessboard(gridCollumns, gridRows), gridSize, ImageFloat32.class);
   }
   
   public void setIntrinsicParameters(IntrinsicParameters intrinsicParameters)
   {
      detector.setIntrinsic(intrinsicParameters);
      intrinsicSet = true;
   }
   
   private final ImageFloat32 gray = new ImageFloat32(1, 1);
   private final Se3_F64 targetToSensor = new Se3_F64();
   private final Matrix3d rotation = new Matrix3d();
   private final Vector3d translation = new Vector3d();
   
   public RigidBodyTransform detectCheckerboard(BufferedImage image)
   {
      if (!intrinsicSet)
      {
         PrintTools.info("Intrinsic parameters not set");
         return null;
      }
      
      gray.reshape(image.getWidth(), image.getHeight());
      ConvertBufferedImage.convertFrom(image, gray);
      
      detector.detect(gray);
      int checkerboards = detector.totalFound();
      if (checkerboards != 1)
      {
         return null;
      }
      detector.getFiducialToWorld(0, targetToSensor);
      rotation.set(targetToSensor.getRotation().getData());
      translation.set(targetToSensor.getTranslation().getX(), targetToSensor.getTranslation().getY(), targetToSensor.getTranslation().getZ());
      
      return new RigidBodyTransform(rotation, translation);
   }
}
