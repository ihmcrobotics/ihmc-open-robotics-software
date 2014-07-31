package us.ihmc.sensorProcessing.pointClouds.testbed;

import com.thoughtworks.xstream.XStream;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Rodrigues_F64;
import us.ihmc.sensorProcessing.pointClouds.GeometryOps;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;

/**
 * @author Peter Abeles
 */
public class ComputeEstimatedToModel {

   public static void main(String[] args) {
      String directory = "../SensorProcessing/data/testbed/2014-07-10/";

      double aveX=0,aveY=0,aveZ=0;

      Rodrigues_F64 aveR = new Rodrigues_F64();

      int N = 4;

      for (int i = 0; i < N; i++) {
         String nameEstimated = String.format(directory+"estimatedTestbedToWorld%02d.xml",(i+1));
         String nameModel = String.format(directory+"modelTestbedToWorld%02d.xml",(i+1));

         Se3_F64 estimatedToWorld,modelToWorld;
         try {
            estimatedToWorld = (Se3_F64)new XStream().fromXML(new FileInputStream(nameEstimated));
            modelToWorld = (Se3_F64)new XStream().fromXML(new FileInputStream(nameModel));
         } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
         }

         Se3_F64 estimatedToModel = estimatedToWorld.concat(modelToWorld.invert(null),null);
         System.out.println(estimatedToModel);

         Rodrigues_F64 r = RotationMatrixGenerator.matrixToRodrigues(estimatedToModel.R,(Rodrigues_F64)null);

         aveX += estimatedToModel.T.x;
         aveY += estimatedToModel.T.y;
         aveZ += estimatedToModel.T.z;

         aveR.theta += r.theta;
         aveR.unitAxisRotation.x += r.unitAxisRotation.x;
         aveR.unitAxisRotation.y += r.unitAxisRotation.y;
         aveR.unitAxisRotation.z += r.unitAxisRotation.z;

      }

      aveX /= N;
      aveY /= N;
      aveZ /= N;

      aveR.theta /= N;
      aveR.unitAxisRotation.x /= N;
      aveR.unitAxisRotation.y /= N;
      aveR.unitAxisRotation.z /= N;

      aveR.unitAxisRotation.normalize();

      Se3_F64 estimatedToModel = new Se3_F64();
      estimatedToModel.T.set(aveX,aveY,aveZ);
      RotationMatrixGenerator.rodriguesToMatrix(aveR,estimatedToModel.getR());

      System.out.println("------------------- OUTPUT -----------------------");
      System.out.println(estimatedToModel);

      try {
         new XStream().toXML(estimatedToModel, new FileOutputStream("estimatedToModel.xml"));
      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }
   }
}
