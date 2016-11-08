package us.ihmc.sensorProcessing.pointClouds.testbed;

import com.thoughtworks.xstream.XStream;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Rodrigues_F64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.Arrays;

/**
 * @author Peter Abeles
 */
public class EvaluateAgainstManualAlignment {
   public static void main(String[] args) {
      String directory = "../SensorProcessing/data/testbed/2014-08-01/";

      int N = 16;

      Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(directory.getClass().
              getResourceAsStream("/testbed/estimatedToModel.xml"));

      double errorsT[] = new double[N];
      double errorsR[] = new double[N];

      DenseMatrix64F R = new DenseMatrix64F(3,3);
      Rodrigues_F64 rod = new Rodrigues_F64();
      for (int i = 0; i < N; i++) {
         String nameEstimated = String.format(directory + "estimatedTestbedToWorld%02d.xml", i);
         String nameModel = String.format(directory + "modelTestbedToWorld%02d.xml", i);


         Se3_F64 estimatedToWorld, modelToWorld;
         try {
            estimatedToWorld = (Se3_F64) new XStream().fromXML(new FileInputStream(nameEstimated));
            modelToWorld = (Se3_F64) new XStream().fromXML(new FileInputStream(nameModel));
         } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
         }

         Se3_F64 found = estimatedToWorld.concat(modelToWorld.invert(null),null);

         errorsT[i] = found.getT().distance(estimatedToModel.getT());



         CommonOps.multTransA(estimatedToModel.getR(), found.getR(), R);
         ConvertRotation3D_F64.matrixToRodrigues(R, rod);
         errorsR[i] = rod.theta;

         System.out.println("errorT["+i+"] = "+errorsT[i]+"  errorR["+i+"] = "+errorsR[i]);
      }

      Arrays.sort(errorsT);
      double error50 = errorsT[ N/2 ];
      double error95 = errorsT[ (int)(N*0.95) ];
      System.out.println("Error Tran 50% = "+error50);
      System.out.println("Error Tran 95% = "+error95);

      Arrays.sort(errorsR);
      error50 = errorsR[ N/2 ];
      error95 = errorsR[ (int)(N*0.95) ];
      System.out.println("Error Rot 50% = "+error50);
      System.out.println("Error Rot 95% = "+error95);
   }
}
