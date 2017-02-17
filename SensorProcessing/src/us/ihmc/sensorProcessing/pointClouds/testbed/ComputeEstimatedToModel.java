package us.ihmc.sensorProcessing.pointClouds.testbed;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.thoughtworks.xstream.XStream;

import georegression.struct.se.Se3_F64;

/**
 * Processes transforms found be the automated algorithm and manually aligned.  It selects the transform which minimizes
 * the error across the set as the official transform from estimated to model.
 *
 * @author Peter Abeles
 */
public class ComputeEstimatedToModel {

   public static void main(String[] args) {
      String directory = "../SensorProcessing/data/testbed/2014-08-01/";


      int N = 16;

      List<Se3_F64> listEstimated = new ArrayList<>();
      List<Se3_F64> listModel = new ArrayList<>();
      List<Se3_F64> listEstimatedToModel = new ArrayList<>();

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

         listEstimated.add(estimatedToWorld);
         listModel.add(modelToWorld);

         Se3_F64 estimatedToModel = estimatedToWorld.concat(modelToWorld.invert(null),null);
         listEstimatedToModel.add( estimatedToModel );

      }

      double bestError = Double.MAX_VALUE;
      Se3_F64 best = null;
      double errors[] = new double[N];
      for (int i = 0; i < N; i++) {
         Se3_F64 estimatedToModel = listEstimatedToModel.get(i);

         for (int j = 0; j < N; j++) {
            Se3_F64 estimatedToWorld = listEstimated.get(j);
            Se3_F64 modelToWorld = listModel.get(j);
            Se3_F64 found = estimatedToWorld.concat(modelToWorld.invert(null),null);

            errors[j] = found.getT().distance(estimatedToModel.getT());
         }

         Arrays.sort(errors);

         double error50 = errors[ N/2 ];
         System.out.println(i+" Error  = "+error50);
         if( error50 < bestError ) {
            bestError = error50;
            best = estimatedToModel;
         }
      }

      System.out.println("Best.  Translation error 50% = "+ bestError);

      try {
         new XStream().toXML(best, new FileOutputStream("estimatedToModel.xml"));
      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }
   }
}
