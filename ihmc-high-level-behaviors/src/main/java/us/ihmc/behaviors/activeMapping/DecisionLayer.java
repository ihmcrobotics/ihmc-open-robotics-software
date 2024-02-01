package us.ihmc.behaviors.activeMapping;

import org.ejml.data.FMatrixRMaj;
import org.ejml.dense.row.CommonOps_FDRM;
import org.ejml.dense.row.NormOps_FDRM;
import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;

public class DecisionLayer
{
   private FMatrixRMaj entropy;
   private float resolution;

   public DecisionLayer(int rows, int cols, float resolution)
   {
      this.resolution = resolution;
      entropy = new FMatrixRMaj(rows, cols);
   }

   public void updateSemanticRewardMap(ArrayList<Point2D> objects)
   {
      Point2D current = new Point2D();
      for (int i = 0; i < entropy.getNumRows(); i++)
      {
         for (int j = 0; j < entropy.getNumCols(); j++)
         {
            current.set(i * resolution, j * resolution);
            float totalDistance = 0;

            for (int k = 0; k < objects.size(); k++)
            {
               Point2D object = objects.get(k);

               float dist = (float) object.distance(current);
               totalDistance += dist;
            }

            entropy.set(i, j, totalDistance);
         }
      }

      float norm = NormOps_FDRM.normF(entropy);

      // Divide each element of the matrix by its norm
      CommonOps_FDRM.divide(entropy, norm);
   }

   public void updateHeightRewardMap()
   {

   }

   public void updateTraversabilityRewardMap()
   {

   }

   public void updateExplorationRewardMap()
   {

   }
}
