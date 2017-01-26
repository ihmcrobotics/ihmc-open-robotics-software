package us.ihmc.imageProcessing.segmentation;

import boofcv.struct.image.GrayF32;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Peter Abeles
 */
public class RoadClassification {
   List<Gaussian2D_F64> road = new ArrayList<Gaussian2D_F64>();
   List<Gaussian2D_F64> other = new ArrayList<Gaussian2D_F64>();

   double maxChiSquare = 12.0;

   public void addRoad( Gaussian2D_F64 d ) {
      road.add(d);
   }

   public void addOther( Gaussian2D_F64 d ) {
      other.add(d);
   }

   public void classify( GrayF32 band0 , GrayF32 band1 , GrayF32 output ) {

      System.out.println("total road = "+road.size()+"  other "+other.size());

      for( int y = 0; y < band0.height; y++ ) {
         for( int x = 0; x < band0.width; x++ ) {
            double v0 = band0.unsafe_get(x,y);
            double v1 = band1.unsafe_get(x,y);

            double scoreRoad = selectBest(road,v0,v1);
//            System.out.println("score "+scoreRoad);
            if( scoreRoad > maxChiSquare ) {
               output.unsafe_set(x,y,0);
            } else {
               double scoreOther = selectBest(other,v0,v1);
               if( scoreRoad < scoreOther ) {
                  output.unsafe_set(x,y,1);
               } else {
                  output.unsafe_set(x,y,0);
               }
            }
         }
      }
   }

   private double selectBest( List<Gaussian2D_F64> list , double v0 , double v1 ) {
      double best = Double.MAX_VALUE;

      for( int i = 0; i < list.size(); i++ ) {
         Gaussian2D_F64 d = list.get(i);
         double chisq = d.chisq(v0,v1);
         if( chisq < best )
            best = chisq;
      }

      return best;
   }
}
