package us.ihmc.imageProcessing.segmentation;

import georegression.metric.UtilAngle;

/**
 * @author Peter Abeles
 */
public class ColorDistance {
   public static double distanceHsv( Gaussian2D_F64 model , double hue , double saturation ) {
      double dx = UtilAngle.dist(hue,model.x);
      double dy = saturation-model.y;

      // chi-square = d^T*S*d
      double tmp0 = dx*model.sxx + dy*model.sxy;
      double tmp1 = dx*model.sxy + dy*model.syy;

      return tmp0*dx + tmp1*dy;
   }
}
