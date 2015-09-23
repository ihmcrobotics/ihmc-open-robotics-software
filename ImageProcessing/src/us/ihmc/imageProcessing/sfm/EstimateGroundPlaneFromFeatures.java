package us.ihmc.imageProcessing.sfm;

import java.util.List;

import org.ddogleg.fitting.modelset.ModelGenerator;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.ransac.Ransac;
import org.ddogleg.struct.FastQueue;
import org.ejml.data.DenseMatrix64F;

import boofcv.alg.geo.MultiViewOps;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.h.HomographyInducedStereo3Pts;
import boofcv.alg.geo.robust.DistanceHomographySq;
import boofcv.struct.calib.StereoParameters;
import boofcv.struct.geo.AssociatedPair;
import georegression.fitting.homography.ModelManagerHomography2D_F64;
import georegression.struct.homography.Homography2D_F64;
import georegression.struct.homography.UtilHomography;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

/**
 * <p>
 * Given a set of associated point feature pairs between two stereo cameras, estimate the ground plane and compute
 * a transform from the left camera to the ground plane.  The ground plane is assumed to be the largest flat plane
 * in the image.
 * </p>
 *
 * <p>
 * The found transform will be from the left camera frame into the ground reference frame.  The origin of the ground
 * reference frame is defined as closest point on the ground to the left camera.  The ground z-axis is the projection
 * of the camera z-axis onto the ground.  Same is true fot the x-axis.  The ground y-axis is then the cross product of
 * the ground z and x axises.
 * </p>
 *
 * @author Peter Abeles
 */
public class EstimateGroundPlaneFromFeatures
{
   StereoParameters stereo;
   // transform from left to right camera
   Se3_F64 l2r;

   // Use RANSAC to estimate the Fundamental matrix
   ModelMatcher<Homography2D_F64,AssociatedPair> robustH;


   SelectBestHomography selectBest;

   FastQueue<AssociatedPair> matches = new FastQueue<AssociatedPair>(AssociatedPair.class,true);

   public EstimateGroundPlaneFromFeatures(StereoParameters stereo)
   {
      this.stereo = stereo;
      l2r = stereo.getRightToLeft().invert(null);

      selectBest = new SelectBestHomography(stereo);

//      stereo.print();

//      Se3_F64 r2l = stereo.getRightToLeft();
      Se3_F64 l2r = stereo.getRightToLeft().invert(null);
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(stereo.getLeft(),null);
      DenseMatrix64F E = MultiViewOps.createEssential(l2r.getR(),l2r.T);
      DenseMatrix64F F = MultiViewOps.createFundamental(E,K);

      // Input will be normalized coordinates, but error will be in pixels
      DistanceHomographySq errorMetric = new DistanceHomographySq();
      ModelManagerHomography2D_F64 manager = new ModelManagerHomography2D_F64();

      robustH = new Ransac<Homography2D_F64, AssociatedPair>(123123,manager,new Foo(F),errorMetric,500,0.2*0.2);
   }

   private static class Foo implements ModelGenerator<Homography2D_F64,AssociatedPair> {

      HomographyInducedStereo3Pts alg;

      public Foo( DenseMatrix64F F ) {
         alg = new HomographyInducedStereo3Pts();
         alg.setFundamental(F,null);
      }

      public Homography2D_F64 createModelInstance()
      {
         return new Homography2D_F64();
      }

      public boolean generate(List<AssociatedPair> pairs, Homography2D_F64 H)
      {
         if( !alg.process(pairs.get(0),pairs.get(1),pairs.get(2)) )
            return false;

         UtilHomography.convert(alg.getHomography(),H);

         return true;
      }

      public int getMinimumPoints()
      {
         return 3;
      }
   }

   public boolean process( List<Point2D_F64> left , List<Point2D_F64> right ) {

      long start = System.currentTimeMillis();

      // convert point into normalized image coordinates
      matches.reset();
      for( int i = 0; i < left.size(); i++ ) {
         Point2D_F64 l = left.get(i);
         Point2D_F64 r = right.get(i);

         matches.grow().assign(l,r);
      }

      System.out.println("Convert pixel to norm: "+(System.currentTimeMillis()-start));

      // Estimate the plane's normal and the distance from the plane
      if( !estimatePlane(matches.toList()) )
         return false;

      System.out.println("Distance from plane: "+selectBest.getDistanceFromPlane());
      selectBest.getNormal().print();


      return true;
   }

   /**
    * Estimate the plane's normal and the distance from the plane.  From the set of matched pairs a homography
    * is estimated.  The homography is then decomposed to find the camera baseline and a description of the plane.
    * The decomposition generates several hypotheses, but only the one which is the best match to the known camera
    * baseline is used.
    */
   private boolean estimatePlane(List<AssociatedPair> matches)
   {
      // RANSAC to find best homography
      if( !robustH.process(matches) )
         return false;

      DenseMatrix64F Hd = UtilHomography.convert(robustH.getModelParameters(), (DenseMatrix64F) null);
      selectBest.process(Hd,true);

      return true;
   }

   public void getInliers( List<Point2D_F64> left , List<Point2D_F64> right ,
                           List<Point2D_F64> inliersLeft , List<Point2D_F64> inliersRight ) {
      int N = robustH.getMatchSet().size();

      for( int i = 0; i < N; i++ ) {
         int index = robustH.getInputIndex(i);
         inliersLeft.add( left.get(index) );
         inliersRight.add( right.get(index) );
      }
   }

   public Se3_F64 getGroundToLeft()
   {
      return selectBest.getGroundToLeft();
   }

   public double getDistanceFromPlane()
   {
      return selectBest.getDistanceFromPlane();
   }

   public Vector3D_F64 getNormal()
   {
      return selectBest.getNormal();
   }
}
