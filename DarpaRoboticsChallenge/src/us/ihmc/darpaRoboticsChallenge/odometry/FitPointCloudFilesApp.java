package us.ihmc.darpaRoboticsChallenge.odometry;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import georegression.geometry.RotationMatrixGenerator;
import georegression.metric.UtilAngle;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

/**
 * //head pos(-0.0025047332, 1.3634932, 1.5311656)
 //head rot(0.23656888, 0.22503908, -0.6880978, 0.64800775)
 (-0.15253736719460753,0.6343160329984205,1.9482167060297617)
 *
 * @author Peter Abeles
 */
public class FitPointCloudFilesApp
{

   public static void main(String[] args) throws IOException
   {
      List<Point3D_F64> cloudA = read("/home/pja/Downloads/scan.dat");
      List<Point3D_F64> cloudB = read("/home/pja/Downloads/scan2.dat");

      Se3_F64 refToCurrent = new Se3_F64();
      refToCurrent.getT().set(0.3,0.05,0);
      RotationMatrixGenerator.eulerXYZ(0,0, UtilAngle.degreeToRadian(20),refToCurrent.getR());

      for( Point3D_F64 p : cloudB )
         SePointOps_F64.transform(refToCurrent,p,p);

      IcpCloud3D icp = new IcpCloud3D(0.2,30,1e-8);

      icp.setReference(cloudA);

      if( !icp.setCurrent(cloudB) )
         throw new RuntimeException("ICP failed");


      Se3_F64 found = icp.getReferenceToCurrent();
      System.out.println("Found");
      found.print();

      System.out.println("actual");
      refToCurrent.print();

      System.out.println("Fit Fraction: "+icp.getFitFraction());
   }

   public static List<Point3D_F64> read( String fileName ) throws IOException
   {
      BufferedReader reader = new BufferedReader(new FileReader(fileName));

      reader.readLine();
      reader.readLine();

      List<Point3D_F64> ret = new ArrayList<Point3D_F64>();

      while( true ) {
         String s = reader.readLine();
         if( s == null )
            break;
         String w[] = s.split(" ");
         if( w.length != 3 )
            break;

         Point3D_F64 p = new Point3D_F64();
         p.x = Double.parseDouble(w[0]);
         p.y = Double.parseDouble(w[1]);
         p.z = Double.parseDouble(w[2]);
         ret.add(p);
      }
      return ret;
   }
}
