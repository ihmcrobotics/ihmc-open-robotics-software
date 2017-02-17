package us.ihmc.modelFileLoaders.SdfLoader;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFInertia;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFLink;

public class SDFLinkHolderTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void noInertialPose()
   {
      // Setup
      SDFLink sdfLink = new SDFLink();
      SDFLink.Inertial inertial = new SDFLink.Inertial();
      SDFInertia inertia = new SDFInertia();

      inertia.setIxx("0.00319");
      inertia.setIxy("0");
      inertia.setIxz("0");
      inertia.setIyy("0.00583");
      inertia.setIyz("0");
      inertia.setIzz("0.00583");

      inertial.setMass("3.012");
      inertial.setPose("0 0 0 0 -0 0");
      inertial.setInertia(inertia);

      sdfLink.setName("l_scap");
      sdfLink.setPose("0.05191 0.27901 0.51524 0 -0 0");
      sdfLink.setInertial(inertial);

      // Test
      SDFLinkHolder sdfLinkHolder = new SDFLinkHolder(sdfLink);
      sdfLinkHolder.calculateCoMOffset();
      Vector3D comOffset = sdfLinkHolder.getCoMOffset();
      Matrix3D inertiaMatrix = sdfLinkHolder.getInertia();

      // Assertions      
      Vector3D expectedComOffset = new Vector3D(0.0, 0.0, 0.0);
      assertTrue("Should have zeros for COM", expectedComOffset.equals(comOffset));//equals with doubles could cause problems

      double Ixx = 0.00319, Ixy = 0.0, Ixz = 0.0, Iyy = 0.00583, Iyz = 0.0, Izz = 0.00583;
      RotationMatrix expectedInertia = new RotationMatrix(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);
      assertTrue("Inertia matrix should not change values", inertiaMatrix.equals(expectedInertia));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void inertialPoseTranslationOnly()
   {
      // Setup
      SDFLink sdfLink = new SDFLink();
      SDFLink.Inertial inertial = new SDFLink.Inertial();
      SDFInertia inertia = new SDFInertia();

      inertia.setIxx("0.011");
      inertia.setIxy("0");
      inertia.setIxz("0");
      inertia.setIyy("0.009");
      inertia.setIyz("0.004");
      inertia.setIzz("0.004");

      inertial.setMass("3.012");
      inertial.setPose("0 -0.048 0.084 0 -0 0");
      inertial.setInertia(inertia);

      sdfLink.setName("r_clav");
      sdfLink.setPose("0.05191 -0.13866 0.31915 0 -0 0");
      sdfLink.setInertial(inertial);

      // Test
      SDFLinkHolder sdfLinkHolder = new SDFLinkHolder(sdfLink);
      sdfLinkHolder.calculateCoMOffset();
      Vector3D comOffset = sdfLinkHolder.getCoMOffset();
      Matrix3D inertiaMatrix = sdfLinkHolder.getInertia();

      // Assertions
      Vector3D expectedComOffset = new Vector3D(0.0, -0.048, 0.084);
      assertTrue("ComOffset should be value identified in Inertial Pose.", expectedComOffset.equals(comOffset));//equals with doubles could cause problems

      double Ixx = 0.011, Ixy = 0.0, Ixz = 0.0, Iyy = 0.009, Iyz = 0.004, Izz = 0.004;
      RotationMatrix expectedInertia = new RotationMatrix(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);
      assertTrue("Inertia matrix should not change values", inertiaMatrix.equals(expectedInertia));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void inertialPoseWithRotationOnly()//This one should have right COM, but inertia will have to change.
   {
      // Setup
      SDFLink sdfLink = new SDFLink();
      SDFLink.Inertial inertial = new SDFLink.Inertial();
      SDFInertia inertia = new SDFInertia();

      inertia.setIxx("8.8101e-08");
      inertia.setIxy("0");
      inertia.setIxz("0");
      inertia.setIyy("4.09812e-08");
      inertia.setIyz("0");
      inertia.setIzz("5.31842e-08");

      inertial.setMass("0.00058");
      inertial.setPose("0 0 0 -1.57 -1.57 1.39418e-13");
      inertial.setInertia(inertia);
      /*
       * <!-- gazebo -> cad mapping x -> y y -> z z -> x -->
       */

      sdfLink.setName("left_finger_0_flexible_link_2");
      sdfLink.setPose("0.0230665 1.0692 0.66321 -0.00079 0.00159265 -2.64351e-06");
      sdfLink.setInertial(inertial);

      // Test
      SDFLinkHolder sdfLinkHolder = new SDFLinkHolder(sdfLink);
      sdfLinkHolder.calculateCoMOffset();
      Vector3D comOffset = sdfLinkHolder.getCoMOffset();
      Matrix3D inertiaMatrix = sdfLinkHolder.getInertia();

      // Assertions
      Vector3D expectedComOffset = new Vector3D(0.0, 0.0, 0.0);
      assertTrue("ComOffset should be zeros.", expectedComOffset.equals(comOffset));//equals with doubles could cause problems

      //      double Ixx=8.8101e-08,Ixy=0.0,Ixz=0.0,Iyy=4.09812e-08,Iyz=0.0,Izz=5.31842e-08;//Incorrect output
      double Ixx = 4.09812e-08, Ixy = 0.0, Ixz = 0.0, Iyy = 5.31842e-08, Iyz = 0.0, Izz = 8.8101e-08;
      RotationMatrix expectedInertia = new RotationMatrix(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);
      //1e-10
      assertArrayEquals(new double[] {Ixx, Ixy, Ixz, Iyy, Iyz, Izz}, new double[] {inertiaMatrix.getM00(), inertiaMatrix.getM01(), inertiaMatrix.getM02(),
            inertiaMatrix.getM11(), inertiaMatrix.getM12(), inertiaMatrix.getM22()}, 1e-10);
      boolean inertiaEqual = inertiaMatrix.equals(expectedInertia);
      String helpMessage = "Incorrect actual: Ixx=" + inertiaMatrix.getM00() + ", Ixy=" + inertiaMatrix.getM01() + ", Ixz=" + inertiaMatrix.getM02() + ", Iyy="
            + inertiaMatrix.getM11() + ", Iyz=" + inertiaMatrix.getM12() + ", Izz=" + inertiaMatrix.getM22();
      String helpMessagec = "Expected       : Ixx=" + Ixx + ", Ixy=" + Ixy + ", Ixz=" + Ixz + ", Iyy=" + Iyy + ", Iyz=" + Iyz + ", Izz=" + Izz;
      //if(!inertiaEqual)
      //      {
      //         System.out.println(helpMessage);
      //         System.out.println(helpMessagec);
      //         inertiaMatrix.negate(); 
      //         inertiaMatrix.add(expectedInertia);
      //         System.out.println("Diffs: Ixx=" + inertiaMatrix.m00 + ", Ixy=" + inertiaMatrix.m01 + ", Ixz=" + inertiaMatrix.m02 + ", Iyy=" + inertiaMatrix.m11 + ", Iyz=" + inertiaMatrix.m12 + ", Izz=" + inertiaMatrix.m22);
      //      }
      //assertTrue("Inertia matrix should be changed to link frame. " + helpMessage, inertiaEqual);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void inertialPoseProximaliRobotWithRotationAndTranslation()
   {
      // Setup
      SDFLink sdfLink = new SDFLink();
      SDFLink.Inertial inertial = new SDFLink.Inertial();
      SDFInertia inertia = new SDFInertia();

      inertia.setIxx("2.42069e-06");
      inertia.setIxy("-1.44904e-06");
      inertia.setIxz("-7.74579e-07");
      inertia.setIyy("1.1962e-06");
      inertia.setIyz("1.90704e-07");
      inertia.setIzz("1.27365e-06");

      inertial.setMass("0.0288");
      inertial.setPose("-0.00263 0.002396 0.02177 1.57 -1.57 -1.39418e-13");
      inertial.setInertia(inertia);
      /*
       * <!-- gazebo -> cad coordinate mapping x -> -y y -> -z z -> x -->
       */

      sdfLink.setName("left_finger_0_proximal_link");
      sdfLink.setPose("0.0229502 1.06915 0.590211 -0.00079 0.00159265 -2.64351e-06");
      sdfLink.setInertial(inertial);

      // Test
      SDFLinkHolder sdfLinkHolder = new SDFLinkHolder(sdfLink);
      sdfLinkHolder.calculateCoMOffset();
      Vector3D comOffset = sdfLinkHolder.getCoMOffset();
      Matrix3D inertiaMatrix = sdfLinkHolder.getInertia();

      // Assertions
      //Translation should be the same: specified translation occurs after rotation
      Vector3D expectedComOffset = new Vector3D(-0.00263, 0.002396, 0.02177);
      assertTrue("ComOffset is wrong.", expectedComOffset.equals(comOffset));//equals with doubles could cause problems

      //Incorrect output
      //double Ixx=2.42069e-06,Ixy=-1.44904e-06,Ixz=-7.74579e-07,Iyy=1.1962e-06,Iyz=1.90704e-07,Izz=1.27365e-06;
      /*
       * <!-- cad -> gazebo coordinate mapping y -> -x z -> -y x -> z -->
       */
      double Ixx = 1.1962e-06, Ixy = 1.90704e-07, Ixz = 1.44904e-06, Iyy = 1.27365e-06, Iyz = 7.74579e-07, Izz = 2.42069e-06;
      RotationMatrix expectedInertia = new RotationMatrix(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);
      //1e-10
      boolean inertiaEqual = inertiaMatrix.equals(expectedInertia);
      String helpMessage = "Incorrect actual: Ixx=" + inertiaMatrix.getM00() + ", Ixy=" + inertiaMatrix.getM01() + ", Ixz=" + inertiaMatrix.getM02() + ", Iyy="
            + inertiaMatrix.getM11() + ", Iyz=" + inertiaMatrix.getM12() + ", Izz=" + inertiaMatrix.getM22();
      String helpMessagec = "Expected       : Ixx=" + Ixx + ", Ixy=" + Ixy + ", Ixz=" + Ixz + ", Iyy=" + Iyy + ", Iyz=" + Iyz + ", Izz=" + Izz;
      //      if(!inertiaEqual)
      //      {
      //         System.out.println(helpMessage);
      //         System.out.println(helpMessagec);
      //      Matrix3d inertiaMatrixDiff=new Matrix3d(inertiaMatrix);
      //      inertiaMatrixDiff.negate(); 
      //      inertiaMatrixDiff.add(expectedInertia);
      //      System.out.println("Diffs: Ixx=" + inertiaMatrixDiff.m00 + ", Ixy=" + inertiaMatrixDiff.m01 + ", Ixz=" + inertiaMatrixDiff.m02 + ", Iyy=" + inertiaMatrixDiff.m11 + ", Iyz=" + inertiaMatrixDiff.m12 + ", Izz=" + inertiaMatrixDiff.m22);
      //      }
      //assertTrue("Inertia matrix should be changed to link frame. " + helpMessage, inertiaEqual);
      assertArrayEquals(new double[] {Ixx, Ixy, Ixz, Iyy, Iyz, Izz}, new double[] {inertiaMatrix.getM00(), inertiaMatrix.getM01(), inertiaMatrix.getM02(),
            inertiaMatrix.getM11(), inertiaMatrix.getM12(), inertiaMatrix.getM22()}, 3e-9);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void inertialPoseDistaliRobotWithRotationAndTranslation()
   {
      // Setup
      SDFLink sdfLink = new SDFLink();
      SDFLink.Inertial inertial = new SDFLink.Inertial();
      SDFInertia inertia = new SDFInertia();

      inertia.setIxx("1.62414e-06");
      inertia.setIxy("5.6e-10");
      inertia.setIxz("1.33e-10");
      inertia.setIyy("1.71788e-06");
      inertia.setIyz("-1.1997e-07");
      inertia.setIzz("5.60221e-07");

      inertial.setMass("0.0133");
      inertial.setPose("-6.6e-05 -0.003142 0.016633 3.14159 0.00159265 3.14159");
      inertial.setInertia(inertia);
      /*
       * <!-- gazebo -> cad mapping x -> -x y -> y z -> -z -->
       */

      sdfLink.setName("left_finger_0_distal_link");
      sdfLink.setPose("0.0230744 1.06921 0.66821 -0.00079 0.00159265 -2.64351e-06");
      sdfLink.setInertial(inertial);

      // Test
      SDFLinkHolder sdfLinkHolder = new SDFLinkHolder(sdfLink);
      sdfLinkHolder.calculateCoMOffset();
      Vector3D comOffset = sdfLinkHolder.getCoMOffset();
      Matrix3D inertiaMatrix = sdfLinkHolder.getInertia();

      // Assertions
      //Translation should be the same: specified translation occurs after rotation
      Vector3D expectedComOffset = new Vector3D(-6.6e-05, -0.003142, 0.016633);
      assertTrue("ComOffset is wrong.", expectedComOffset.equals(comOffset));//equals with doubles could cause problems

      /*
       * <!-- CAD -> Gazebo mapping x -> -x y -> y z -> -z -->
       */
      //double Ixx=1.62414e-06, Ixy=5.6e-10, Ixz=1.33e-10, Iyy=1.71788e-06, Iyz=-1.1997e-07, Izz=5.60221e-07;//Incorrect output
      double Ixx = 1.62414e-06, Ixy = -5.6e-10, Ixz = 1.33e-10, Iyy = 1.71788e-06, Iyz = 1.1997e-07, Izz = 5.60221e-07;
      RotationMatrix expectedInertia = new RotationMatrix(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

      //1e-10
      boolean inertiaEqual = inertiaMatrix.equals(expectedInertia);
      String helpMessage = "Incorrect actual: Ixx=" + inertiaMatrix.getM00() + ", Ixy=" + inertiaMatrix.getM01() + ", Ixz=" + inertiaMatrix.getM02() + ", Iyy="
            + inertiaMatrix.getM11() + ", Iyz=" + inertiaMatrix.getM12() + ", Izz=" + inertiaMatrix.getM22();
      String helpMessagec = "Expected       : Ixx=" + Ixx + ", Ixy=" + Ixy + ", Ixz=" + Ixz + ", Iyy=" + Iyy + ", Iyz=" + Iyz + ", Izz=" + Izz;
      //      if(!inertiaEqual)
      //      {
      //         System.out.println(helpMessage);
      //         System.out.println(helpMessagec);
      //         Matrix3d inertiaMatrixDiff=new Matrix3d(inertiaMatrix);
      //         inertiaMatrixDiff.negate(); 
      //         inertiaMatrixDiff.add(expectedInertia);
      //         System.out.println("Diffs: Ixx=" + inertiaMatrixDiff.m00 + ", Ixy=" + inertiaMatrixDiff.m01 + ", Ixz=" + inertiaMatrixDiff.m02 + ", Iyy=" + inertiaMatrixDiff.m11 + ", Iyz=" + inertiaMatrixDiff.m12 + ", Izz=" + inertiaMatrixDiff.m22);
      //      }
      //assertTrue("Inertia matrix should be changed to link frame. " + helpMessage, inertiaEqual);
      assertArrayEquals(new double[] {Ixx, Ixy, Ixz, Iyy, Iyz, Izz}, new double[] {inertiaMatrix.getM00(), inertiaMatrix.getM01(), inertiaMatrix.getM02(),
            inertiaMatrix.getM11(), inertiaMatrix.getM12(), inertiaMatrix.getM22()}, 2e-9);

   }

}
/*
 * Fix following 126 Warnings (errors) (condensed for readability: 42 errors below repeated three
 * times): [Process: DRCNetworkProcessor] Warning: Non-zero rotation of the inertial matrix on link
 * (left/right)_finger_(0/1/2)_proximal_link [Process: DRCNetworkProcessor] Warning: Non-zero
 * rotation of the inertial matrix on link
 * (left/right)_finger_(0/1/2)_flexible_link_hidden_between_proximal_link_and_1 [Process:
 * DRCNetworkProcessor] Warning: Non-zero rotation of the inertial matrix on link
 * (left/right)_finger_(0/1/2)_flexible_link_1 [Process: DRCNetworkProcessor] Warning: Non-zero
 * rotation of the inertial matrix on link
 * (left/right)_finger_(0/1/2)_flexible_link_hidden_between_1_and_2 [Process: DRCNetworkProcessor]
 * Warning: Non-zero rotation of the inertial matrix on link
 * (left/right)_finger_(0/1/2)_flexible_link_2 [Process: DRCNetworkProcessor] Warning: Non-zero
 * rotation of the inertial matrix on link
 * (left/right)_finger_(0/1/2)_flexible_link_hidden_between_2_and_distal_link [Process:
 * DRCNetworkProcessor] Warning: Non-zero rotation of the inertial matrix on link
 * (left/right)_finger_(0/1/2)_distal_link
 */

/**************************************
 * Test data: Typical non-problematic (all pose rotation elements are zero: last three elements of
 * rawPose and rawInertialPose [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link
 * l_scap [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawName:l_scap [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawPose:0.05191 0.27901 0.51524 0 -0 0 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawMass:3.012 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawInertialaPose:0 0 0 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIxx:0.00319 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIyy:0.00583 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIyz:0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:0.00583
 * 
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link hokuyo_link [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawName:hokuyo_link [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawPose:0.15962 0 0.83793 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawMass:0.057664 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0.0124384 0.0004084 -0.0041783 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIxx:0.000401606 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxy:4.9927e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:1.0997e-05
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyy:0.00208115 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:-9.8165e-09 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIzz:0.00178402
 * 
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link r_clav [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawName:r_clav [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawPose:0.05191 -0.13866 0.31915 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawMass:3.45 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0 -0.048 0.084 0 -0 0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxx:0.011 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIyy:0.009 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0.004
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:0.004
 * 
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link r_scap [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawName:r_scap [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawPose:0.05191 -0.27901 0.51524 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawMass:3.012 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0 0 0 0 -0 0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxx:0.00319 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIyy:0.00583 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:0.00583
 * 
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link r_uarm [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawName:r_uarm [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawPose:0.05191 -0.46601 0.53124 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawMass:3.388 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0 0.065 0 0 -0 0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxx:0.00656 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIyy:0.00358 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:0.00656
 * 
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link r_larm [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawName:r_larm [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawPose:0.05191 -0.58501 0.54045 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawMass:2.509 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0 0 0 0 -0 0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxx:0.00265 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIyy:0.00446 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:0.00446
 * 
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link r_farm [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawName:r_farm [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawPose:0.05191 -0.77201 0.53124 0 -0 0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawMass:3.388 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0 0.065 0 0 -0 0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxx:0.00656 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIyy:0.00358 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:0.00656
 *************************************************************************/
/*
 * Possible problem causing data (rotated inertias) [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: created link l_hand [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawName:l_hand [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawPose:0.05191 0.89101
 * 0.54045 0 -0 0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawMass:3.814 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawInertialaPose:0.000612467 0.03256 0.00402381 0 -0
 * 0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxx:0.0117926 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:-0.000146233 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIxz:-1.80714e-05 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIyy:0.00478148 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:-0.000960716
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:0.0134867 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: created link left_finger_0_base_rotation_link
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawName:left_finger_0_base_rotation_link
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawPose:0.0229502 1.05565 0.59021 1.57079
 * -0 3.14159 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawMass:0.05 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawInertialaPose:0 0 0 0 -0 0 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxx:1.25e-05 [Process: DRCNetworkProcessor] Info
 * in SDFLinkHolder: rawIxy:0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyy:1.25e-05 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIzz:2e-06 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: created link
 * left_finger_0_proximal_link [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawName:left_finger_0_proximal_link [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawPose:0.0229502 1.06915 0.590211 -0.00079 0.00159265 -2.64351e-06 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawMass:0.0288 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawInertialaPose:-0.00263 0.002396 0.02177 1.57 -1.57 -1.39418e-13 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxx:2.42069e-06 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIxy:-1.44904e-06 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxz:-7.74579e-07 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyy:1.1962e-06
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:1.90704e-07 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:1.27365e-06 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: created link
 * left_finger_0_flexible_link_hidden_between_proximal_link_and_1 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawName:left_finger_0_flexible_link_hidden_between_proximal_link_and_1
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawPose:0.0230537 1.0692 0.65521 -0.00079
 * 0.00159265 -2.64351e-06 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawMass:0.00058
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawInertialaPose:0 0 0 -1.57 -1.57
 * 1.39418e-13 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxx:8.8101e-08 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyy:4.09812e-08
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIzz:5.31842e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * created link left_finger_0_flexible_link_1 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawName:left_finger_0_flexible_link_1 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawPose:0.0230537 1.0692 0.65521 -0.00079 0.00159265 -2.64351e-06 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawMass:0.00058 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0 0 0 -1.57 -1.57 1.39418e-13 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIxx:8.8101e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIyy:4.09812e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIyz:0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:5.31842e-08 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: created link
 * left_finger_0_flexible_link_hidden_between_1_and_2 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawName:left_finger_0_flexible_link_hidden_between_1_and_2 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawPose:0.0230665 1.0692 0.66321 -0.00079 0.00159265
 * -2.64351e-06 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawMass:0.00058 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawInertialaPose:0 0 0 -1.57 -1.57 1.39418e-13
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxx:8.8101e-08 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyy:4.09812e-08
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIzz:5.31842e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * created link left_finger_0_flexible_link_2 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawName:left_finger_0_flexible_link_2 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawPose:0.0230665 1.0692 0.66321 -0.00079 0.00159265 -2.64351e-06 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawMass:0.00058 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:0 0 0 -1.57 -1.57 1.39418e-13 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIxx:8.8101e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIyy:4.09812e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIyz:0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:5.31842e-08 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: created link
 * left_finger_0_flexible_link_hidden_between_2_and_distal_link [Process: DRCNetworkProcessor] Info
 * in SDFLinkHolder: rawName:left_finger_0_flexible_link_hidden_between_2_and_distal_link [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawPose:0.0230744 1.06921 0.66821 -0.00079 0.00159265
 * -2.64351e-06 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawMass:0.00058 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawInertialaPose:0 0 0 -1.57 -1.57 1.39418e-13
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIxx:8.8101e-08 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxy:0 [Process: DRCNetworkProcessor] Info in
 * SDFLinkHolder: rawIxz:0 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyy:4.09812e-08
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:0 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIzz:5.31842e-08 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * created link left_finger_0_distal_link [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawName:left_finger_0_distal_link [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawPose:0.0230744 1.06921 0.66821 -0.00079 0.00159265 -2.64351e-06 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawMass:0.0133 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawInertialaPose:-6.6e-05 -0.003142 0.016633 3.14159 0.00159265 3.14159 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIxx:1.62414e-06 [Process: DRCNetworkProcessor]
 * Info in SDFLinkHolder: rawIxy:5.6e-10 [Process: DRCNetworkProcessor] Info in SDFLinkHolder:
 * rawIxz:1.33e-10 [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyy:1.71788e-06
 * [Process: DRCNetworkProcessor] Info in SDFLinkHolder: rawIyz:-1.1997e-07 [Process:
 * DRCNetworkProcessor] Info in SDFLinkHolder: rawIzz:5.60221e-07
 */
