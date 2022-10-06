package us.ihmc.promp.test;

import us.ihmc.promp.ProMPNativeLibrary;

import static us.ihmc.promp.presets.ProMPInfoMapper.EigenMatrixXd;

public class EigenTest
{
   public static void main(String[] args)
   {
      ProMPNativeLibrary.load();

      EigenMatrixXd matrixXd = new EigenMatrixXd(2, 2);

      matrixXd.apply(0, 0).put(3);
      matrixXd.apply(1, 0).put(2.5);
      matrixXd.apply(0, 1).put(-1);
      matrixXd.apply(1, 1).put(matrixXd.coeff(1, 0) + matrixXd.coeff(0, 1));

      matrixXd.debugPrint("Test matrix");
   }
}
