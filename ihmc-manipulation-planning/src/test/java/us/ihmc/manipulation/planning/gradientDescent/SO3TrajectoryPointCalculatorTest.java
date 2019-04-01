package us.ihmc.manipulation.planning.gradientDescent;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;

public class SO3TrajectoryPointCalculatorTest
{
   @Test
   public void testSO3TrajectoryPointCalculator()
   {
      Quaternion qt1 = new Quaternion();

      Quaternion qt2 = new Quaternion(qt1);
      qt2.appendYawRotation(Math.PI * 0.1);

      Quaternion qt3 = new Quaternion(qt2);
      qt3.appendRollRotation(Math.PI * 0.1);
      qt3.appendYawRotation(Math.PI * 0.1);

      Quaternion qt4 = new Quaternion(qt3);
      qt4.appendPitchRotation(Math.PI * 0.1);

      Quaternion qt5 = new Quaternion(qt4);
      qt5.appendYawRotation(-Math.PI * 0.05);

      SO3TrajectoryPointCalculator trajectoryPointCalculator = new SO3TrajectoryPointCalculator();

      trajectoryPointCalculator.clear();
      trajectoryPointCalculator.appendTrajectoryPoint(0.0, qt1);
      trajectoryPointCalculator.appendTrajectoryPoint(1.0, qt2);
      trajectoryPointCalculator.appendTrajectoryPoint(2.0, qt3);
      trajectoryPointCalculator.appendTrajectoryPoint(3.0, qt4);
      trajectoryPointCalculator.appendTrajectoryPoint(5.0, qt5);

      trajectoryPointCalculator.compute();

      LogTools.info("done");
   }
}
