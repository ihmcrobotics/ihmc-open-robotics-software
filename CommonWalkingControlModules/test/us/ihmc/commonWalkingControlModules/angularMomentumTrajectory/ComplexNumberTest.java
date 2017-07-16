package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.ComplexNumber;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;

public class ComplexNumberTest
{
   @Test
   public void testInitialization()
   {
      ComplexNumber c1 = new ComplexNumber();
      assert (c1.getRealPart() == 0.0);
      assert (c1.getImaginaryPart() == 0.0);
      c1 = new ComplexNumber(1, 1);
      assert (c1.getRealPart() == 1.0);
      assert (c1.getImaginaryPart() == 1.0);
      c1.set(2.12, 3.14159);
      assert (c1.getRealPart() == 2.12);
      assert (c1.getImaginaryPart() == 3.14159);
      c1.setFromEuler(2, Math.toRadians(90));
      assert (Math.abs(c1.getRealPart()) < Epsilons.ONE_BILLIONTH);
      assert (c1.getImaginaryPart() == 2);
   }

   @Test
   public void testMath()
   {
      ComplexNumber c1 = new ComplexNumber(2.4, 3.1);
      ComplexNumber c2 = new ComplexNumber(1.2, 4.5);
      ComplexNumber c3 = new ComplexNumber();
      c3.add(c1, c2);
      assert (c3.getRealPart() == c1.getRealPart() + c2.getRealPart());
      assert (c3.getImaginaryPart() == c1.getImaginaryPart() + c2.getImaginaryPart());
      c3.multiply(c1, c2);
      assert (c3.getRealPart() == 2.4 * 1.2 - 4.5 * 3.1);
      assert (c3.getImaginaryPart() == 2.4 * 4.5 + 1.2 * 3.1);
      c3.setToPurelyReal(1);
      assert (c3.getMagnitude() == 1);
      assert (c3.getArgument() == 0);
      c3.getRoots(c1, c2);
      assert (c1.getRealPart() == 1);
      assert (c1.getImaginaryPart() == 0);
      assert (c2.getRealPart() == -1);
      assert (Math.abs(c2.getImaginaryPart()) < Epsilons.ONE_BILLIONTH);
      c2.getRoots(c1, c3);
      assert (Math.abs(c1.getRealPart() - 0) < Epsilons.ONE_BILLIONTH);
      assert (Math.abs(c1.getImaginaryPart() - 1) < Epsilons.ONE_BILLIONTH);
      assert (Math.abs(c3.getRealPart() - 0) < Epsilons.ONE_BILLIONTH);
      assert (Math.abs(c3.getImaginaryPart() + 1) < Epsilons.ONE_BILLIONTH);
   }

   @Test
   public void tesNthRoots()
   {
      ComplexNumber c1 = new ComplexNumber(1.0, 0.0);
      List<ComplexNumber> roots = new ArrayList<>(8);
      for(int i = 0; i < 8; i++)
         roots.add(new ComplexNumber());
      c1.getRoots(roots, 8);
      for(int i = 0; i < roots.size(); i++)
         PrintTools.debug(roots.get(i).toString());
      
   }
}
