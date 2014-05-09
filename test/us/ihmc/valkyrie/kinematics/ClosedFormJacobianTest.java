package us.ihmc.valkyrie.kinematics;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.valkyrie.kinematics.util.ClosedFormJacobian;

public class ClosedFormJacobianTest
{
   double TOLERANCE = 1E1;
   double[] roll =  new double[]{ 0.0, 0.1, 0.1, -0.1, 0.0, 0.0, 0.25 };
   double[] pitch =  new double[]{ 0.0, 0.2, -0.2, 0.2, 0.35, -0.35, 0.0};
   ClosedFormJacobian f2J = new ClosedFormJacobian();
   
   double[] m11_matlab = new double[]{-0.0341186865059837,   -0.0298823091357904,  -0.0318160782835614,  -0.0344018257826992,  -0.0303559109244497,  -0.0334135464455588,  -0.0267978328143697};
   double[] m12_matlab = new double[]{0.0341186865059837, 0.0344018257826992,   0.0364053846119558,   0.0298823091357904,   0.0303559109244497,   0.0334135464455588,   0.0381505409007311};
   double[] m21_matlab = new double[]{0.0366712094326246, 0.0369525474192938,   0.0355099629333052,   0.0347403295453367,   0.0344099137953029,   0.0340672933874358,   0.0376791531319578};
   double[] m22_matlab = new double[]{0.0366712094326246, 0.0347403295453367,   0.0362023558757294,   0.0369525474192938,   0.0344099137953029,   0.0340672933874358,   0.0353981354095287};
   
   @Test
   public void matchesMATLAB()
   {
      System.out.println("Running MATLAB Tests");
      for(int i=0; i<7; i++){
         double[][] J = f2J.getUpdatedTransform(roll[i], pitch[i]);
         System.out.println( J[0][0] + ", " + J[0][1] + ", " + J[1][0] + ", " + J[1][1] );
         assertEquals(J[0][0], m11_matlab[i], TOLERANCE);
         assertEquals(J[0][1], m12_matlab[i], TOLERANCE);
         assertEquals(J[1][0], m21_matlab[i], TOLERANCE);
         assertEquals(J[1][1], m22_matlab[i], TOLERANCE);
      }
   }

   double[] tau5_roll_jerry = new double[]{0.034118686505983736, 0.03640538461195576, 0.03440182578269918, 0.03181607828356141, 0.03341354644555879, 0.030355910924449715, 0.038150540900731125};
   double[] tau6_roll_jerry = new double[]{0.034118686505983736, 0.03640538461195576, 0.03440182578269918, 0.03181607828356141, 0.03341354644555879, 0.030355910924449715, 0.038150540900731125};
   double[] tau5_pitch_jerry = new double[]{-0.0366712094326246, -0.036202355875729446, -0.034740329545336665, -0.035509962933305175, -0.03406729338743576, -0.03440991379530292, -0.03539813540952868};
   double[] tau6_pitch_jerry = new double[]{-0.0366712094326246, -0.035509962933305175, -0.03695254741929382, -0.036202355875729446, -0.03406729338743576, -0.03440991379530292, -0.037679153131957736};
   
   @Test
   public void matchesJerry()
   {
      System.out.println("Running Jerry Tests");
      for(int i=0; i<7; i++){
         double[][] J = f2J.getUpdatedTransform(roll[i], pitch[i] + 7.5*Math.PI/180.0);
         double tau5_roll = J[0][0];
         double tau5_pitch = J[1][0];
         double tau6_roll = J[0][1];
         double tau6_pitch = J[1][1];
         System.out.println( tau5_roll + ", " + tau6_roll + ", " + tau5_pitch + ", " + tau6_pitch );
         assertEquals(tau5_roll, tau5_roll_jerry[i], TOLERANCE);
         assertEquals(tau6_roll, tau6_roll_jerry[i], TOLERANCE);
         assertEquals(tau5_pitch, tau5_pitch_jerry[i], TOLERANCE);
         assertEquals(tau6_pitch, tau6_pitch_jerry[i], TOLERANCE);
      }
   }
}
