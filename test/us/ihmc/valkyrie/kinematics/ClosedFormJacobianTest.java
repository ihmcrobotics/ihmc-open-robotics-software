package us.ihmc.valkyrie.kinematics;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.valkyrie.kinematics.transmissions.InefficientPushrodTransmissionJacobian;
import us.ihmc.valkyrie.kinematics.transmissions.PushRodTransmissionJoint;
import us.ihmc.valkyrie.kinematics.util.ClosedFormJacobian;

public class ClosedFormJacobianTest
{
   private static final boolean DEBUG = true;

   private double TOLERANCE = 1E-7;
   private double[] roll = new double[]
   {
      0.0, 0.1, 0.1, -0.1, 0.0, 0.0, 0.25
   };
   private double[] pitch = new double[]
   {
      0.0, 0.2, -0.2, 0.2, 0.35, -0.35, 0.0
   };
   private ClosedFormJacobian closedFormJacobian = new ClosedFormJacobian(PushRodTransmissionJoint.ANKLE);
   private ClosedFormJacobian closedFormJacobianWaist = new ClosedFormJacobian(PushRodTransmissionJoint.WAIST);
   
   private double[] m11_matlab = new double[]
   {
      -0.0341186865059837, -0.0298823091357904, -0.0318160782835614, -0.0344018257826992, -0.0303559109244497, -0.0334135464455588, -0.0267978328143697
   };
   private double[] m12_matlab = new double[]
   {
      0.0341186865059837, 0.0344018257826992, 0.0364053846119558, 0.0298823091357904, 0.0303559109244497, 0.0334135464455588, 0.0381505409007311
   };
   private double[] m21_matlab = new double[]
   {
      0.0366712094326246, 0.0369525474192938, 0.0355099629333052, 0.0347403295453367, 0.0344099137953029, 0.0340672933874358, 0.0376791531319578
   };
   private double[] m22_matlab = new double[]
   {
      0.0366712094326246, 0.0347403295453367, 0.0362023558757294, 0.0369525474192938, 0.0344099137953029, 0.0340672933874358, 0.0353981354095287
   };

   private double[] m11_matlab_waist = new double[]
   {
         0.063367872788363,
         0.070375770098200,
         0.058422901012790,
         0.067299533258194,
         0.072602680754717,
         0.052124598829027,
         0.065948809462726
   };
   
   private double[] m12_matlab_waist = new double[]
   {
         0.063367872788363,
         0.067299533258194,
         0.055151262022775,
         0.070375770098200,
         0.072602680754717,
         0.052124598829027,
         0.057675420837286
   };
   
   private double[] m21_matlab_waist = new double[]
   {
         -0.045200359335076,
         -0.042531256340934,
         -0.047683204352025,
         -0.040960816370066,
         -0.039979762559698,
         -0.051849035209403,
         -0.043582905494040
   };
   
   private double[] m22_matlab_waist = new double[]
   {
         0.045200359335076,
         0.040960816370066,
         0.049961508736648,
         0.042531256340934,
         0.039979762559698,
         0.051849035209403,
         0.044875400766635
   };
   
   @Test
   public void testJacobianMatchesMATLABAnkle()
   {
      for (int i = 0; i < 7; i++)
      {
         double[][] J = closedFormJacobian.getUpdatedTransform(-roll[i], -pitch[i]);
         if (DEBUG) System.out.println(J[0][0] + ", " + J[0][1] + ", " + J[1][0] + ", " + J[1][1]);
         assertEquals(J[1][0], -m11_matlab[i], TOLERANCE);
         assertEquals(J[1][1], -m12_matlab[i], TOLERANCE);
         assertEquals(J[0][0], -m21_matlab[i], TOLERANCE);
         assertEquals(J[0][1], -m22_matlab[i], TOLERANCE);
      }
   }

   @Test
   public void testJacobianMatchesInefficientImplementationAnkle()
   {
      InefficientPushrodTransmissionJacobian inefficientButReadablePushrodTransmission = new InefficientPushrodTransmissionJacobian(PushRodTransmissionJoint.ANKLE, null, null);

      for (int i = 0; i < 7; i++)
      {
         double[][] J = closedFormJacobian.getUpdatedTransform(roll[i], pitch[i]);    // + 7.5*Math.PI/180.0);

         double tau5_roll = J[1][0];
         double tau5_pitch = J[0][0];
         double tau6_roll = J[1][1];
         double tau6_pitch = J[0][1];

         double[][] inefficientJacobian = new double[2][2];
         inefficientButReadablePushrodTransmission.computeJacobian(inefficientJacobian, pitch[i], roll[i]);

         double tau5_pitch_jerry = inefficientJacobian[0][0];
         double tau6_pitch_jerry = inefficientJacobian[0][1];
         double tau5_roll_jerry = inefficientJacobian[1][0];
         double tau6_roll_jerry = inefficientJacobian[1][1];

         if (DEBUG)
         {
            System.out.println("tau5_roll = " + tau5_roll + ", tau6_roll = " + tau6_roll + ", tau5_pitch = " + tau5_pitch + ", tau6_pitch = " + tau6_pitch);
            System.out.println("tau5_roll_jerry = " + tau5_roll_jerry + ", tau6_roll_jerry = " + tau6_roll_jerry + ", tau5_pitch_jerry[i] = "
                               + tau5_pitch_jerry + ", tau6_pitch_jerry = " + tau6_pitch_jerry);
         }
         assertEquals(tau5_roll, tau5_roll_jerry, TOLERANCE);
         assertEquals(tau6_roll, tau6_roll_jerry, TOLERANCE);
         assertEquals(tau5_pitch, tau5_pitch_jerry, TOLERANCE);
         assertEquals(tau6_pitch, tau6_pitch_jerry, TOLERANCE);
      }
   }

@Test
public void testJacobianMatchesMATLABWaist()
{
   for (int i = 0; i < 7; i++)
   {
      double[][] J = closedFormJacobianWaist.getUpdatedTransform(roll[i], pitch[i]);
      if(DEBUG){
         System.out.println(J[0][0] + ", " + J[0][1] + ", " + J[1][0] + ", " + J[1][1]);
         System.out.println(m11_matlab_waist[i] + ", " + m12_matlab_waist[i] + ", " + m21_matlab_waist[i] + ", " + m22_matlab_waist[i]);         
      }
      assertEquals(J[0][0], -m11_matlab_waist[i], TOLERANCE);
      assertEquals(J[0][1], -m12_matlab_waist[i], TOLERANCE);
      assertEquals(J[1][0], -m21_matlab_waist[i], TOLERANCE);
      assertEquals(J[1][1], -m22_matlab_waist[i], TOLERANCE);
   }
}

//@Test
public void testJacobianMatchesInefficientImplementationWaist()
{
   InefficientPushrodTransmissionJacobian inefficientButReadablePushrodTransmission = new InefficientPushrodTransmissionJacobian(PushRodTransmissionJoint.WAIST, null, null);

   for (int i = 0; i < 7; i++)
   {
      double[][] J = closedFormJacobianWaist.getUpdatedTransform(roll[i], pitch[i]);    // + 7.5*Math.PI/180.0);

      double tau5_pitch = J[0][0];  double tau6_pitch = J[0][1];
      double tau5_roll = J[1][0];   double tau6_roll = J[1][1];

      double[][] inefficientJacobian = new double[2][2];
      inefficientButReadablePushrodTransmission.computeJacobian(inefficientJacobian, pitch[i], roll[i]);

      double tau5_pitch_jerry = inefficientJacobian[0][0];  double tau6_pitch_jerry = inefficientJacobian[0][1];
      double tau5_roll_jerry = inefficientJacobian[1][0];   double tau6_roll_jerry = inefficientJacobian[1][1];

      if (DEBUG)
      {
         System.out.println("");
         System.out.println("tau5_roll_wills = " + tau5_roll + ", tau6_roll_wills = " + tau6_roll + ", tau5_pitch_wills = " + tau5_pitch + ", tau6_pitch_wills = " + tau6_pitch);
         System.out.println("tau5_roll_jerry = " + tau5_roll_jerry + ", tau6_roll_jerry = " + tau6_roll_jerry + ", tau5_pitch_jerry = " + tau5_pitch_jerry + ", tau6_pitch_jerry = " + tau6_pitch_jerry);
      }
      assertEquals(tau5_roll, tau5_roll_jerry, TOLERANCE);
      assertEquals(tau6_roll, tau6_roll_jerry, TOLERANCE);
      assertEquals(tau5_pitch, tau5_pitch_jerry, TOLERANCE);
      assertEquals(tau6_pitch, tau6_pitch_jerry, TOLERANCE);
   }
}

}
