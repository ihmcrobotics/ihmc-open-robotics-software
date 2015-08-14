package us.ihmc.moonwalking.models.InvertedPendulumWithFoot;

import us.ihmc.robotics.MathTools;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class InvertedPendulumWithFootController implements RobotController
{
   private final double KP_DEFAULT = 1.04;    // 1.1;
   private final double KD_DEFAULT = 0.24;

   private final YoVariableRegistry yoVariableRegistry;
   private final YoVariable tau;
   private final YoVariable q_d;
   private final YoVariable qd_d;

   private final YoVariable kp;
   private final YoVariable kd;
   private final YoVariable error;
// private final YoVariable error_d;
// private final AlphaFilteredYoVariable error_d_Filt;

   private final YoVariable errorD;
   private final YoVariable percentOvershoot;
// private final YoVariable passedDesired;
   private final YoVariable initialErrorDirection;
   private final YoVariable initialError;

   private final YoVariable riseTime, riseTimeSet;

   private final YoVariable settled;

   private double previousError;
   private double previousTime;

   private final YoVariable omegaN_calc, omegaD_calc, zeta, percentOvershoot_calc;
   private final YoVariable omegaD;

   private final YoVariable timeOfLastErrorPeak;
   private final YoVariable lastError_d;


   private final InvertedPendulumWithFoot invertedPendulumWithFoot;

   private final double gravity, pendulumLength, pendulumMass;


   public InvertedPendulumWithFootController(InvertedPendulumWithFoot invertedPendulumWithFoot)
   {
      this.gravity = invertedPendulumWithFoot.getGravity();
      this.pendulumLength = invertedPendulumWithFoot.getPendulumLength();
      this.pendulumMass = invertedPendulumWithFoot.getPendulumMass();

      yoVariableRegistry = new YoVariableRegistry("controller");
      tau = new DoubleYoVariable("tau", yoVariableRegistry);
      q_d = new DoubleYoVariable("q_d", yoVariableRegistry);
      qd_d = new DoubleYoVariable("qd_d", yoVariableRegistry);

      error = new DoubleYoVariable("error", yoVariableRegistry);
      errorD = new DoubleYoVariable("errorD", yoVariableRegistry);

//    error_d = new DoubleYoVariable("error_d", yoVariableRegistry);

      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(0.3, InvertedPendulumWithFootSimulation.DT);
//    error_d_Filt = new AlphaFilteredYoVariable("error_d_Filt", yoVariableRegistry, alpha, error_d);

      percentOvershoot = new DoubleYoVariable("percentOvershoot", yoVariableRegistry);
      initialErrorDirection = new YoVariable("initialErrorDirectionPositive", ErrorDirection.values(), yoVariableRegistry);
      initialError = new DoubleYoVariable("initialError", yoVariableRegistry);

      riseTime = new DoubleYoVariable("riseTime", yoVariableRegistry);
      riseTimeSet = new BooleanYoVariable("riseTimeSet", yoVariableRegistry);

      omegaN_calc = new DoubleYoVariable("omegaN_calc", yoVariableRegistry);
      omegaD_calc = new DoubleYoVariable("omegaD_calc", yoVariableRegistry);
      zeta = new DoubleYoVariable("zeta", yoVariableRegistry);
      percentOvershoot_calc = new DoubleYoVariable("percentOvershoot_calc", yoVariableRegistry);
      omegaD = new DoubleYoVariable("omegaD", yoVariableRegistry);
      timeOfLastErrorPeak = new DoubleYoVariable("timeOfLastErrorPeak", yoVariableRegistry);
      lastError_d = new YoVariable("lastError_d", Sign.values(), yoVariableRegistry);
      settled = new BooleanYoVariable("settled", yoVariableRegistry);

      reset();

      kp = new DoubleYoVariable("kp", yoVariableRegistry);
      kd = new DoubleYoVariable("kd", yoVariableRegistry);

      kp.val = KP_DEFAULT;
      kd.val = KD_DEFAULT;

      this.invertedPendulumWithFoot = invertedPendulumWithFoot;
   }

   public void setKp(double kp)
   {
      this.kp.val = kp;
   }


   public void setKd(double kd)
   {
      this.kd.val = kd;
   }

   public double getPercentOvershoot()
   {
      return percentOvershoot.val;
   }


   public boolean doesControllerGiveUpOrSettled()
   {
//    if (Math.abs(error.val) > (1.1 * Math.abs(initialError.val)))
//    {
//      System.out.println("error=" + error.val);
//      return true;
//    }
//    else
//      return false;

      if (Math.abs(invertedPendulumWithFoot.getBodyPitch()) > 1.0)
         return true;
      else if (settled.getBooleanValue() == true)
         return true;
      else
         return false;
   }


   public void doControl()
   {
      double kpEffective = pendulumMass * gravity * pendulumLength * kp.val;
      double kdEffective = pendulumMass * gravity * pendulumLength * Math.sqrt(pendulumLength / gravity) * kd.val;

      calculateResponseVariables(kpEffective, kdEffective);

      error.val = q_d.val - invertedPendulumWithFoot.getAngularPosition();
      errorD.val = qd_d.val - invertedPendulumWithFoot.getAngularVelocity();

//    error_d_Filt.update();

      checkSettled(error.val, errorD.val);

      double time = invertedPendulumWithFoot.getTime();
      checkForErrorReversal(errorD.val, time);

//    if (!Double.isNaN(previousTime))
//    {
//      error_d.val = (error.val - previousError)/(time - previousTime);
//    }
//    else
//    {
//      error_d.val = 0.0;
//    }


      if (initialErrorDirection.getEnumValue() == ErrorDirection.NOT_SET)
      {
         initialError.val = error.val;

         if (error.val > 0.0)
            initialErrorDirection.set(ErrorDirection.POSITIVE);
         else if (error.val < 0.0)
            initialErrorDirection.set(ErrorDirection.NEGATIVE);
         else
            initialErrorDirection.set(ErrorDirection.ZERO);
      }


      switch ((ErrorDirection) initialErrorDirection.getEnumValue())
      {
         case POSITIVE :
         case NEGATIVE :
         {
            if ((initialError.val * error.val) < 0.0)
            {
               // the the error is an overshoot
               double currentPercentOverShoot = Math.abs(error.val / initialError.val);
               if (currentPercentOverShoot > percentOvershoot.val)
               {
                  percentOvershoot.val = currentPercentOverShoot;
               }
            }


            if (!riseTimeSet.getBooleanValue())
            {
               if ((error.val / initialError.val) < 0.37)
               {
                  riseTimeSet.set(true);
                  riseTime.set(invertedPendulumWithFoot.getTime());
               }
            }

            break;
         }

         case ZERO :
         case NOT_SET :
         {
            // do nothing
            break;
         }
      }



      tau.val = kpEffective * error.val + kdEffective * errorD.val;

      tau.val = clipTorqueBasedOnFootWidth(tau.val);

      invertedPendulumWithFoot.setTorque(tau.val);

      previousError = error.val;
      previousTime = time;
   }

   /**
    * checkSettled
    *
    * @param d double
    * @param d1 double
    */
   private void checkSettled(double error, double errorDot)
   {
      if ((Math.abs(error) < 0.015) && (Math.abs(errorDot) < 0.005))
         settled.set(true);
      else
         settled.set(false);
   }

   /**
    * clipTorqueBasedOnFootWidth
    *
    * @param d double
    * @return double
    */
   private double clipTorqueBasedOnFootWidth(double desiredTorque)
   {
      double maximumtorque = invertedPendulumWithFoot.getPendulumMass() * invertedPendulumWithFoot.getGravity() * invertedPendulumWithFoot.getFootLength()
                             / 2.0;

      return MathTools.clipToMinMax(desiredTorque, -maximumtorque, maximumtorque);
   }

   /**
    * calculateNaturalFrequence
    *
    * @param d double
    */
   private void checkForErrorReversal(double error_d, double time)
   {
      switch ((Sign) lastError_d.getEnumValue())
      {
         case NOT_SET :
         {
            if (error_d > 0.0)
               lastError_d.set(Sign.POSITIVE);
            else
               lastError_d.set(Sign.NOT_POSITIVE);

            break;
         }

         case POSITIVE :
         {
            if (error_d > 0.0)
            {
               // do nothing
            }
            else
            {
               // It changed sign
               lastError_d.set(Sign.NOT_POSITIVE);
               calculateNaturalFrequency(time);
            }

            break;
         }

         case NOT_POSITIVE :
         {
            if (error_d > 0.0)
            {
               // It changed sign
               lastError_d.set(Sign.POSITIVE);
               calculateNaturalFrequency(time);
            }
            else
            {
               // do nothing
            }

            break;
         }
      }
   }

   private void calculateNaturalFrequency(double time)
   {
      if (Double.isNaN(timeOfLastErrorPeak.val))
      {
         timeOfLastErrorPeak.val = time;
      }
      else
      {
         // multiply by two because these are actually peak to trough times
         double period = 2.0 * (time - timeOfLastErrorPeak.val);
         omegaD.val = 2.0 * Math.PI / period;
         timeOfLastErrorPeak.val = time;
      }
   }

   private void calculateResponseVariables(double kp, double kd)
   {
      omegaN_calc.val = Math.sqrt((kp - pendulumMass * gravity * pendulumLength) / (pendulumMass * pendulumLength * pendulumLength));
      zeta.val = kd / (2.0 * pendulumMass * pendulumLength * pendulumLength * omegaN_calc.val);


      if (zeta.val > 1.0)
      {
         percentOvershoot_calc.val = 0.0;
         omegaD_calc.val = 0.0;
      }
      else
      {
         double sqrtTerm = Math.sqrt(1.0 - zeta.val * zeta.val);
         double term1 = -Math.PI * zeta.val / sqrtTerm;
         percentOvershoot_calc.val = Math.exp(term1);
         omegaD_calc.val = omegaN_calc.val * sqrtTerm;
      }


//    zeta
//    omegaD_calc,
//    percentOvershoot_calc
   }

   public void reset()
   {
      percentOvershoot.val = 0.0;
      initialErrorDirection.set(ErrorDirection.NOT_SET);
      initialError.val = Double.NaN;
      previousError = 0.0;
      previousTime = Double.NaN;
//    error_d_Filt.reset();
      riseTime.val = 0.0;
      riseTimeSet.set(false);
      timeOfLastErrorPeak.val = Double.NaN;
      lastError_d.set(Sign.NOT_SET);
      settled.set(false);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return yoVariableRegistry;
   }

   private enum ErrorDirection {POSITIVE, NEGATIVE, ZERO, NOT_SET}

   ;
   private enum Sign {POSITIVE, NOT_POSITIVE, NOT_SET}

   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   public String getName()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }

   ;

}
