package us.ihmc.robotics.math.filters;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 * This class is used to have a varaible switch slowly between two inputs. When initialized with the startTransition
 * method, the classes internal clock (Ti) is "reset" (Ti=0). V is the value of the varaible. The update method must be called every clock
 * tick. At Ti=0, V=initialVariableValue. At Ti=transitionTime, V=finalVariableValue. For 0<Ti<transitionTime:
 *  V = alpha * initialVariableValue + (1.0 - alpha) * finalVariableValue
 * where
 *  alpha = 0.5 * (1.0 + Math.cos(Ti * Math.PI / transitionTime))
 *
 * Unitl the startTransition method is called, alpha starts at 1.0 (i.e.) V = finalVariableValue.
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class TwoVariableTransitionFilter extends DoubleYoVariable
{
   private final DoubleYoVariable transitionStartTime;
   private final DoubleYoVariable time;
   private double lengthOfTransitionTime = 0.0;

   public TwoVariableTransitionFilter(String name, YoVariableRegistry registry, DoubleYoVariable time)
   {
      super(name, registry);

      transitionStartTime = new DoubleYoVariable("transStartTime_" + name, registry);

      //    alpha = new YoVariable("alpha_" + name, registry);

      this.time = time;
   }

   /**
    * This method must be called every clock tick.
    *
    * @param initialVariableValue is the initial variable to be used
    * @param finalVariableValue is the final variable to be used
    */
   public void update(double initialVariableValue, double finalVariableValue)
   {
      double initialValueScale, finalValueScale;
      if (lengthOfTransitionTime > 0.0)
      {
         double timeSinceLastStartTransition = time.getDoubleValue() - transitionStartTime.getDoubleValue();

         if (timeSinceLastStartTransition < 0.0)
         {
            initialValueScale = 1.0;
            finalValueScale = 0.0;
         }
         else if (timeSinceLastStartTransition < lengthOfTransitionTime)
         {
            initialValueScale = 0.5 * (1.0 + Math.cos(timeSinceLastStartTransition * Math.PI / lengthOfTransitionTime));
            finalValueScale = 1.0 - initialValueScale;
         }
         else
         {
            initialValueScale = 0.0;
            finalValueScale = 1.0;
         }
      }
      else
      {
         initialValueScale = 0.0;
         finalValueScale = 1.0;
      }

      this.set(initialValueScale * initialVariableValue + finalValueScale * finalVariableValue);
   }

   public void startTransition(double lengthOfTransitionTime)
   {
      transitionStartTime.set(time.getDoubleValue());
      this.lengthOfTransitionTime = lengthOfTransitionTime;
   }

   public static void main(String[] args)
   {
      YoVariableRegistry reg = new YoVariableRegistry("main");

      DoubleYoVariable time = new DoubleYoVariable("time", "", null);

      TwoVariableTransitionFilter testVariable = new TwoVariableTransitionFilter("testVariable", reg, time);

      double deltaT = 0.05;
      double endT = 4.0;

      int numberOfPoints = (int) (endT / deltaT);

      double[][] initialData = new double[2][numberOfPoints];
      double[][] finalData = new double[2][numberOfPoints];
      double[][] filteredData = new double[2][numberOfPoints];

      boolean setStart = false;

      double initialValue, finalValue;

      initialValue = 0.0;
      finalValue = 1.0;

      double t = 0.0;
      for (int i = 0; i < numberOfPoints; i++)
      {
         if ((t > 0.7) && !setStart)
         {
            testVariable.startTransition(2.5);
            setStart = true;
         }

         initialValue = Math.sin(t * 2.0 * Math.PI / 0.7);
         finalValue = 3.0 * Math.sin(t * 2.0 * Math.PI / 0.5);

         testVariable.update(initialValue, finalValue);

         initialData[0][i] = t;
         initialData[1][i] = initialValue;

         finalData[0][i] = t;
         finalData[1][i] = finalValue;

         filteredData[0][i] = t;
         filteredData[1][i] = testVariable.getDoubleValue();

         t = t + deltaT;
         time.set(t);

         //       System.out.println(t + " " + testVariable.val);
      }

      ArrayList<double[][]> listOfCurves = new ArrayList<double[][]>();
      listOfCurves.add(initialData);
      listOfCurves.add(finalData);
      listOfCurves.add(filteredData);

   }

}
