package us.ihmc.exampleSimulations.singgleLeggedRobot;

public class MPCSolversForSinggleLeggedRobot
{
   private int failedNum = 0;

   public MPCSolversForSinggleLeggedRobot()
   {

   }

   public double GradientDescentMethodVer1(int k,
                                           double initialPosition,
                                           double currentVelocity,
                                           double previousVelocity,
                                           double dT,
                                           double mass,
                                           double penalizeWeighting,
                                           MatrixForML desiredPositionSeq)
   {
      double ret;
      double g = 9.81;
      double meanDeltaForce;
      double forceLimit = 170.0;

      MatrixForML force = new MatrixForML(k, 1);
      MatrixForML dcostfunc_dforce = new MatrixForML(k, 1);
      MatrixForML position = new MatrixForML(k, 1);
      MatrixForML prevVelocity = new MatrixForML(k, 1);

      int iteration = 0;
      double initialPoint = mass * g;

      /**
       * Magic Numbers * ***** *** * *
       */
      int maxIteration = 1000;
      double learningRate = 0.020;
      double terminateCondition = 1e-4;

      /**
       * End Magic Numbers.
       */

      for (int i = 0; i < k; i++)
      {
         force.set(i, 0, initialPoint); // Set initial point
      }

      while (true)
      {
         iteration++;

         for (int i = 0; i < k; i++)
         {
            if (i == 0)
            {
               prevVelocity.set(i, 0, previousVelocity);
               position.set(i, 0, initialPosition + dT * currentVelocity);
            }
            else if (i == 1)
            {
               prevVelocity.set(i, 0, currentVelocity);
               position.set(i,
                            0,
                            position.getDoubleValue(i - 1, 0) + dT * dT / mass * force.getDoubleValue(i - 1, 0) + dT * prevVelocity.getDoubleValue(i, 0)
                                  - dT * dT * g);
            }
            else
            {
               prevVelocity.set(i, 0, prevVelocity.getDoubleValue(i - 1, 0) + dT * (force.getDoubleValue(i - 2, 0) / mass - g));
               position.set(i,
                            0,
                            position.getDoubleValue(i - 1, 0) + dT * dT / mass * force.getDoubleValue(i - 1, 0) + dT * prevVelocity.getDoubleValue(i, 0)
                                  - dT * dT * g);
            }
         }

         for (int j = 0; j < k; j++)
         {
            double temp = 0.0;
            for (int i = 0; i < k; i++)
            {
               double dz_df = 0.0;

               if (i - 2 < j)
               {
                  dz_df = 0.0;

               }
               else
               {
                  //                  dz_df = (i - j - 1) * dT * dT / mass; // Too small.. 
                  dz_df = (i - j - 1) * 1.0;

               }
               temp += temp + dz_df * 2.0 * (position.getDoubleValue(i, 0) - desiredPositionSeq.getDoubleValue(i, 0));

            }
            dcostfunc_dforce.set(j, 0, temp + 2 * penalizeWeighting * force.getDoubleValue(j, 0));
         }

         for (int i = 0; i < k; i++)
         {
            double nextForce = force.getDoubleValue(i, 0) - learningRate * dcostfunc_dforce.getDoubleValue(i, 0);
            force.set(i, 0, nextForce);
         }

         meanDeltaForce = learningRate * learningRate * dcostfunc_dforce.transpose().dot(dcostfunc_dforce).getDoubleValue(0, 0) / k;

         if (iteration > maxIteration)
         {
            failedNum++;
            System.out.println("failed to find optimal force.");
            //            System.out.println("force:" + force.getDoubleValue(0, 0));
            System.out.println("failed num: " + failedNum);
            break;
         }

         if (meanDeltaForce < terminateCondition)
         {
            System.out.println("iteration : " + iteration);
            System.out.println("success to find optimal force!!!");
            //            System.out.println("=======Force=======");
            //            force.printMat();
            System.out.println("failed num: " + failedNum);
            break;
         }

      }

      ret = force.getDoubleValue(0, 0);
      if (ret > forceLimit)
      {
         ret = forceLimit;
      }
      else if (ret < -forceLimit)
      {
         ret = -forceLimit;
      }
      else
      {

      }
      return ret;
   }

   /*
    * position and velocity trajectory. It takes a lot time in calculating the matrix dot product.
    */
   public double GradientDescentMethodVer2(int k,
                                           double initialPosition,
                                           double currentVelocity,
                                           double previousVelocity,
                                           double dT,
                                           double mass,
                                           double penalizeWeighting,
                                           MatrixForML desiredSeq)
   {
      int iteration = 0;
      double g = 9.81;
      double initialPoint = mass * g;
      double ret;
      double meanDeltaForce;
      double forceLimit = 200.0;
      MatrixForML weightingMat = new MatrixForML(2, 2);
      MatrixForML force = new MatrixForML(k, 1);
      MatrixForML dcostfunc_dforce = new MatrixForML(k, 1);
      MatrixForML states = new MatrixForML(k, 2);
      MatrixForML dz_df = new MatrixForML(1, 2);
      MatrixForML tempStates = new MatrixForML(2, 1);
      MatrixForML tempDesiredStates = new MatrixForML(2, 1);

      /**
       * Magic Numbers.
       */
      int maxIteration = 200;
      double learningRate = 0.08;
      double terminateCondition = 1e-3;
      weightingMat.set(0, 0, 1.5);
      weightingMat.set(1, 1, 0.001);
      /**
       * End Magic Numbers.
       */

      for (int i = 0; i < k; i++)
      {
         force.set(i, 0, initialPoint);
      }

      while (true)
      {
         iteration++;

         for (int i = 0; i < k; i++)
         {
            if (i == 0)
            {
               states.set(i, 0, initialPosition + dT * currentVelocity);
               states.set(i, 1, currentVelocity + dT * (force.getDoubleValue(i, 0) / mass - g));
            }
            else if (i == 1)
            {
               states.set(i, 0, states.getDoubleValue(i - 1, 0) + dT * dT / mass * force.getDoubleValue(i - 1, 0) + dT * currentVelocity - dT * dT * g);
               states.set(i, 1, states.getDoubleValue(i - 1, 1) + dT * (force.getDoubleValue(i, 0) / mass - g));
            }
            else
            {
               states.set(i,
                          0,
                          states.getDoubleValue(i - 1, 0) + dT * dT / mass * force.getDoubleValue(i - 1, 0) + dT * states.getDoubleValue(i - 2, 1)
                                - dT * dT * g);
               states.set(i, 1, states.getDoubleValue(i - 1, 1) + dT * (force.getDoubleValue(i, 0) / mass - g));
            }
         }

         for (int j = 0; j < k; j++)
         {
            double temp = 0.0;
            for (int i = 0; i < k; i++)
            {
               tempStates.set(0, 0, states.getDoubleValue(i, 0));
               tempStates.set(1, 0, states.getDoubleValue(i, 1));

               tempDesiredStates.set(0, 0, desiredSeq.getDoubleValue(i, 0));
               tempDesiredStates.set(1, 0, desiredSeq.getDoubleValue(i, 1));

               if (i <= j)
               {
                  dz_df.set(0, 0, 0);
                  dz_df.set(0, 1, 0);
               }

               else if (i - 2 < j)
               {
                  dz_df.set(0, 0, 0);
                  dz_df.set(0, 1, (i - j) / dT);
               }
               else
               {
                  //                  dz_df = (i - j - 1) * dT * dT / mass; // Too small.. 
                  dz_df.set(0, 0, (i - j - 1.0));
                  dz_df.set(0, 1, (i - j) / dT);
               }
               temp += temp + dz_df.dot(weightingMat).dot(tempStates.sub(tempDesiredStates)).getDoubleValue(0, 0);
            }
            dcostfunc_dforce.set(j, 0, temp + 2 * penalizeWeighting * force.getDoubleValue(j, 0));
         }

         for (int i = 0; i < k; i++)
         {
            double nextForce = force.getDoubleValue(i, 0) - learningRate * dcostfunc_dforce.getDoubleValue(i, 0);
            if (nextForce > forceLimit)
            {
               nextForce = forceLimit;
            }
            else if (nextForce < -forceLimit)
            {
               nextForce = -forceLimit;
            }
            else
            {

            }
            force.set(i, 0, nextForce);
         }

         meanDeltaForce = learningRate * learningRate * dcostfunc_dforce.transpose().dot(dcostfunc_dforce).getDoubleValue(0, 0) / k;

         if (iteration > maxIteration)
         {
            failedNum++;
            System.out.println("failed to find optimal force.");
            System.out.println("failed num: " + failedNum);
            break;
         }

         if (meanDeltaForce < terminateCondition)
         {
            System.out.println("success to find optimal force!!!");
            System.out.println("failed num: " + failedNum);
            System.out.println("iterations: " + iteration);
            break;
         }
      }
      ret = force.getDoubleValue(0, 0);
      
      return ret;
   }

   /*
    * TODO LM method : super slow... and really bad performance (check equations)
    */
   public double LMMethodVer1(int k,
                          double initialPosition,
                          double currentVelocity,
                          double previousVelocity,
                          double dT,
                          double mass,
                          double penalizeWeighting,
                          MatrixForML desiredPositionSeq)
   {
      double ret;
      double g = 9.81;
      double deltaForce;
      double forceLimit = 200.0;
      double error = desiredPositionSeq.getDoubleValue(0, 0) - initialPosition;
      double lambda = 0.1;
      int maxIteration = 200;      
      double terminateCondition = 1e-3;
      
      MatrixForML force = new MatrixForML(k, 1);
      MatrixForML jacobian = new MatrixForML(1, k);
      MatrixForML position = new MatrixForML(k, 1);
      MatrixForML prevVelocity = new MatrixForML(k, 1);
      MatrixForML identityMat = new MatrixForML(k, k);
      MatrixForML previousForce = new MatrixForML(k, 1);
      MatrixForML tempMat = new MatrixForML(k, 1);
      int iteration = 0;
      double initialPoint = mass * g;
      
      for (int i = 0; i < k; i++)
      {
         force.set(i, 0, initialPoint);
         identityMat.set(i, i, 1.0);
      }

      while (true)
      {
         iteration++;
         for (int i = 0; i < k; i++)
         {
            if (i == 0)
            {
               position.set(i, 0, initialPosition + dT * dT / mass * force.getDoubleValue(i, 0) + dT * previousVelocity - dT * dT * g);
            }
            else
            {
               position.set(i,
                            0,
                            position.getDoubleValue(i - 1, 0) + dT * dT / mass * force.getDoubleValue(i, 0) + dT * prevVelocity.getDoubleValue(i, 0)
                                  - dT * dT * g);
            }
            if (i == 0)
            {
               prevVelocity.set(i, 0, previousVelocity);
            }
            else if (i == 1)
            {
               prevVelocity.set(i, 0, currentVelocity);
            }
            else
            {
               prevVelocity.set(i, 0, prevVelocity.getDoubleValue(i - 1, 0) + dT * (force.getDoubleValue(i - 2, 0) / mass - g));
            }

         }

         for (int j = 0; j < k; j++)
         {
            double temp = 0.0;
            for (int i = 0; i < k; i++)
            {
               double dz_df = 0.0;

               if (i <= j)
               {
                  dz_df = 0.0;
               }
               else if ((i - j) == 1)
               {
                  //                  dz_df = dT * dT / mass;
                  dz_df = 1.0;
               }
               else
               {
                  //                  dz_df = (i - j - 1) * dT * dT / mass;
                  dz_df = (i - j - 1) * 1.0;
               }
               temp += temp + dz_df * 2.0 * (position.getDoubleValue(i, 0) - desiredPositionSeq.getDoubleValue(i, 0));

            }
            jacobian.set(0, j, temp + 2 * penalizeWeighting * force.getDoubleValue(j, 0));

            previousForce.set(j, 0, force.getDoubleValue(j, 0));
         }

         tempMat = jacobian.transpose().dot(jacobian).add(identityMat.mul(error)).add(identityMat.mul(lambda)).inverse().dot(jacobian.transpose().mul(error));

         for (int i = 0; i < k; i++)
         {
            double nextForce = force.getDoubleValue(i, 0) - tempMat.getDoubleValue(i, 0);
            if (nextForce > forceLimit)
            {
               nextForce = forceLimit;
            }
            else if (nextForce < -forceLimit)
            {
               nextForce = -forceLimit;
            }
            else
            {

            }
            force.set(i, 0, nextForce);
         }

         deltaForce = force.sub(previousForce).transpose().dot(force.sub(previousForce)).getDoubleValue(0, 0);
         
         if (iteration > maxIteration)
         {
            System.out.println("failed to find optimal force.");
            break;
         }

         if (deltaForce < terminateCondition)
         {
            System.out.println("success to find optimal force!!!");
            break;
         }
      }

      ret = force.getDoubleValue(0, 0);

      return ret;
   }

}
