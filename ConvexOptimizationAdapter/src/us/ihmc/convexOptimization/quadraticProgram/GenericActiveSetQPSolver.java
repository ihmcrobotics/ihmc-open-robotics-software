package us.ihmc.convexOptimization.quadraticProgram;

import java.util.logging.Level;
import java.util.logging.Logger;

import org.ejml.alg.dense.decomposition.TriangularSolver;
import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.QRPDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.utilities.math.linearAlgebra.MatrixTools;

public class GenericActiveSetQPSolver extends AbstractActiveSetQPSolver
{
   
   Logger logger = Logger.getLogger(getClass().getSimpleName());
   QRPDecomposition<DenseMatrix64F> qrpEqualityConstraintATranspose= DecompositionFactory.qrp(1, 1);
   QRPDecomposition<DenseMatrix64F> qrpEqualityConstraintA = DecompositionFactory.qrp(1, 1);
   boolean[] inActiveSet;
   int activeEqualitySize=0;

   DenseMatrix64F x;
   double eps=1e-10;
   
   public GenericActiveSetQPSolver()
   {
      super();
      logger.setLevel(Level.OFF);
   }
   
   public void setThreshold(double threshHold)
   {
      this.eps=threshHold;
   }
   
   boolean checkEqualityConstraintFeasibility()
   {
      assert linearEqualityConstraintA!=null && linearEqualityConstraintB!=null;
      qrpEqualityConstraintA.setSingularThreshold(eps);
      qrpEqualityConstraintA.decompose(linearEqualityConstraintA);
      DenseMatrix64F Q=qrpEqualityConstraintA.getQ(null, false);
      DenseMatrix64F QtB = new DenseMatrix64F(Q.getNumRows(),1);
      CommonOps.multTransA(Q, linearEqualityConstraintB, QtB);
      for(int i=qrpEqualityConstraintA.getRank();i<QtB.numRows;i++)
      {
         if(Math.abs(QtB.get(i))>eps)
            return false;
      }
      return true;
   }

  
   
   int[] initializeActiveSet()
   {
      setZeroSizeMatrixForNullFields();
      assert linearEqualityConstraintA!=null;
      assert linearEqualityConstraintATranspose!=null;
      assert linearInequalityConstraintA!=null;
      assert checkEqualityConstraintFeasibility();
      
      qrpEqualityConstraintATranspose.setSingularThreshold(eps);
      qrpEqualityConstraintATranspose.decompose(linearEqualityConstraintATranspose);

      int[] pivots=qrpEqualityConstraintATranspose.getPivots();
      
      activeEqualitySize = qrpEqualityConstraintATranspose.getRank();
      int maxActiveSetSize=activeEqualitySize+(linearInequalityConstraintA!=null?linearInequalityConstraintA.numRows:0);
      activeSetA = new DenseMatrix64F(activeEqualitySize,linearEqualityConstraintA.numCols);
      activeSetB = new DenseMatrix64F(activeEqualitySize, 1);
      inActiveSet = new boolean[maxActiveSetSize];
      int[] selectedEqualityConstraints = new int[activeEqualitySize];
      for(int i=0;i<activeEqualitySize;i++)
      {
         
         int prow = pivots[i];
         selectedEqualityConstraints[i]=prow;
         for(int j=0;j<linearEqualityConstraintA.numCols;j++)
         {
            activeSetA.set(i,j,linearEqualityConstraintA.get(prow,j));
         }
         activeSetB.set(i,0,linearEqualityConstraintB.get(prow,0));
         inActiveSet[i]=true;
      }
      return selectedEqualityConstraints;
   }

   private int addConstraintToActiveSet(int inequalityConstraintIndex)
   {
      //using the RowMajor property
      assert(inequalityConstraintIndex>=0 && inequalityConstraintIndex<linearInequalityConstraintA.numRows);

      activeSetA.reshape(activeSetA.numRows+1, activeSetA.numCols, true);
      CommonOps.extract(linearInequalityConstraintA,
            inequalityConstraintIndex,inequalityConstraintIndex+1,0, linearInequalityConstraintA.numCols,
            activeSetA, activeSetA.numRows-1, 0);

      activeSetB.reshape(activeSetB.numRows+1, 1, true);
      activeSetB.set(activeSetB.numRows-1, 0, linearInequalityConstraintB.get(inequalityConstraintIndex,0));

      inActiveSet[linearEqualityConstraintA.numRows+inequalityConstraintIndex]=true;
      return activeSetB.numRows-1;
   }

   private void removeConstraintFromActiveSet(int indexInActiveSet)
   {
      if(indexInActiveSet<linearEqualityConstraintA.numRows)
         throw new RuntimeException("shouldn't try to remove equality constraint");
      inActiveSet[indexInActiveSet]=false;

      DenseMatrix64F tmpA = new DenseMatrix64F(activeSetA);
      CommonOps.extract(tmpA, indexInActiveSet+1, tmpA.numRows, 0, tmpA.numCols, activeSetA, indexInActiveSet, 0);
      activeSetA.reshape(activeSetA.numRows-1,activeSetA.numCols, true);
      
      DenseMatrix64F tmpB = new DenseMatrix64F(activeSetB);
      CommonOps.extract(tmpB, indexInActiveSet+1, tmpB.numRows, 0, tmpB.numCols, activeSetB, indexInActiveSet, 0);
      activeSetB.reshape(activeSetB.numRows-1,activeSetB.numCols, true);
   }
   
   //consider solve directly from equalities in activeset (guaranteed indepdent)
   private void solveEqualityConstraint(DenseMatrix64F x0)
   {
      qrpEqualityConstraintA.decompose(linearEqualityConstraintA);
      int rank=qrpEqualityConstraintA.getRank();

      if(linearEqualityConstraintA.numCols <= linearEqualityConstraintA.numRows)
      {
        DenseMatrix64F tmpr = new DenseMatrix64F(rank,1);
        DenseMatrix64F Q=qrpEqualityConstraintA.getQ(null,true);
        DenseMatrix64F R=qrpEqualityConstraintA.getR(null,true);
        CommonOps.multTransA(Q,linearEqualityConstraintB,tmpr);
        TriangularSolver.solveU(R.data, tmpr.data, R.numCols);
        CommonOps.insert(tmpr, x0, 0, 0);
      }
      else
      {
         
         DenseMatrix64F tmpr = new DenseMatrix64F(linearEqualityConstraintA.numRows,1);
         qrpEqualityConstraintATranspose.decompose(linearEqualityConstraintATranspose);
         DenseMatrix64F Q=qrpEqualityConstraintATranspose.getQ(null,true);
         DenseMatrix64F R=qrpEqualityConstraintATranspose.getR(null,true);
         CommonOps.insert(linearEqualityConstraintB, tmpr, 0, 0);
         TriangularSolver.solveL(R.data, tmpr.data, R.numCols);
         MatrixTools.multAllowEmptyMatrix(Q,tmpr, x0);
      }
   }
   
   
   public DenseMatrix64F findFeasiblePoint() 
   {
      /**
       * solving initial problem
       * Aeq x=beq
       * Ain (x+lambda) <= bin
       * 
       * [A 0][x;lambda]=b;
       * A2 = [A I][x;-lambda] <= b
       * x0 = [zeros;min(b)];
       */
      GenericActiveSetQPSolver lpSolver=new GenericActiveSetQPSolver();
      int augmentNumVariables = linearEqualityConstraintA.numCols+1;
      
      // solve Aeq x0 = beq using existing QR decomposition
      DenseMatrix64F x0 = new DenseMatrix64F(linearEqualityConstraintA.numCols,1);
      if(getLinearEqualityConstraintsSize()>0)
        solveEqualityConstraint(x0);

      

      // [Aeq 0]*[x;lambda]=beq
      DenseMatrix64F augmentedEqaulityConstraintA = new DenseMatrix64F(linearEqualityConstraintA.numRows,augmentNumVariables);
      CommonOps.insert(linearEqualityConstraintA, augmentedEqaulityConstraintA, 0, 0);
      lpSolver.setLinearEqualityConstraints(augmentedEqaulityConstraintA,linearEqualityConstraintB);
      
      // [Ain 1;]*[x;lambda]<=[bin]
      // [ 0  1;]             [0]
      DenseMatrix64F augmentedInequalityConstraintA = new DenseMatrix64F(linearInequalityConstraintA.numRows+1, augmentNumVariables);
      CommonOps.insert(linearInequalityConstraintA, augmentedInequalityConstraintA, 0, 0);
      MatrixTools.fillColumn(augmentedInequalityConstraintA, augmentedInequalityConstraintA.numCols-1, 1);

      DenseMatrix64F augmentedInequalityConstraintB = new DenseMatrix64F(linearInequalityConstraintB.numRows+1,1);
      CommonOps.insert(linearInequalityConstraintB, augmentedInequalityConstraintB, 0, 0);
      lpSolver.setLinearInequalityConstraints(augmentedInequalityConstraintA, augmentedInequalityConstraintB);


      // cost function =  lambda
      DenseMatrix64F augmentCostVector = new DenseMatrix64F(augmentNumVariables,1);
      augmentCostVector.set(augmentCostVector.numRows-1, 0, -1);
      DenseMatrix64F augmentedCostMatrix=new DenseMatrix64F(augmentNumVariables,augmentNumVariables);
      lpSolver.setQuadraticCostFunction(augmentedCostMatrix, augmentCostVector,0);
      
      
      //initialize augment augX0 =[x0;min(bin-Ain*x)]
      DenseMatrix64F tmpIneq = new DenseMatrix64F(linearInequalityConstraintB);
      CommonOps.multAdd(-1, linearInequalityConstraintA, x0, tmpIneq);
      
      DenseMatrix64F augmentX0 = new DenseMatrix64F(augmentNumVariables, 1);
      CommonOps.insert(x0, augmentX0, 0, 0);
      if(getLinearInequalityConstraintsSize()>0)
        augmentX0.set(augmentX0.numRows-1,0, Math.min(0, CommonOps.elementMin(tmpIneq)));

      assert(augmentX0.numRows==lpSolver.numberOfVariablesToSolve);
      

      lpSolver.setZeroSizeMatrixForNullFields();
//      lpSolver.displayProblem();
//      System.out.println("Initial x0 for feasible point problem");
//      System.out.println(augmentX0);

      DenseMatrix64F solution0=lpSolver.solve(augmentX0);
      double slack=solution0.get(augmentNumVariables-1, 0);
      if(slack < -eps)
      {
        throw new RuntimeException("Constraints Not Feasible, slack="+slack); 
      }
      solution0.setNumRows(solution0.numRows-1);
//      MatrixIO.print(System.out, solution0, "%20.15e");
      return solution0;
   }
   

   private void packGradient(DenseMatrix64F gradient0)
   {
      gradient0.set(quadraticCostFVector);
      CommonOps.multAdd(quadraticCostGMatrix, x, gradient0);
 
   }
   
   /**
    *  solve
    *  min  1/2 p' G p  +  (Gx+f)' * p
    *                    gradient0
    *  subject to active A * p = 0
    */
   private STEPTYPE calculateStepDirection(DenseMatrix64F p)
   {
      //prepare A' = QR
      QRPDecomposition<DenseMatrix64F> qrpActiveSet = DecompositionFactory.qrp(1, 1);
      DenseMatrix64F activeSetATranspose = CommonOps.transpose(activeSetA, null);
      qrpActiveSet.setSingularThreshold(eps);
      qrpActiveSet.decompose(activeSetATranspose);
      DenseMatrix64F basisQ = qrpActiveSet.getQ(null, false);

      //prepare tmp variables 
      int rank=qrpActiveSet.getRank();
      int nullity = activeSetATranspose.numRows-rank;
      DenseMatrix64F tmpx = new DenseMatrix64F(numberOfVariablesToSolve,1);
      DenseMatrix64F tmpr = new DenseMatrix64F(rank,1);
      DenseMatrix64F pz = new DenseMatrix64F(x.numRows-rank,1);
      
      /* Decompose ActiveSetCoefficients
       * Ax=0
       *  A'=QR
       *  Q = [Y | Z] = [rangeBasis | NullBasis];
       *  x = Z pz 
       */
      DenseMatrix64F activeSetNullBasis;
      if(rank == basisQ.numCols)
         activeSetNullBasis = new DenseMatrix64F(basisQ.numRows,0);
      else
         activeSetNullBasis = CommonOps.extract(basisQ, 0, basisQ.numRows, rank, basisQ.numCols);

      DenseMatrix64F activeSetRangeBasis;
      if(rank==0)
         activeSetRangeBasis=new DenseMatrix64F(basisQ.numRows,0);
      else
         activeSetRangeBasis=CommonOps.extract(basisQ, 0, basisQ.numRows, 0, rank);
      
      
      //gradient0 = G x0+f
      DenseMatrix64F gradient0 = new DenseMatrix64F(numberOfVariablesToSolve,1);
      packGradient(gradient0);
      
      // steepestDirection=-Z'gradient
      DenseMatrix64F steepestDirection = new DenseMatrix64F(nullity, 1);
      CommonOps.multTransA(-1, activeSetNullBasis, gradient0, steepestDirection);

      STEPTYPE stepType;
      // projG = (Z' G Z)
      DenseMatrix64F projectedG = new DenseMatrix64F(nullity,nullity);
      MatrixTools.multQuad(activeSetNullBasis, quadraticCostGMatrix, projectedG);

      //Calc Newton step with fallback to Steepest Descent
//      LinearSolver<DenseMatrix64F> cholSolver=LinearSolverFactory.symmPosDef(projectedG.numCols);
      SolvePseudoInverseSvd linearSolver = new SolvePseudoInverseSvd(projectedG.numRows, projectedG.numCols);

      boolean setAResult=false;
      int projectedGRank=0;
      double minSingularValue=Double.MAX_VALUE;
      if(projectedG.numRows>0)
      {
        setAResult=linearSolver.setA(new DenseMatrix64F(projectedG));
//        projectedGRank=SingularOps.rank(linearSolver.getDecomposer(), eps);
        double[] singularValues = linearSolver.getDecomposer().getSingularValues();
//        System.out.println("SV"+Arrays.toString(singularValues));
        for(int i=0;i<singularValues.length;i++)
        {
           double sv=singularValues[i];
           if(!Double.isInfinite(sv))
           {
            if(sv>0)
              projectedGRank++;
            if(sv<minSingularValue)
               minSingularValue=sv;
           }
        }
      }
      if(setAResult && projectedGRank>0)
      {
        // pz = - projG \ Z' gradient0 
        stepType=STEPTYPE.NEWTON;
        linearSolver.solve(steepestDirection, pz);
      }
      else
      {
         //gradientStep = -Z'*gradient0
         stepType=STEPTYPE.STEEPEST;
         pz = steepestDirection;
      }
         
      // stepDirection = Z*pz
      MatrixTools.multAllowEmptyMatrix(activeSetNullBasis, pz, p);
      
      //xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
      if(stepType==STEPTYPE.NEWTON)
      {
         logger.info("minSingularValue="+minSingularValue);
         // -Z'GZ *pz + steep =0
        DenseMatrix64F sol=new DenseMatrix64F(steepestDirection);
        CommonOps.multAdd(-1,projectedG, pz, sol);
        double solNorm=NormOps.normP1(sol);
//        System.out.println("projG pz = steepestDirection, svd accuracy->"+ solNorm);
        try{
          assert(solNorm<eps);
        }
        catch(AssertionError err)
        {
           throw new RuntimeException("Inaccurate LinearSolver solution : solNorm="+solNorm +" Consider set a looser threshold by setThreshold");
        }
        // -GZpz + gradient = 0
        DenseMatrix64F tmpp = new DenseMatrix64F(gradient0); //-Z' "gradient0"
        CommonOps.multAdd(quadraticCostGMatrix, p, tmpp); //    Z'  "-G*Zpz"
        Double Gp_grad = NormOps.normP1(tmpp);
//        System.out.println("Gp_grad: G p = -gradient0 "+Gp_grad);
      }
      //xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
      
      return stepType;
      
   }
   private void getLagrangeMultipliers(DenseMatrix64F lagrangeMultipliers)
   {
      
      /* Lagrange multipliers
       * solving A' lambda = gradient0 + G stepDirection 
       * 
       * methodA
       * A' lambda = QR lambda = gradient +G stepDirection
       * lambda = R(1:nactive) \ Q(1:nactive)' (gradient+stepDirection)
       * 
       * methodB
       * Y'A' lambda =  Y'(gradient + G stepDirection)
       * lambda = 
       */
      DenseMatrix64F tmpx = new DenseMatrix64F(numberOfVariablesToSolve,1);
//      tmpx.set(gradient0);
        tmpx.set(quadraticCostFVector);
        MatrixTools.multAddAllowEmptyMatrix(quadraticCostGMatrix, x, tmpx);
//      if(false)
//      {;
        QRPDecomposition<DenseMatrix64F> qrpActiveSet = DecompositionFactory.qrp(1, 1);
        DenseMatrix64F activeSetATranspose = CommonOps.transpose(activeSetA, null);
        qrpActiveSet.setSingularThreshold(eps);
        qrpActiveSet.decompose(activeSetATranspose);

        DenseMatrix64F R = qrpActiveSet.getR(null, true);
        DenseMatrix64F Qcompact = qrpActiveSet.getQ(null, true);
        CommonOps.multTransA(Qcompact, tmpx, lagrangeMultipliers);
        TriangularSolver.solveU(R.data,lagrangeMultipliers.data,R.numCols);
//      }
//      else
//      {
//         if(rank>0) //there is some constraint
//         {
//            //invAY
//            DenseMatrix64F AY = new DenseMatrix64F(rank,rank);
//            CommonOps.mult(activeSetA, activeSetRangeBasis, AY);
//            CommonOps.invert(AY);
//            CommonOps.multTransA(activeSetRangeBasis, tmpx, tmpr);
//            MatrixTools.multAllowEmptyMatrix(AY,tmpr, lagrangeMultipliers);
//         }
//      }
   }

   
   enum STEPTYPE {NEWTON, STEEPEST};
   class Step
   {
      Step()
      {
         direction= new DenseMatrix64F(numberOfVariablesToSolve,1);
      }
      int blockingConstraint;
      double size;
      double projectedStepIntoBlockingConstraint;
      DenseMatrix64F direction;
      STEPTYPE type;
   };
   
   private void findingBlockingConstraint(Step step)
   {
      DenseMatrix64F product=new DenseMatrix64F(linearInequalityConstraintA.numRows,1);
      CommonOps.mult(linearInequalityConstraintA, step.direction, product);
//      System.out.println("stepDirection*inequality"+product);

      DenseMatrix64F violation= new DenseMatrix64F(linearInequalityConstraintB);
      CommonOps.multAdd(-1, linearInequalityConstraintA, x, violation);
      double stepNorm = NormOps.normP2(step.direction);
      
      double minStep = Double.POSITIVE_INFINITY; // Double.MAX_VALUE;
      step.blockingConstraint=Integer.MIN_VALUE;
      step.size=Double.POSITIVE_INFINITY; 
      for(int i=0;i<linearInequalityConstraintA.numRows;i++)
      {
         if(product.get(i,0)>=eps*stepNorm) 
            // projected stepDirection on constraint plane normals is sufficiently large
            // so we won't take colinear constraints making the activeset matrix singular
         {
            double stepSizeToCurrentConstraint = violation.get(i,0)/(product.get(i,0)+eps);
            if(stepSizeToCurrentConstraint> -eps && stepSizeToCurrentConstraint < minStep) //a_i: consider constraint "blocking" us and the smaller one.
            {
               step.blockingConstraint=i;
               step.projectedStepIntoBlockingConstraint = product.get(i,0);
               minStep = step.size = stepSizeToCurrentConstraint;
            }
         }
      }
      
   }

   public DenseMatrix64F solve(DenseMatrix64F x0)
   {
      setZeroSizeMatrixForNullFields();
      if(x0==null)
         x0=findFeasiblePoint();
      
      setAndAssertCorrectNumberOfVariablesToSolve(x0.numRows);
      initializeActiveSet();

      x=new DenseMatrix64F(x0);
      
      int maximumIteration=2000;
      int iteration=0;
      Step step = new Step();
      
      DenseMatrix64F finalStep = new DenseMatrix64F(numberOfVariablesToSolve,1);
      DenseMatrix64F gradient = new DenseMatrix64F(numberOfVariablesToSolve,1);
      
      while(true)
      {
           logger.info("iter "+ iteration + " ----------");

           logger.info("Obj= "+ getObjectiveCost(x));
//         System.out.println("------------------");

        DenseMatrix64F activeSetLagrangeMultipliers = new DenseMatrix64F(activeSetA.numRows,1);
        step.type=calculateStepDirection(step.direction);
        logger.info("StepDirectionSize="+NormOps.normP2(step.direction));
        if(NormOps.normP2(step.direction)<eps)
        {
           getLagrangeMultipliers(activeSetLagrangeMultipliers);
           int maxIndex = MatrixTools.findMaxElementIndex(activeSetLagrangeMultipliers.data, activeEqualitySize, activeSetLagrangeMultipliers.numRows);
           double maxInequalityMultiplier = maxIndex<0?Double.NEGATIVE_INFINITY:activeSetLagrangeMultipliers.get(maxIndex,0);
           if(maxInequalityMultiplier<=0)
           {
                 logger.info("Optimum reached");
                 return x;
           }
           else
           {
              logger.info("remove constraint "+ maxIndex + " from activeset (" + (maxIndex-activeEqualitySize) +" from active inequality constraint)");
              removeConstraintFromActiveSet(maxIndex); 
           }
              
        }
        else //step-or-not
        {
          findingBlockingConstraint(step);
          packGradient(gradient);
          if(step.type==STEPTYPE.NEWTON)
          {
             if (step.size>1.0)
             {
//              assert(step.blockingConstraint<0);
              finalStep.set(step.direction);
              logger.info("make newton step (unblocked), norm="+NormOps.normP2(step.direction));
             }
             else
             {
              CommonOps.scale(step.size, step.direction, finalStep);
              int newConstraint= addConstraintToActiveSet(step.blockingConstraint);
              logger.info("make blocked (Newton) step, adding inequality constraint "+ step.blockingConstraint +" into activeset "+ newConstraint + " stepDistanceIntoConstraint " + step.projectedStepIntoBlockingConstraint);
             }
          }
          else if(step.type==STEPTYPE.STEEPEST)
          {
                double minStep=0,maxStep=0,midStep=0;
                final double bigStepSize=1e16;
                if(step.blockingConstraint<0)
                {
                   maxStep=bigStepSize; //non-blocking, default bigsteps
                }
                else
                {
                   maxStep=step.size;
                }
                
                
                for(int counter=0; counter<=100 && minStep<(maxStep-eps/1000); counter++)
                { 
                   double minObj,maxObj,midObj;
                   midStep = (minStep+maxStep)/2.0;
                   CommonOps.scale(minStep, step.direction, finalStep);
                   minObj = getObjectiveCost(finalStep);

                   CommonOps.scale(midStep, step.direction, finalStep);
                   midObj = getObjectiveCost(finalStep);

                   CommonOps.scale(maxStep, step.direction, finalStep);
                   maxObj = getObjectiveCost(finalStep);
//                   System.out.println("min="+minStep + " midStep=" + midStep+ " maxStep="+maxStep);
                   
                   if(minObj > midObj)
                   {
                      if(midObj > maxObj)
                         minStep = midStep;
                      else
                      {
                         minStep=(minStep+midStep)/2;
                         maxStep=(maxStep+midStep)/2;
                      }
                   }
                   else
                   {
                      maxStep=midStep;
                   }
                   
                   if(counter>1000)
                      throw new RuntimeException("Line search failed, exeeding maximum iteration");
                }
                

                if((maxStep-bigStepSize>-eps)) //maxstep == bigStep (unbounded)
                {
                  throw new RuntimeException("Unbounded Solution");
                }
                else if(maxStep > (step.size-eps)) //maxStep==step.size (bounded at stepsize)
                {
                   int newRowInActiveSet = addConstraintToActiveSet(step.blockingConstraint);
                   logger.info("make steepest-step until hitting blocking inequality " + step.blockingConstraint + " ( " + newRowInActiveSet + " in activeSet)");
                }
                else if(maxStep<eps && step.blockingConstraint<0) //maxStep=zero, unbounded step but not going anyhere.
                {
                   System.out.println("unbounded step too small, local optimum");
                   return x;
                }
                else //make the searched minimum objective-value step
                {
                   System.out.println("unbounded steepest descent back-track line search alpha="+maxStep);
                }
                
          }
          else
          {
             throw new RuntimeException("Unknow step type!");
          } 
          
          DenseMatrix64F expectedDescentMatrix = new DenseMatrix64F(1,1);
          CommonOps.multTransA(finalStep, gradient, expectedDescentMatrix);
          double expectedDescent = expectedDescentMatrix.get(0);
          logger.info("Expected Desecent " + expectedDescent);
          if(expectedDescent > 0)
          {
             logger.info("step gain too small");
             return x;
          }
          

          CommonOps.add(x, finalStep, x);
        } //step or not

        if(iteration>maximumIteration)
        {
           throw new RuntimeException("Maximum Iteration Reached");
        }

        iteration++;
      }
      
   }

   public DenseMatrix64F getSolution()
   {
      return x;
   }
   

   
   @Override
   public double[] solve()
   {
      solve();
      return getSolution().getData();
   }
}
