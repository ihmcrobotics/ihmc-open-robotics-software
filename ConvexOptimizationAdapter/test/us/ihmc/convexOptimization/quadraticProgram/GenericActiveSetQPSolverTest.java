package us.ihmc.convexOptimization.quadraticProgram;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.Map;
import java.util.Random;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.QRPDecomposition;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixIO;
import org.ejml.ops.NormOps;
import org.ejml.ops.RandomMatrices;
import org.ejml.ops.SingularOps;
import org.junit.Test;
import org.yaml.snakeyaml.Yaml;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.testing.JUnitTools;
public class GenericActiveSetQPSolverTest
{
   /** 
    * EJML can't handle these cases earlier before August 2014
    * So we put these test cases here to check 
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000) 
   public void choleskyDecompositionAccuracy()
   {
      /**
    pg =

      0.8859500178757540   0.2588001949262530
      0.2588001949262530   0.0755996834386430

    octave:13> sd
    sd =

      -1.38777878078145e-17
      0.00000000000000e+00

    octave:14> [r p]=chol(pg);
    octave:15> r'\r\sd
    ans =

      4.41160312094377e-11
      -1.51022339626802e-10

       */
      double[][] pgArray= new double[][]{
                  { 0.885950017875754,  0.258800194926253},
                  {0.258800194926253,   0.075599683438643}};
            
      double[][] sdArray = new double[][]{ {-0.138777878078145e-16},{0.0} };
           

      DenseMatrix64F pg=new DenseMatrix64F(pgArray);
      DenseMatrix64F sd=new DenseMatrix64F(sdArray);
      
//      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.pseudoInverse(false);
      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.symmPosDef(pg.numRows);
      DenseMatrix64F X = new DenseMatrix64F(sd.numRows,1);
      boolean setASuccess=solver.setA(pg);
      solver.solve(sd, X);
      System.out.println("setSucc="+setASuccess + "\nX="+X);
      
      DenseMatrix64F violation =new DenseMatrix64F(pg.numRows,1);
      CommonOps.mult(pg, X, violation);
      System.out.println("Violation="+violation.get(0));
      
      RealMatrix A=new Array2DRowRealMatrix(pgArray);
      CholeskyDecomposition dec = new CholeskyDecomposition(A,1e-200,1e-200);
      RealMatrix b = new Array2DRowRealMatrix(sdArray);
      RealMatrix sol=dec.getSolver().solve(b);
      

      System.out.println("common-math sol:"+sol);
      System.out.println("violation:"+A.multiply(sol).subtract(b));
      
      
      // b=rand(2,1);a=rand(2,10);x=quadprog(a'*a,a'*b);0.5*x'*a'*a*x+b'*a*x+0.5*b'*b,qp=a*x+b, backSlash=a*(a\b)-b,[u s v]=svd(a); x=v(:,1:size(s,1))*diag([1./diag(s)])*u'*b;svdx=a*x-b
         
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void zeroMatrixSVD(){
      DenseMatrix64F []testMatrices = new DenseMatrix64F[]{
            RandomMatrices.createRandom(2, 2, new Random()),
            new DenseMatrix64F(2,2),
            new DenseMatrix64F(new double[][]{{1,1},{0,0}})
      };
      for(int i=0;i<testMatrices.length;i++)
      {
         DenseMatrix64F testMatrix = testMatrices[i];
          System.out.println("Matrix "+ i + testMatrix);
          SolvePseudoInverseSvd svdPseudoInverseSolver = new SolvePseudoInverseSvd(testMatrix.numRows, testMatrix.numCols);
          boolean setAResult=svdPseudoInverseSolver.setA(new DenseMatrix64F(testMatrix));
          int rank=SingularOps.rank(svdPseudoInverseSolver.getDecomposer(), 1e-10);
          System.out.println("Singular Values: "+ Arrays.toString(svdPseudoInverseSolver.getDecomposer().getSingularValues()));
          System.out.println("setA="+setAResult + " rank="+ rank);
          System.out.println("----------------------------------------------------------------------------------------------------");
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void errorSVD()
   {
      SingularValueDecomposition<DenseMatrix64F> dec = DecompositionFactory.svd(16, 16, true, true, true);
      DenseMatrix64F m = RandomMatrices.createRandom(16, 16, new Random(64));
      MatrixIO.print(System.out, m,"%.10e");
      dec.decompose(m);
      System.out.println("SVs="+Arrays.toString(dec.getSingularValues()));
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void errorZeroSizeMatrix()
   {
      DenseMatrix64F 
      a=new DenseMatrix64F(10,0),
      b=new DenseMatrix64F(0,1),
      c=new DenseMatrix64F(10,1);
      CommonOps.mult(a, b, c);
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void matrixRankFromQRDecomositionPivot()
   {
      double[][] AMatrix = new double[][]
            {
               {1,2},
               {2,4},
            };
      
      QRPDecomposition<DenseMatrix64F> qrp = DecompositionFactory.qrp(1, 1);
      qrp.setSingularThreshold(1e-10);
      qrp.decompose(new DenseMatrix64F(AMatrix));
      assertEquals(1, qrp.getRank());
   }

   /** Basic QP Solver function test
    * 
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void consistentColinearEqualityConstraints()
   {
      double[][] AMatrix = new double[][]
            {
               {1,2},
               {2,4},
               {3,5}
            };
      
      double[] bVector = new double[]
            {
               1, 
               2,
               3
            };
      
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setLinearEqualityConstraints(AMatrix, bVector);
      assertTrue(solver.checkEqualityConstraintFeasibility());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void inconsistentColinearEqualityConstraints()
   {
      double[][] AMatrix = new double[][]
            {
               {1,2},
               {2,4},
               {3,5}
            };
      
      double[] bVector = new double[]
            {
               1, 
               4,
               3,
            };
      
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setLinearEqualityConstraints(AMatrix, bVector);
      assertFalse(solver.checkEqualityConstraintFeasibility());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void findBestIndependentEqualityConstraints()
   {
      double[][] AMatrix = new double[][]
            {
               {1,2},
               {3,5},
               {2,4},
               {4,8},
               {6,10},
            };
      double[] BVector = new double[]
            {1,2,2,4,4};
      
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setLinearEqualityConstraints(AMatrix, BVector);
      int[] selectActiveSetIndexes=solver.initializeActiveSet();
      Arrays.sort(selectActiveSetIndexes);
      int[] expected= new int[]{3,4};
      assertArrayEquals(expected, selectActiveSetIndexes);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void initializeActivesetTest() throws Exception
   {
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setLinearEqualityConstraints(
      new double[][]
            {
               {1,2},
            } ,
      new double[]
            {
               3,
            }
      );
 
      solver.initializeActiveSet();
      assertTrue(Arrays.equals(solver.activeSetA.data, new double[]{1,2}));
      assertTrue(Arrays.equals(solver.activeSetB.data, new double[]{3}));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testLinearProblemWithInitialSeedNonConstraints() throws Exception
   {
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setQuadraticCostFunction(
            new double[][]{
                  {1, 0},
                  {0, 1}},
            new double[]
                  {0,0},
                  0);
      DenseMatrix64F solution=solver.solve(MatrixTools.createVector(new double[] {1,1}));
      JUnitTools.assertDoubleArrayEquals(new double[]{0,0}, solution.data, 1e-10);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testLinearProblemWithInitialSeedEqualityConstraint() throws Exception
   {
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setQuadraticCostFunction(
            new double[][]{
                  {1, 0},
                  {0, 1}},
            new double[]
                  {0,0},
                  0);
      solver.setLinearEqualityConstraints(
      new double[][]
            {
               {1,2},
            },
      new double[]
            {
               3,
            }
      );

      DenseMatrix64F solution=solver.solve(MatrixTools.createVector(new double[] {1,1}));
      JUnitTools.assertDoubleArrayEquals(new double[]{0.6,  1.2}, solution.data, 1e-10);
   } 

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testLinearProblemWithInitialSeedInequalityConstraint() throws Exception
   {
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setQuadraticCostFunction(
            new double[][]{
                  {1, 0},
                  {0, 1}},
            new double[]
                  {0,0},
                  0);
      solver.setLinearEqualityConstraints(
      new double[][]
            {
               {1,2},
            },
      new double[]
            {
               3,
            }
      );
      
      solver.setLinearInequalityConstraints(
            new double[][]{
                  {-1.0, 0.0}
            },
            new double[]{
                  -1.5
            });
      DenseMatrix64F solution=solver.solve(MatrixTools.createVector(new double[] {3,0}));
      JUnitTools.assertDoubleArrayEquals(new double[]{1.5,  0.75}, solution.data, 1e-10);
   } 

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testFindFeasiblePointTest() throws Exception
   {
      GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
      solver.setLinearEqualityConstraints(
      new double[][]
            {
               {1,2},
            },
      new double[]
            {
               3,
            }
      );
      
      solver.setLinearInequalityConstraints(
            new double[][]
            {
                  {-1.0, 0.0},
                  {0.0, -1.0}
            },
            new double[]
            {
                  -1,
                  -1,
            }
      );
      solver.displayProblem();
      DenseMatrix64F xopt = solver.findFeasiblePoint();

      // complies to equality constraint - zero violation
      DenseMatrix64F equalityConstraintViolation= new DenseMatrix64F(solver.linearEqualityConstraintB);
      CommonOps.multAdd(-1,solver.linearEqualityConstraintA, xopt, equalityConstraintViolation);
      JUnitTools.assertMatrixEqualsZero(equalityConstraintViolation, 1e-10);

      // complies to inequality constraint - violation>0
      DenseMatrix64F inequalityConstraintViolation= new DenseMatrix64F(solver.linearInequalityConstraintB);
      CommonOps.multAdd(-1,solver.linearInequalityConstraintA, xopt, inequalityConstraintViolation);
      for(int i=0;i<inequalityConstraintViolation.numRows;i++)
      {
         double v = inequalityConstraintViolation.get(i,0);
         System.out.println(v);
         assertTrue(v>=-1e-9);
      }
      
   }

   /**
    * Randomized Test
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000) 
   public void testRandomQuadraticCostFunction()
   {

      Random random = new Random(0L);
      int numberDimension= 20;
      int rank=numberDimension;
      for(int i=0;i<100;i++)
      {
         random.setSeed(i);
         System.out.println("*** test case "+ i);
         //We solve min_x (Ax+b)'(Ax+b)
         DenseMatrix64F costQuadGMatrix = new DenseMatrix64F(numberDimension,numberDimension);
         DenseMatrix64F randomA= RandomMatrices.createRandom(rank, numberDimension, random);
         CommonOps.multTransA(randomA, randomA, costQuadGMatrix);
         
         DenseMatrix64F randomB= RandomMatrices.createRandom(rank, 1, random);
         DenseMatrix64F costQuadfVector = new DenseMatrix64F(numberDimension,1);
         CommonOps.multTransA(randomA, randomB, costQuadfVector);
         

         DenseMatrix64F costQuadScalar=new DenseMatrix64F(1,1);
         CommonOps.multTransA(randomB, randomB, costQuadScalar);

         GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
         solver.setThreshold(1e-7);
         solver.setQuadraticCostFunction(costQuadGMatrix, costQuadfVector, costQuadScalar.get(0,0));
         
         try{
//            System.out.println("A "+random1);
//            System.out.println("b "+random2);
            solver.solve(null);
         }
         catch(RuntimeException e)
         {
            e.printStackTrace(System.err);
            System.err.println(e.getMessage());
            System.err.println("Warning, maximum iteration reached... result may not be accurate");
         }

         DenseMatrix64F residual = new DenseMatrix64F(randomB);
         CommonOps.multAdd(randomA,solver.getSolution(),residual);
         double resNorm=NormOps.normPInf(residual);
         System.out.println("norm:"+resNorm);
         assertEquals("Result norm should be zero", 0, resNorm, 1e-9);
         
         
//         //Reference Solution solving Ax+b=0;
//         LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.pseudoInverse(true);
//         DenseMatrix64F minusRandom2 =  new DenseMatrix64F(random2);
//         CommonOps.scale(-1, minusRandom2);
//         DenseMatrix64F sol2= new DenseMatrix64F(random1.numCols,1);
//         linearSolver.setA(random1);
//         linearSolver.solve(minusRandom2, sol2);
//
//         DenseMatrix64F res2= new DenseMatrix64F(random2);
//         CommonOps.multAdd(random1, sol2, res2);
//         System.out.println("norm:"+NormOps.normPInf(res2));
      }
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000) 
   public void testSingularQuadraticCostFunction()
   {

      Random random = new Random(0L);
      int numberDimension= 100;
      int rank=numberDimension/2;
      for(int i=0;i<10;i++)
      {
         System.out.println("*** test case "+ i);
         //We solve min_x (Ax+b)'(Ax+b)
         DenseMatrix64F costQuadGMatrix = new DenseMatrix64F(numberDimension,numberDimension);
         DenseMatrix64F randomA= RandomMatrices.createRandom(rank, numberDimension, random);
         CommonOps.multTransA(randomA, randomA, costQuadGMatrix);
         
         DenseMatrix64F randomB= RandomMatrices.createRandom(rank, 1, random);
         DenseMatrix64F costQuadfVector = new DenseMatrix64F(numberDimension,1);
         CommonOps.multTransA(randomA, randomB, costQuadfVector);
         

         DenseMatrix64F costQuadScalar=new DenseMatrix64F(1,1);
         CommonOps.multTransA(randomB, randomB, costQuadScalar);

         GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
         solver.setQuadraticCostFunction(costQuadGMatrix, costQuadfVector, costQuadScalar.get(0,0));
         
         try{
//            System.out.println("A "+random1);
//            System.out.println("b "+random2);
            solver.solve(null);
         }
         catch(RuntimeException e)
         {
            e.printStackTrace(System.err);
            System.err.println(e.getMessage());
            System.err.println("Warning, maximum iteration reached... result may not be accurate");
         }
         
         double objVal=solver.getObjectiveCost(solver.getSolution());
         System.out.println("obj:="+ objVal);
         assertEquals("The objective value should be zero", 0, objVal, 1e-10);

         DenseMatrix64F residual = new DenseMatrix64F(randomB);
         CommonOps.multAdd(randomA,solver.getSolution(),residual);
         double resNorm=NormOps.normPInf(residual);
         System.out.println("norm:"+resNorm);
         assertEquals("Result norm should be zero", 0, resNorm, 1e-10);
         
         //Reference Solution solving Ax+b=0;
//         LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.pseudoInverse(true);
//         DenseMatrix64F minusRandom2 =  new DenseMatrix64F(randomB);
//         CommonOps.scale(-1, minusRandom2);
//         DenseMatrix64F sol2= new DenseMatrix64F(randomA.numCols,1);
//         linearSolver.setA(randomA);
//         linearSolver.solve(minusRandom2, sol2);
//
//         DenseMatrix64F res2= new DenseMatrix64F(randomB);
//         CommonOps.multAdd(randomA, sol2, res2);
//         System.out.println("norm:"+NormOps.normPInf(res2));
      }
      
   }
   
   /**
    * Test problems download from a dataset names QPS 
    */
   private void testCaseFromQPS(String qpsFileName, GenericActiveSetQPSolver solver) throws FileNotFoundException
   {
        DenseMatrix64F beq=null ;
        DenseMatrix64F Aeq =null;
        DenseMatrix64F b  =null;
        DenseMatrix64F A  =null;
        DenseMatrix64F H  =null;
        DenseMatrix64F f  =null;
        DenseMatrix64F lb =null;
        DenseMatrix64F ub =null;
        DenseMatrix64F x =null; 
        Yaml yaml = new Yaml();
        
        String qpsPath="YamlQpProblems/";
        InputStream input = getClass().getClassLoader().getResourceAsStream(qpsPath+qpsFileName);
        Map<String, Object> object = (Map<String, Object>) yaml.load(input);
        
        beq=MatrixTools.yamlFieldToMatrix(null,"beq",object);
        Aeq=MatrixTools.yamlFieldToMatrix(null,"Aeq",object);
        A=MatrixTools.yamlFieldToMatrix(null,"A",object);
        b=MatrixTools.yamlFieldToMatrix(null,"b",object);
        H=MatrixTools.yamlFieldToMatrix(null,"H",object);
        f=MatrixTools.yamlFieldToMatrix(null,"f",object);
        lb=MatrixTools.yamlFieldToMatrix(null,"lb",object);
        ub=MatrixTools.yamlFieldToMatrix(null,"ub",object);
        x=MatrixTools.yamlFieldToMatrix(null,"X",object);

        CommonOps.scale(2.0, H);
        
        if(f==null)
           f=new DenseMatrix64F(H.numRows,1);
        solver.setQuadraticCostFunction(H, f, 0);
        
        if(Aeq!=null)
           solver.setLinearEqualityConstraints(Aeq, beq);
        
        int maxInequalityRows=solver.numberOfVariablesToSolve*2 +( A==null?0:A.numRows);
        DenseMatrix64F aggregatedInequalityMatrix = new DenseMatrix64F(maxInequalityRows, solver.numberOfVariablesToSolve);
        DenseMatrix64F aggregatedInequalityVector = new DenseMatrix64F(maxInequalityRows, 1);
        int offset=0;
        if(A!=null)
        {
           CommonOps.insert(A,aggregatedInequalityMatrix,0,0);
           CommonOps.insert(b, aggregatedInequalityVector, 0, 0);
           assert(A.numRows==b.numRows);
           offset+=A.numRows;
        }
        
        if(ub!=null)
        {
           for(int j=0;j<solver.numberOfVariablesToSolve;j++)
               aggregatedInequalityMatrix.set(j+offset,j,1);
           CommonOps.insert(ub, aggregatedInequalityVector, offset, 0);
           offset+=ub.numRows;
        }
        
        if(lb!=null)
        {
           for(int j=0;j<solver.numberOfVariablesToSolve;j++)
              aggregatedInequalityMatrix.set(j+offset,j,-1);
           CommonOps.scale(-1, lb);
           CommonOps.insert(lb, aggregatedInequalityVector, offset, 0);
           offset+=lb.numRows;
        }
        
        aggregatedInequalityMatrix.setNumRows(offset);
        aggregatedInequalityVector.setNumRows(offset);
        
        if(offset>0)
           solver.setLinearInequalityConstraints(aggregatedInequalityMatrix, aggregatedInequalityVector);
        
//        solver.displayProblem();
        solver.solve(null);
        DenseMatrix64F solution = solver.getSolution();
        CommonOps.subtract(solution, x, solution);
        System.out.println("File"+qpsFileName);
        double norm=NormOps.normP1(solution);
        System.out.println("diffNormToMatlabAnswer="+ norm);
//        assertTrue(norm<1.0f);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_TAME() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     testCaseFromQPS("TAME.yaml", solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_SimpleOneD() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("simple1DQpTest.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS35() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     testCaseFromQPS("HS35.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS21() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("HS21.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000) 
   public void testQPS_ZECEVVIC2() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("ZECEVIC2.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS35MOD() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("HS35MOD.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS76() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("HS76.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS51() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("HS51.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS52() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("HS52.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS53() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-8);
     testCaseFromQPS("HS53.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS268() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("HS268.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_LOTSCHD() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("LOTSCHD.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_HS118() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("HS118.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_DUALC2() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("DUALC2.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_DUALC5() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("DUALC5.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_DUAL4() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("DUAL4.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000) 
   public void testQPS_DUAL1() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("DUAL1.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_DUAL2() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("DUALC5.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000) 
   public void testQPS_QPCBLEND() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-5);
     testCaseFromQPS("QPCBLEND.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000) 
   public void testQPS_DUAL3() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-6);
     testCaseFromQPS("DUAL3.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000) 
   public void testQPS_DUALC1() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-5);
     testCaseFromQPS("DUALC1.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000) 
   public void testQPS_DUALC8() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     solver.setThreshold(1e-5);
     testCaseFromQPS("DUALC8.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.7)
   @Test(timeout = 30000) 
   public void testQPS_PRIMAL1() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     testCaseFromQPS("PRIMAL1.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 11.1)
   @Test(timeout = 55000) 
   public void testQPS_PRIMALC2() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     testCaseFromQPS("PRIMALC2.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 11.7)
   @Test(timeout = 59000) 
   public void testQPS_PRIMALC1() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     testCaseFromQPS("PRIMALC1.yaml",solver); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 26.0)
   @Test(timeout = 130000) 
   public void testQPS_PRIMALC5() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     testCaseFromQPS("PRIMALC5.yaml",solver); 
   }
   
   //take about 200s to solve
   //@Test(timeout=300000)
   public void testQPS_PRIMALC8() throws FileNotFoundException
   {
     GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
     testCaseFromQPS("PRIMALC8.yaml",solver); 
   }
   
   
// @Test(timeout=300000) //constraint in/out
 public void testQPS_CVXQP3_S() throws FileNotFoundException
 {
   GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
   solver.setThreshold(1e-6);
   testCaseFromQPS("CVXQP3_S.yaml",solver); 
 }

// @Test(timeout=300000) //end-point gradient descent stock
 public void testQPS_QPCBOEI1() throws FileNotFoundException
 {
   GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
   testCaseFromQPS("QPCBOEI1.yaml",solver); 
 }
// @Test(timeout=300000) //end point jumping
 public void testQPS_QPCBOEI2() throws FileNotFoundException
 {
   GenericActiveSetQPSolver solver = new GenericActiveSetQPSolver();
   testCaseFromQPS("QPCBOEI2.yaml",solver); 
 } 
 
 

   
}
