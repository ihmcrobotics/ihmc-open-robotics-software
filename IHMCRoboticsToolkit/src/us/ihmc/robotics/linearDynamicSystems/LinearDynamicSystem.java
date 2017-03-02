package us.ihmc.robotics.linearDynamicSystems;

import java.util.ArrayList;
//~--- JDK imports ------------------------------------------------------------

//~--- non-JDK imports --------------------------------------------------------

import Jama.Matrix;
import us.ihmc.robotics.dataStructures.Polynomial;

public class LinearDynamicSystem {
    private final Matrix                 matrixA;
    private Matrix                       matrixB, matrixC, matrixD;
    private final TransferFunctionMatrix sIMinusAInverse;

    public LinearDynamicSystem(Matrix matrixA, Matrix matrixB, Matrix matrixC, Matrix matrixD) {
        if (matrixA == null) {
            throw new RuntimeException("matrixA must be defined. B,C,D can be null");
        }

        int order = matrixA.getRowDimension();

        if (matrixA.getColumnDimension() != order) {
            throw new RuntimeException("matrixA must be square!");
        }

        this.matrixA = new Matrix(matrixA.getArrayCopy());
        setMatrixB(matrixB);
        setMatrixC(matrixC);
        setMatrixD(matrixD);
        sIMinusAInverse = computeSIMinusAInverse();
    }

    public Matrix getMatrixA() {
        return new Matrix(matrixA.getArrayCopy());
    }

    public Matrix getMatrixB() {
        return new Matrix(matrixB.getArrayCopy());
    }

    public Matrix getMatrixC() {
        return new Matrix(matrixC.getArrayCopy());
    }

    public Matrix getMatrixD() {
        return new Matrix(matrixD.getArrayCopy());
    }

    public LinearDynamicSystem addFullStateFeedback(Matrix matrixK) {
        if (matrixB == null) {
            throw new RuntimeException("Matrix B must not be null for addFullStateFeedback!");
        }

        Matrix newMatrixA = matrixA.plus(matrixB.times(matrixK.times(-1.0)));
        Matrix newMatrixB = matrixB.copy();
        Matrix newMatrixC = null,
               newMatrixD = null;

        if (matrixC != null) {
            newMatrixC = matrixC;

            if (matrixD != null) {
                newMatrixC = matrixC.plus(matrixD.times(matrixK.times(-1.0)));
                newMatrixD = matrixD.copy();
            }
        }

        return new LinearDynamicSystem(newMatrixA, newMatrixB, newMatrixC, newMatrixD);
    }

    public LinearDynamicSystem addOutputStateFeedback(Matrix matrixK) {
        return addOutputStateFeedback(matrixK, null);
    }

    public LinearDynamicSystem addOutputStateFeedback(Matrix matrixK, Matrix matrixFF) {
        if (matrixB == null) {
            throw new RuntimeException("Matrix B must not be null for addOutputStateFeedback!");
        }

        if (matrixD != null) {
            throw new RuntimeException("Matrix D must be null for addOutputStateFeedback!");
        }

        if (matrixC == null) {
            throw new RuntimeException("Matrix C must not be null for addOutputStateFeedback!");
        }

        Matrix newMatrixA = matrixA.plus(matrixB.times(matrixK.times(-1.0)).times(matrixC));
        Matrix newMatrixB = matrixB.copy();
        Matrix newMatrixC = matrixC.copy();

        if (matrixFF != null) {
            newMatrixB = newMatrixB.times(matrixFF);
        }

        return new LinearDynamicSystem(newMatrixA, newMatrixB, newMatrixC, null);
    }

    public TransferFunctionMatrix getTransferFunctionMatrix() {

        // Returns C(sI-A)^(-1)B + D.
        // If any of C, B, or D are null, then just skip the multiplication.
        TransferFunctionMatrix ret = sIMinusAInverse;

        if (matrixC != null) {
            ret = ret.preMultiply(matrixC);
        }

        if (matrixB != null) {
            ret = ret.times(matrixB);
        }

        if (matrixD != null) {
            ret = ret.plus(matrixD);
        }

        return ret;
    }

    public void setMatrixB(Matrix matrixB) {
        if (matrixB != null) {
            this.matrixB = new Matrix(matrixB.getArrayCopy());
        }
    }

    public void setMatrixC(Matrix matrixC) {
        if (matrixC != null) {
            this.matrixC = new Matrix(matrixC.getArrayCopy());
        }
    }

    public void setMatrixD(Matrix matrixD) {
        if (matrixD != null) {
            this.matrixD = new Matrix(matrixD.getArrayCopy());
        }
    }

    private TransferFunctionMatrix computeSIMinusAInverse() {
        return computeSIMinusAInverseUsingCofactors();

        // return computeSIMinusAInverseUsingEigenvalueDecomposition();
    }

    private TransferFunctionMatrix computeSIMinusAInverseUsingCofactors() {
        int              order                  = matrixA.getColumnDimension();
        PolynomialMatrix sIMinusA               = PolynomialMatrix.constructSIMinusA(matrixA);
        Polynomial       characteristicEquation = sIMinusA.computeDeterminant();
        Polynomial[][]   numerators             = new Polynomial[order][order];

        for (int i = 0; i < order; i++) {
            for (int j = 0; j < order; j++) {
                numerators[j][i] = sIMinusA.computeCofactor(i, j);
            }
        }

        return new TransferFunctionMatrix(numerators, characteristicEquation);
    }

    @SuppressWarnings("unused")
    private TransferFunctionMatrix computeSIMinusAInverseUsingEigenvalueDecomposition() {
        EigenvalueDecomposer            eigenvalueDecomposer  = new EigenvalueDecomposer(matrixA);
        ArrayList<SingleRealMode>       realModes             = eigenvalueDecomposer.getRealModes();
        ArrayList<ComplexConjugateMode> complexConjugateModes = eigenvalueDecomposer.getComplexConjugateModes();
        TransferFunctionMatrix          ret                   = null;

        for (SingleRealMode realMode : realModes) {
            TransferFunctionMatrix realModeMatrix = realMode.constructTransferFunctionMatrix();

            if (ret == null) {
                ret = realModeMatrix;
            } else {
                ret = ret.plus(realModeMatrix);
            }
        }

        for (ComplexConjugateMode complexConjugateMode : complexConjugateModes) {
            TransferFunctionMatrix complexConjugateModeMatrix = complexConjugateMode.constructTransferFunctionMatrix();

            if (ret == null) {
                ret = complexConjugateModeMatrix;
            } else {
                ret = ret.plus(complexConjugateModeMatrix);
            }
        }

        return ret;
    }

    public double[][] simulateInitialConditions(double[] initialConditions, double stepSize, int numTicks) {
        int order = matrixA.getRowDimension();

        if (initialConditions.length != order) {
            throw new RuntimeException("initialConditions.length != order");
        }

        // Just use Euler integrations for now:
        double[][] ret   = new double[numTicks][order];
        Matrix     state = new Matrix(order, 1);

        copyArray(initialConditions, state);

        for (int i = 0; i < numTicks; i++) {
            copyArray(state, ret[i]);

            Matrix aTimesX              = matrixA.times(state);
            Matrix aTimesXTimesStepSize = aTimesX.times(stepSize);

            state = state.plus(aTimesXTimesStepSize);
        }

        return ret;
    }

    @SuppressWarnings("unused")
    private void copyArray(double[] in, double[] out) {
        for (int i = 0; i < in.length; i++) {
            out[i] = in[i];
        }
    }

    private void copyArray(Matrix vectorIn, double[] out) {
        for (int i = 0; i < vectorIn.getRowDimension(); i++) {
            out[i] = vectorIn.get(i, 0);
        }
    }

    private void copyArray(double[] in, Matrix vectorOut) {
        for (int i = 0; i < vectorOut.getRowDimension(); i++) {
            vectorOut.set(i, 0, in[i]);
        }
    }
}
