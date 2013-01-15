package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiGroundReactionWrenchDistributor
{
   private static final int VECTOR3D_LENGTH = 3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable wk = new DoubleYoVariable("wk", registry);    // TODO: better name
   private final ReferenceFrame centerOfMassFrame;

   private final List<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   private final HashMap<PlaneContactState, Double> coefficientsOfFriction = new HashMap<PlaneContactState, Double>();
   private final HashMap<PlaneContactState, FrameVector> forces = new HashMap<PlaneContactState, FrameVector>();
   private final HashMap<PlaneContactState, FramePoint2d> centersOfPressure = new HashMap<PlaneContactState, FramePoint2d>();
   private final HashMap<PlaneContactState, Double> normalTorques = new HashMap<PlaneContactState, Double>();

   // TODO: pass into constructor:
   private final FrameVector gravitationalForce;
   private final int nSupportVectors;

   private final DenseMatrix64F phi;
   private final DenseMatrix64F beta;
   private final DenseMatrix64F delta;
   private final DenseMatrix64F xi;

   public LeeGoswamiGroundReactionWrenchDistributor(ReferenceFrame centerOfMassFrame, FrameVector gravitationalAcceleration, double mass, int nSupportVectors,
           YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      this.gravitationalForce = new FrameVector(gravitationalAcceleration);
      gravitationalForce.scale(mass);
      gravitationalForce.changeFrame(centerOfMassFrame);

      this.nSupportVectors = nSupportVectors;
      int defaultNumberOfContacts = 2;    // reshape stuff later
      phi = new DenseMatrix64F(Momentum.SIZE, defaultNumberOfContacts * nSupportVectors);
      beta = new DenseMatrix64F(VECTOR3D_LENGTH, defaultNumberOfContacts * nSupportVectors);
      delta = new DenseMatrix64F(VECTOR3D_LENGTH, defaultNumberOfContacts * nSupportVectors);
      xi = new DenseMatrix64F(Momentum.SIZE, 1);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      contactStates.clear();
      coefficientsOfFriction.clear();
      forces.clear();
      centersOfPressure.clear();
      normalTorques.clear();
   }

   public void addContact(PlaneContactState contactState, double coefficientOfFriction)
   {
      // TODO: reshape matrices, update hashmaps
   }

   public void solve(SpatialForceVector groundReactionWrench)
   {
      int nContacts = coefficientsOfFriction.size();

      // phi
      int startColumn = 0;
      for (int i = 0; i < nContacts; i++)
      {
         PlaneContactState contactState = contactStates.get(i);
         double mu = coefficientsOfFriction.get(contactState);

         // update beta
         DenseMatrix64F betaBlock = computeBetaBlock(contactState, mu, nSupportVectors, centerOfMassFrame);
         CommonOps.insert(betaBlock, beta, 0, startColumn);

         // update delta TODO
         DenseMatrix64F deltaBlock = computeDeltaBlock(contactState, betaBlock, centerOfMassFrame);
         CommonOps.insert(deltaBlock, delta, 0, startColumn);

         startColumn += betaBlock.getNumCols();
      }

      int startRow = 0;
      CommonOps.insert(beta, phi, startRow, 0);
      startRow += beta.getNumRows();

      MatrixTools.setMatrixBlock(delta, 0, 0, phi, startRow, 0, delta.getNumRows(), delta.getNumCols(), wk.getDoubleValue());
      startRow += delta.getNumRows();

      // TODO: epsilonf * eye(nFeet * nSupportVectors)
   }

   public FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      return centersOfPressure.get(contactState);
   }

   public double getNormalTorque(PlaneContactState contactState)
   {
      return normalTorques.get(contactState);
   }

   public FrameVector getForce(PlaneContactState contactState)
   {
      return forces.get(contactState);
   }

   // TODO: garbage
   private static DenseMatrix64F computeBetaBlock(PlaneContactState contactState, double mu, int nSupportVectors, ReferenceFrame centerOfMassFrame)
   {
      DenseMatrix64F betaBlock = new DenseMatrix64F(VECTOR3D_LENGTH, nSupportVectors);
      double angleIncrement = 2.0 * Math.PI / nSupportVectors;
      for (int i = 0; i < nSupportVectors; i++)
      {
         double angle = i * angleIncrement;
         double x = mu * Math.cos(angle);
         double y = mu * Math.sin(angle);
         double z = 1.0;
         FrameVector supportVector = new FrameVector(contactState.getPlaneFrame(), x, y, z);
         supportVector.changeFrame(centerOfMassFrame);

         MatrixTools.setDenseMatrixFromTuple3d(betaBlock, supportVector.getVector(), 0, i);
      }

      return betaBlock;
   }

   // TODO: garbage
   private static DenseMatrix64F computeDeltaBlock(PlaneContactState contactState, DenseMatrix64F betaBlock, ReferenceFrame centerOfMassFrame)
   {
      FramePoint ankle = new FramePoint(contactState.getBodyFrame());
      ankle.changeFrame(centerOfMassFrame);

      DenseMatrix64F skew = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      MatrixTools.vectorToSkewSymmetricMatrix(skew, ankle.getPoint());

      DenseMatrix64F deltaBlock = new DenseMatrix64F(VECTOR3D_LENGTH, betaBlock.getNumCols());
      CommonOps.mult(skew, betaBlock, deltaBlock);

      return deltaBlock;
   }
}
