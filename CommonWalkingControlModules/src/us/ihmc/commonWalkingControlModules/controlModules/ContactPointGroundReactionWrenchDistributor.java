package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ContactPointWrenchOptimizerNative;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ContactPointGroundReactionWrenchDistributor implements GroundReactionWrenchDistributorInterface
{
   private final ReferenceFrame centerOfMassFrame;

   private final ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   private final HashMap<PlaneContactState, Double> coefficientsOfFriction = new HashMap<PlaneContactState, Double>();

   private final LinkedHashMap<PlaneContactState, FrameVector> forces = new LinkedHashMap<PlaneContactState, FrameVector>();
   private final LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure = new LinkedHashMap<PlaneContactState, FramePoint2d>();
   private final LinkedHashMap<PlaneContactState, Double> normalTorques = new LinkedHashMap<PlaneContactState, Double>();

   private final double[] diagonalCWeights = new double[Wrench.SIZE];
   private double epsilonRho;
   
   private final int rhoDimension = ContactPointWrenchOptimizerNative.NUMBER_OF_SUPPORT_VECTORS * ContactPointWrenchOptimizerNative.NUMBER_OF_POINTS_PER_CONTACT * ContactPointWrenchOptimizerNative.MAX_NUMBER_OF_CONTACTS;
  
   private final ArrayList<FrameVector> normalizedSupportVectors = new ArrayList<FrameVector>(ContactPointWrenchOptimizerNative.NUMBER_OF_SUPPORT_VECTORS);
   
   private final DenseMatrix64F aMatrix = new DenseMatrix64F(Wrench.SIZE, rhoDimension);
   private final DenseMatrix64F desiredWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F normalForceSelectorBMatrix = new DenseMatrix64F(ContactPointWrenchOptimizerNative.MAX_NUMBER_OF_CONTACTS, rhoDimension);   
   private final double[] minimumNormalForces = new double[ContactPointWrenchOptimizerNative.MAX_NUMBER_OF_CONTACTS];

   private final double[] aMatrixAsDoubleArray = new double[aMatrix.getNumElements()];
   private final double[] normalForceSelectorBMatrixAsDoubleArray = new double[normalForceSelectorBMatrix.getNumElements()];
   private final double[] desiredWrenchAsDoubleArray = new double[desiredWrench.getNumElements()];


   private final ContactPointWrenchOptimizerNative contactPointWrenchOptimizerNative = new ContactPointWrenchOptimizerNative();

   public ContactPointGroundReactionWrenchDistributor(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
   }

   public void setWeights(double[] diagonalCWeights, double epsilonRho)
   {
      for (int i=0; i<diagonalCWeights.length; i++)
      {
         this.diagonalCWeights[i] = diagonalCWeights[i];
      }
        
      this.epsilonRho = epsilonRho;
   }
   
   public void setMinimumNormalForces(double[] minimumNormalForces)
   {
      for (int i=0; i<minimumNormalForces.length; i++)
      {
         this.minimumNormalForces[i] = minimumNormalForces[i];
      }
   }

   public void reset()
   {
      // TODO: inefficient; should hang on to a bunch of temporary objects instead of deleting all references to them
      contactStates.clear();
      coefficientsOfFriction.clear();
      forces.clear();
      centersOfPressure.clear();
      normalTorques.clear();
   }

   public void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFrictionIgnored)
   {
      contactStates.add(contactState);
      coefficientsOfFriction.put(contactState, coefficientOfFriction);

      forces.put(contactState, new FrameVector(centerOfMassFrame));
      centersOfPressure.put(contactState, new FramePoint2d(contactState.getPlaneFrame()));
      normalTorques.put(contactState, 0.0);
   }

   public void solve(SpatialForceVector desiredGroundReactionWrench, RobotSide upcomingSupportleg) 
   {
      desiredGroundReactionWrench.changeFrame(centerOfMassFrame);

      computeAMatrix();
      
      
      MatrixTools.denseMatrixToArrayColumnMajor(aMatrix, aMatrixAsDoubleArray);
      MatrixTools.denseMatrixToArrayColumnMajor(normalForceSelectorBMatrix, normalForceSelectorBMatrixAsDoubleArray);
      MatrixTools.denseMatrixToArrayColumnMajor(desiredWrench, desiredWrenchAsDoubleArray);  
      
      try
      {
         contactPointWrenchOptimizerNative.solve(aMatrixAsDoubleArray, diagonalCWeights, normalForceSelectorBMatrixAsDoubleArray, desiredWrenchAsDoubleArray, minimumNormalForces, epsilonRho);
      } 
      catch (NoConvergenceException e)
      {
         e.printStackTrace();
      }
   }

   public FrameVector getForce(PlaneContactState planeContactState)
   {
      return forces.get(planeContactState);
   }

   public FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      return centersOfPressure.get(contactState);
   }

   public double getNormalTorque(PlaneContactState contactState)
   {
      return normalTorques.get(contactState);
   }
   
   
   
   private void computeAMatrix()
   {
      aMatrix.zero();
      
      for (PlaneContactState contactState : contactStates)
      {
         List<FramePoint2d> contactPoints2d = contactState.getContactPoints2d();
         
         for (FramePoint2d contactPoint2d : contactPoints2d)
         {
            for (int supportVectorIndex = 0; supportVectorIndex < ContactPointWrenchOptimizerNative.NUMBER_OF_SUPPORT_VECTORS; supportVectorIndex++)
            {
               WrenchDistributorTools.getSupportVectors(normalizedSupportVectors, coefficientsOfFriction.get(contactState), contactState.getPlaneFrame());

               
            }
         }
      }
        
     
   }
}
