package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 5/1/13
 */
public class ContactPointWrenchMatrixCalculator
{
   private final ReferenceFrame centerOfMassFrame;

   private final DenseMatrix64F q;
   private final Map<RigidBody, Wrench> wrenches = new LinkedHashMap<RigidBody, Wrench>();

   // intermediate result storage:
   private final ArrayList<FrameVector> normalizedSupportVectors = new ArrayList<FrameVector>(4);
   private final FramePoint tempContactPoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FrameVector tempVector = new FrameVector(ReferenceFrame.getWorldFrame());
   private final DenseMatrix64F qBlock = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F rhoBlock = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F rhoMin;

   public ContactPointWrenchMatrixCalculator(ReferenceFrame centerOfMassFrame, int nSupportVectorsPerContactPoint, int nColumns)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      for (int i = 0; i < nSupportVectorsPerContactPoint; i++)
      {
         normalizedSupportVectors.add(new FrameVector(ReferenceFrame.getWorldFrame()));
      }
      q = new DenseMatrix64F(Wrench.SIZE, nColumns);
      rhoMin = new DenseMatrix64F(nColumns, 1);
   }

   public DenseMatrix64F getRhoMin(Collection<? extends PlaneContactState> contactStates, double rhoMinScalar)
   {
      rhoMin.zero();
      int index = 0;
      for (PlaneContactState contactState : contactStates)
      {
         for (int i = 0; i < contactState.getNumberOfContactPointsInContact(); i++)
         {
            for (int j = 0; j < normalizedSupportVectors.size(); j++)
            {
               rhoMin.set(index++, 0, rhoMinScalar);
            }
         }
      }

      return rhoMin;
   }

   public void computeMatrix(Collection<? extends PlaneContactState> contactStates)
   {
      q.zero();

      int column = 0;
      for (PlaneContactState contactState : contactStates)
      {
         List<FramePoint2d> contactPoints2d = contactState.getContactFramePoints2dInContactCopy();
         WrenchDistributorTools.getSupportVectors(normalizedSupportVectors, contactState.getCoefficientOfFriction(), contactState.getPlaneFrame()); // TODO: use normal

         for (FramePoint2d contactPoint2d : contactPoints2d)
         {
            // torque part of A
            tempContactPoint.setIncludingFrame(contactPoint2d.getReferenceFrame(), contactPoint2d.getX(), contactPoint2d.getY(), 0.0);
            tempContactPoint.changeFrame(centerOfMassFrame);

            for (FrameVector supportVector : normalizedSupportVectors)
            {
               supportVector.changeFrame(centerOfMassFrame);
               supportVector.getVector().get(3, column, q);

               tempVector.setToZero(centerOfMassFrame);
               tempVector.cross(tempContactPoint, supportVector);
               tempVector.getVector().get(0, column, q);
               column++;
            }
         }
      }
   }

   public DenseMatrix64F getMatrix()
   {
      return q;
   }

   public Map<RigidBody, Wrench> computeWrenches(LinkedHashMap<RigidBody, ? extends PlaneContactState> contactStates, DenseMatrix64F rho)
   {
      wrenches.clear();
      int columnNumber = 0;
      for (RigidBody rigidBody : contactStates.keySet())
      {
         PlaneContactState contactState = contactStates.get(rigidBody);

         int nColumns = contactState.getNumberOfContactPointsInContact() * normalizedSupportVectors.size();
         if (nColumns > 0)
         {
            qBlock.reshape(Wrench.SIZE, nColumns);
            CommonOps.extract(q, 0, Wrench.SIZE, columnNumber, columnNumber + nColumns, qBlock, 0, 0);

            rhoBlock.reshape(nColumns, 1);
            CommonOps.extract(rho, columnNumber, columnNumber + nColumns, 0, 1, rhoBlock, 0, 0);

            CommonOps.mult(qBlock, rhoBlock, wrenchMatrix);

            Wrench wrench = new Wrench(rigidBody.getBodyFixedFrame(), centerOfMassFrame);
            wrench.set(centerOfMassFrame, wrenchMatrix);
            wrench.changeFrame(rigidBody.getBodyFixedFrame());
            wrenches.put(rigidBody, wrench);
            columnNumber += nColumns;
         }
      }
      return wrenches;
   }
}
