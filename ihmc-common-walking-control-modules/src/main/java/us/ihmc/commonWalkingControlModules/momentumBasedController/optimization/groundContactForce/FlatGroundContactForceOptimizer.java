package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class FlatGroundContactForceOptimizer
{
   private static final int maxContactPointsForViz = 30;
   private static final AppearanceDefinition transparentBlue = new YoAppearanceRGBColor(Color.BLUE, 0.8);
   private static final AppearanceDefinition transparentRed = new YoAppearanceRGBColor(Color.RED, 0.8);

   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble friction = new YoDouble("Friction", registry);
   private final YoInteger vectorsPerContact = new YoInteger("VectorsPerContact", registry);
   private final YoDouble regWeight = new YoDouble("RegWeight", registry);

   private final List<List<FrameVector3D>> forceVectors = new ArrayList<>();
   private final List<List<YoFrameVector>> yoGRFVectors = new ArrayList<>();

   private final List<YoFramePoint> yoContactPoints = new ArrayList<>();
   private final YoFramePoint yoCenterOfMass;
   private final YoFrameVector yoForce;
   private final YoFrameVector yoAchievedForce;
   private final YoFrameVector yoTorque;
   private final YoFrameVector yoAchievedTorque;

   private final QuadProgSolver solver = new QuadProgSolver();

   public FlatGroundContactForceOptimizer(double friction, int vectorsPerContact, double regWeight, YoGraphicsListRegistry graphicsListRegistry,
                                          YoVariableRegistry parentRegistry)
   {
      this.friction.set(friction);
      this.vectorsPerContact.set(vectorsPerContact);
      this.regWeight.set(regWeight);

      if (graphicsListRegistry == null)
      {
         graphicsListRegistry = new YoGraphicsListRegistry();
      }

      for (int n = 0; n < maxContactPointsForViz; n++)
      {
         YoFramePoint yoContactPoint = new YoFramePoint("ContactPointPosition" + n, ReferenceFrame.getWorldFrame(), registry);
         yoContactPoints.add(yoContactPoint);

         List<FrameVector3D> forceVectors = new ArrayList<>();
         List<YoFrameVector> yoForceVectors = new ArrayList<>();
         List<YoFrameVector> yoGRFVectors = new ArrayList<>();

         for (int i = 0; i < vectorsPerContact; i++)
         {
            double angle = 2.0 * Math.PI * i / vectorsPerContact;
            double x = Math.sin(angle) * friction;
            double y = Math.cos(angle) * friction;
            FrameVector3D vector = new FrameVector3D(ReferenceFrame.getWorldFrame(), x, y, 1.0);
            vector.normalize();
            forceVectors.add(vector);
            YoFrameVector yoVector = new YoFrameVector("ForceVector" + i + "Contact" + n, ReferenceFrame.getWorldFrame(), registry);
            YoFrameVector yoGRFVector = new YoFrameVector("GRFVector" + i + "Contact" + n, ReferenceFrame.getWorldFrame(), registry);
            yoForceVectors.add(yoVector);
            yoGRFVectors.add(yoGRFVector);
            yoVector.set(vector);

            YoGraphicVector vectorViz = new YoGraphicVector("Force Vector " + i + " Contact " + n, yoContactPoint, yoVector, 0.2, YoAppearance.Aquamarine());
            graphicsListRegistry.registerYoGraphic("Friction Cone", vectorViz);
            YoGraphicVector grfViz = new YoGraphicVector("GRF Vector " + i + " Contact " + n, yoContactPoint, yoGRFVector, 1.0, transparentBlue);
            graphicsListRegistry.registerYoGraphic("GRF", grfViz);
         }

         this.yoGRFVectors.add(yoGRFVectors);
         this.forceVectors.add(forceVectors);
      }

      yoCenterOfMass = new YoFramePoint("CenterOfMass", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition comViz = new YoGraphicPosition("Center of Mass", yoCenterOfMass, 0.03, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic("Center of Mass", comViz);

      yoForce = new YoFrameVector("force", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicVector forceViz = new YoGraphicVector("Desired Linear Momentum Rate", yoCenterOfMass, yoForce, 1.0, YoAppearance.Blue());
      graphicsListRegistry.registerYoGraphic("Desired Linear Momentum Rate", forceViz);

      yoAchievedForce = new YoFrameVector("achievedForce", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicVector achievedForceViz = new YoGraphicVector("Achieved Linear Momentum Rate", yoCenterOfMass, yoAchievedForce, 1.0, transparentBlue);
      graphicsListRegistry.registerYoGraphic("Achieved Linear Momentum Rate", achievedForceViz);

      yoTorque = new YoFrameVector("torque", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicVector torqueViz = new YoGraphicVector("Desired Angular Momentum Rate", yoCenterOfMass, yoTorque, 1.0, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic("Desired Angular Momentum Rate", torqueViz);

      yoAchievedTorque = new YoFrameVector("achievedTorque", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicVector achievedTorqueViz = new YoGraphicVector("Achieved Angular Momentum Rate", yoCenterOfMass, yoAchievedTorque, 1.0, transparentRed);
      graphicsListRegistry.registerYoGraphic("Achieved Angular Momentum Rate", achievedTorqueViz);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      hide();
   }

   private final DenseMatrix64F x = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Q = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F f = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Aeq = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F beq = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Ain = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F bin = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F objective = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F J = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F W = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F temp1 = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F temp2 = new DenseMatrix64F(1, 1);

   private final Vector3D offset = new Vector3D();
   private final Vector3D unitTorque = new Vector3D();
   private final Vector3D unitForce = new Vector3D();

   private final FrameVector3D contactForce = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D resultForce = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D resultTorque = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final Vector3D forceVector = new Vector3D();
   private final Vector3D torqueVector = new Vector3D();

   /**
    * This method generated garbage.
    * It should be used if there are groups of contact points (such as a foot) that can exert a wrench. If this wrench if of interest this method
    * can be used to compute the ground reaction forces and return the wrenches exerted by the groups of contact points. The contact frames are
    * the point at which this wrench is assumed to be located.
    */
   public List<Wrench> compute(List<List<FramePoint3D>> contactPlanes, List<ReferenceFrame> contactFrames, FramePoint3D centerOfMass, FrameVector3D force,
                               FrameVector3D torque, WeightMatrix6D weights)
   {
      List<FramePoint3D> contactPoints = new ArrayList<>();
      List<FrameVector3D> contactForces = new ArrayList<>();

      contactPlanes.stream().forEachOrdered(points -> contactPoints.addAll(points));

      if (!compute(contactPoints, centerOfMass, force, torque, weights, contactForces))
      {
         return null;
      }

      List<Wrench> wrenches = new ArrayList<>();

      int indexOffset = 0;
      for (int planeIdx = 0; planeIdx < contactPlanes.size(); planeIdx++)
      {
         resultForce.setToZero();
         resultTorque.setToZero();
         ReferenceFrame planeFrame = contactFrames.get(planeIdx);
         FramePoint3D planeCenter = new FramePoint3D(planeFrame);
         planeCenter.changeFrame(ReferenceFrame.getWorldFrame());

         int contactPointsInPlane = contactPlanes.get(planeIdx).size();
         for (int pointIdx = 0; pointIdx < contactPointsInPlane; pointIdx++)
         {
            int index = pointIdx + indexOffset;
            FrameVector3D contactForce = contactForces.get(index);
            offset.sub(planeCenter, contactPoints.get(index));
            torqueVector.cross(forceVector, offset);
            resultForce.add(contactForce);
            resultTorque.add(torqueVector);
         }
         indexOffset += contactPointsInPlane;

         Wrench wrench = new Wrench(planeFrame, ReferenceFrame.getWorldFrame(), resultForce, resultTorque);
         wrenches.add(wrench);
      }

      return wrenches;
   }

   /**
    * Method computes the ground reaction forces at the contact points that will best achieve the specified momentum rate.
    */
   public boolean compute(List<FramePoint3D> contactPoints, FramePoint3D centerOfMass, FrameVector3D force, FrameVector3D torque, WeightMatrix6D weights)
   {
      return compute(contactPoints, centerOfMass, force, torque, weights, null);
   }

   /**
    * Method computes the ground reaction forces at the contact points that will best achieve the specified momentum rate. This method also
    * packs the forces at the contact points if contactForcesToPack is not null. In that case the method will generate garbage.
    */
   public boolean compute(List<FramePoint3D> contactPoints, FramePoint3D centerOfMass, FrameVector3D force, FrameVector3D torque, WeightMatrix6D weights,
                          List<FrameVector3D> contactForcesToPack)
   {
      for (int contactIdx = 0; contactIdx < contactPoints.size(); contactIdx++)
      {
         yoContactPoints.get(contactIdx).set(contactPoints.get(contactIdx));
      }

      yoCenterOfMass.set(centerOfMass);
      yoForce.set(force);
      yoTorque.set(torque);

      int vectorsPerPoint = vectorsPerContact.getIntegerValue();
      int size = vectorsPerPoint * contactPoints.size();

      // initialize x as vector with the grf magnitudes
      x.reshape(size, 1);
      CommonOps.fill(x, 0.0);

      // set up inequality constraints such that Ain * x < bin makes sure all x are positive
      Ain.reshape(size, size);
      CommonOps.setIdentity(Ain);
      CommonOps.scale(-1.0, Ain);
      bin.reshape(size, 1);
      CommonOps.fill(bin, 0.0);

      // no equality constraints
      Aeq.reshape(0, size);
      beq.reshape(0, size);

      // regulate the resulting forces: add small diagonal to the Q matrix
      Q.reshape(size, size);
      CommonOps.setIdentity(Q);
      CommonOps.scale(regWeight.getDoubleValue(), Q);

      weights.getFullWeightMatrixInFrame(ReferenceFrame.getWorldFrame(), W);

      // build the task jacobian J and the objective o such that the goal is to minimize (Jx - objective)
      J.reshape(6, size);
      for (int contactIdx = 0; contactIdx < contactPoints.size(); contactIdx++)
      {
         // vector from contact to center of mass
         offset.sub(centerOfMass, contactPoints.get(contactIdx));

         for (int vectorIdx = 0; vectorIdx < vectorsPerPoint; vectorIdx++)
         {
            unitForce.set(forceVectors.get(contactIdx).get(vectorIdx));
            unitTorque.cross(unitForce, offset);

            J.set(0, contactIdx * vectorsPerPoint + vectorIdx, unitTorque.getX());
            J.set(1, contactIdx * vectorsPerPoint + vectorIdx, unitTorque.getY());
            J.set(2, contactIdx * vectorsPerPoint + vectorIdx, unitTorque.getZ());
            J.set(3, contactIdx * vectorsPerPoint + vectorIdx, unitForce.getX());
            J.set(4, contactIdx * vectorsPerPoint + vectorIdx, unitForce.getY());
            J.set(5, contactIdx * vectorsPerPoint + vectorIdx, unitForce.getZ());
         }
      }

      objective.reshape(6, 1);
      objective.set(0, torque.getX());
      objective.set(1, torque.getY());
      objective.set(2, torque.getZ());
      objective.set(3, force.getX());
      objective.set(4, force.getY());
      objective.set(5, force.getZ());

      // assemble the matrices for the optimization such that we can minimize 0.5 x'Qx + x'f
      // the task is minimizing
      // 0.5 * (Jx - objective)' W (Jx - objective) = 0.5 * x'J'WJx - x'J'W objective
      // temp1 = J'W
      temp1.reshape(size, 6);
      CommonOps.multTransA(J, W, temp1);
      // temp2 = temp1 * J = J'WJ
      temp2.reshape(size, size);
      CommonOps.mult(temp1, J, temp2);

      // add J'WJ to Q
      CommonOps.add(Q, temp2, Q);

      // set f to -J'W
      f.reshape(size, 1);
      CommonOps.fill(f, 0.0);
      CommonOps.mult(temp1, objective, f);
      CommonOps.scale(-1.0, f);

      try
      {
         solver.solve(Q, f, Aeq, beq, Ain, bin, x, false);
      }
      catch (NoConvergenceException e)
      {
         hide();
         return false;
      }

      resultForce.setToZero();
      resultTorque.setToZero();

      if (contactForcesToPack != null)
      {
         contactForcesToPack.clear();
      }

      for (int contactIdx = 0; contactIdx < contactPoints.size(); contactIdx++)
      {
         contactForce.setToZero();
         for (int vectorIdx = 0; vectorIdx < vectorsPerPoint; vectorIdx++)
         {
            forceVector.set(forceVectors.get(contactIdx).get(vectorIdx));
            forceVector.scale(x.get(contactIdx * vectorsPerPoint + vectorIdx));
            yoGRFVectors.get(contactIdx).get(vectorIdx).set(forceVector);

            offset.sub(centerOfMass, contactPoints.get(contactIdx));
            torqueVector.cross(forceVector, offset);

            contactForce.add(forceVector);
            resultForce.add(forceVector);
            resultTorque.add(torqueVector);
         }

         if (contactForcesToPack != null)
         {
            contactForcesToPack.add(new FrameVector3D(contactForce));
         }
      }

      yoAchievedForce.set(resultForce);
      yoAchievedTorque.set(resultTorque);
      return true;
   }

   public void getAchievedMomentumRate(FrameVector3D angularMomentumRateToPack, FrameVector3D linearMomentumRateToPack)
   {
      angularMomentumRateToPack.set(yoAchievedTorque);
      linearMomentumRateToPack.set(yoAchievedForce);
   }

   private void hide()
   {
      yoContactPoints.stream().forEach(point -> point.setToNaN());
   }
}
