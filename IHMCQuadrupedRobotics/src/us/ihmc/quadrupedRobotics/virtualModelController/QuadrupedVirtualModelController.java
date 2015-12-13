package us.ihmc.quadrupedRobotics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointLimits;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedVirtualModelParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVirtualModelController
{
   private final double MINIMUM_CONTACT_FORCE_REGULARIZATION_WEIGHT = 0.001;

   private final YoVariableRegistry registry;
   private final QuadrupedVirtualModelParameters parameters;
   private final QuadrupedJointNameMap jointMap;
   private final QuadrupedJointLimits jointLimits;

   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame comFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;
   private final ReferenceFrame worldFrame;

   private final FrameVector comForceCommand;
   private final FrameVector comForceOptimal;
   private final FrameVector comTorqueCommand;
   private final FrameVector comTorqueOptimal;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<FrameVector> swingForceCommand;
   private final QuadrantDependentList<FrameVector> swingForceOptimal;
   private final QuadrantDependentList<boolean[]> contactState;
   private final QuadrantDependentList<double[]> contactPressureLowerLimit;
   private final QuadrantDependentList<double[]> contactPressureUpperLimit;
   private final QuadrantDependentList<double[]> contactCoefficientOfFriction;
   private final QuadrantDependentList<FrameVector> contactForceOptimal;

   private final double[] comForceCommandWeights;
   private final double[] comTorqueCommandWeights;
   private double contactForceRegularizationWeight;

   private final YoFrameVector yoComForceCommand;
   private final YoFrameVector yoComForceOptimal;
   private final YoFrameVector yoComTorqueCommand;
   private final YoFrameVector yoComTorqueOptimal;
   private final QuadrantDependentList<YoFramePoint> yoSolePosition;
   private final QuadrantDependentList<YoFrameVector> yoSwingForceCommand;
   private final QuadrantDependentList<YoFrameVector> yoSwingForceOptimal;
   private final QuadrantDependentList<BooleanYoVariable> yoContactState;
   private final QuadrantDependentList<DoubleYoVariable> yoContactPressureLowerLimit;
   private final QuadrantDependentList<DoubleYoVariable> yoContactPressureUpperLimit;
   private final QuadrantDependentList<DoubleYoVariable> yoContactCoefficientOfFriction;
   private final QuadrantDependentList<YoFrameVector> yoContactForceOptimal;

   private final QuadrantDependentList<double[]> jointEffortLowerLimit;
   private final QuadrantDependentList<double[]> jointEffortUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLowerLimit;
   private final QuadrantDependentList<double[]> jointPositionUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLimitStiffness;
   private final QuadrantDependentList<double[]> jointPositionLimitDamping;

   private final QuadrantDependentList<OneDoFJoint[]> legJoints;
   private final QuadrantDependentList<GeometricJacobian> footJacobian;
   private final QuadrantDependentList<PointJacobian> soleJacobian;

   private final DenseMatrix64F comWrenchMap;
   private final DenseMatrix64F comWrenchVector;
   private final DenseMatrix64F comWrenchWeightMatrix;
   private final DenseMatrix64F contactForceVector;
   private final DenseMatrix64F contactForceRegularizationMatrix;
   private final DenseMatrix64F virtualForceVector;
   private final QuadrantDependentList<DenseMatrix64F> legEffortVector;

   private final DenseMatrix64F leastSquaresConstraintVector;
   private final DenseMatrix64F leastSquaresConstraintMatrix;
   private final DenseMatrix64F leastSquaresTemporaryMatrixA;
   private final DenseMatrix64F leastSquaresTemporaryMatrixB;

   private final YoGraphicsList yoGraphicsList;
   private final QuadrantDependentList<YoGraphicVector> yoContactForceOptimalViz;
   private boolean contactForceIsVisible;

   public QuadrupedVirtualModelController(SDFFullRobotModel fullRobotModel, QuadrupedRobotParameters robotParameters, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      parameters = robotParameters.getQuadrupedVirtualModelParameters();
      jointMap = robotParameters.getJointMap();
      jointLimits = robotParameters.getJointLimits();
      registry = new YoVariableRegistry(getClass().getSimpleName());

      // initialize reference frames
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointMap, robotParameters.getPhysicalProperties());
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();
      worldFrame = ReferenceFrame.getWorldFrame();

      // initialize control variables
      comForceCommand = new FrameVector(comFrame);
      comForceOptimal = new FrameVector(comFrame);
      comTorqueCommand = new FrameVector(comFrame);
      comTorqueOptimal = new FrameVector(comFrame);
      solePosition = new QuadrantDependentList<FramePoint>();
      swingForceCommand = new QuadrantDependentList<FrameVector>();
      swingForceOptimal = new QuadrantDependentList<FrameVector>();
      contactState = new QuadrantDependentList<boolean[]>();
      contactPressureLowerLimit = new QuadrantDependentList<double[]>();
      contactPressureUpperLimit = new QuadrantDependentList<double[]>();
      contactCoefficientOfFriction = new QuadrantDependentList<double[]>();
      contactForceOptimal = new QuadrantDependentList<FrameVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(comFrame));
         swingForceCommand.set(robotQuadrant, new FrameVector(comFrame));
         swingForceOptimal.set(robotQuadrant, new FrameVector(comFrame));
         contactState.set(robotQuadrant, new boolean[1]);
         contactPressureLowerLimit.set(robotQuadrant, new double[1]);
         contactPressureUpperLimit.set(robotQuadrant, new double[1]);
         contactCoefficientOfFriction.set(robotQuadrant, new double[1]);
         contactForceOptimal.set(robotQuadrant, new FrameVector(comFrame));
      }

      // initialize optimization weights
      comForceCommandWeights = new double[3];
      comTorqueCommandWeights = new double[3];
      contactForceRegularizationWeight = MINIMUM_CONTACT_FORCE_REGULARIZATION_WEIGHT;

      // initialize yo variables
      yoComForceCommand = new YoFrameVector("comForceCommand", worldFrame, registry);
      yoComForceOptimal = new YoFrameVector("comForceOptimal", worldFrame, registry);
      yoComTorqueCommand = new YoFrameVector("comTorqueCommand", worldFrame, registry);
      yoComTorqueOptimal = new YoFrameVector("comTorqueOptimal", worldFrame, registry);
      yoSolePosition = new QuadrantDependentList<YoFramePoint>();
      yoSwingForceCommand = new QuadrantDependentList<YoFrameVector>();
      yoSwingForceOptimal = new QuadrantDependentList<YoFrameVector>();
      yoContactState = new QuadrantDependentList<BooleanYoVariable>();
      yoContactPressureLowerLimit = new QuadrantDependentList<DoubleYoVariable>();
      yoContactPressureUpperLimit = new QuadrantDependentList<DoubleYoVariable>();
      yoContactCoefficientOfFriction = new QuadrantDependentList<DoubleYoVariable>();
      yoContactForceOptimal = new QuadrantDependentList<YoFrameVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSolePosition.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SolePosition", worldFrame, registry));
         yoSwingForceCommand.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SwingForceCommand", worldFrame, registry));
         yoSwingForceOptimal.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SwingForceOptimal", worldFrame, registry));
         yoContactState.set(robotQuadrant, new BooleanYoVariable(robotQuadrant.getCamelCaseNameForStartOfExpression() + "ContactState", registry));
         yoContactPressureLowerLimit.set(robotQuadrant,
               new DoubleYoVariable(robotQuadrant.getCamelCaseNameForStartOfExpression() + "ContactPressureLowerLimit", registry));
         yoContactPressureUpperLimit.set(robotQuadrant,
               new DoubleYoVariable(robotQuadrant.getCamelCaseNameForStartOfExpression() + "ContactPressureUpperLimit", registry));
         yoContactCoefficientOfFriction.set(robotQuadrant,
               new DoubleYoVariable(robotQuadrant.getCamelCaseNameForStartOfExpression() + "ContactCoefficientOfFriction", registry));
         yoContactForceOptimal.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "ContactForceOptimal", worldFrame, registry));
      }

      // initialize jacobians
      legJoints = new QuadrantDependentList<OneDoFJoint[]>();
      footJacobian = new QuadrantDependentList<GeometricJacobian>();
      soleJacobian = new QuadrantDependentList<PointJacobian>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
      }

      // initialize joint limits
      jointEffortLowerLimit = new QuadrantDependentList<double[]>();
      jointEffortUpperLimit = new QuadrantDependentList<double[]>();
      jointPositionLowerLimit = new QuadrantDependentList<double[]>();
      jointPositionUpperLimit = new QuadrantDependentList<double[]>();
      jointPositionLimitStiffness = new QuadrantDependentList<double[]>();
      jointPositionLimitDamping = new QuadrantDependentList<double[]>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         jointEffortLowerLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointEffortUpperLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLowerLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionUpperLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLimitStiffness.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLimitDamping.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
      }

      // initialize matrix terms
      comWrenchMap = new DenseMatrix64F(6, 12);
      comWrenchVector = new DenseMatrix64F(6, 1);
      comWrenchWeightMatrix = new DenseMatrix64F(6, 6);
      contactForceVector = new DenseMatrix64F(12, 1);
      contactForceRegularizationMatrix = new DenseMatrix64F(12, 12);
      virtualForceVector = new DenseMatrix64F(3, 1);
      legEffortVector = new QuadrantDependentList<DenseMatrix64F>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         legEffortVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
      }
      leastSquaresConstraintVector = new DenseMatrix64F(12, 1);
      leastSquaresConstraintMatrix = new DenseMatrix64F(12, 12);
      leastSquaresTemporaryMatrixA = new DenseMatrix64F(12, 12);
      leastSquaresTemporaryMatrixB = new DenseMatrix64F(12, 12);

      // initialize graphics
      yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      yoContactForceOptimalViz = new QuadrantDependentList<YoGraphicVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         yoContactForceOptimalViz.set(robotQuadrant, new YoGraphicVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleForceOptimal",
               yoSolePosition.get(robotQuadrant), yoContactForceOptimal.get(robotQuadrant), 0.002, YoAppearance.Chartreuse()));
         yoGraphicsList.add(yoContactForceOptimalViz.get(robotQuadrant));
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      // initialize limits
      this.reinitialize();

      parentRegistry.addChild(registry);
   }

   public void reinitialize()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < jointMap.getLegJointNames().length; i++)
         {
            // initialize joint limits
            LegJointName legJointName = jointMap.getLegJointNames()[i];
            String jointName = jointMap.getLegJointName(robotQuadrant, legJointName);
            double positionLowerLimit = jointLimits.getJointSoftPositionLowerLimit(jointName);
            double positionUpperLimit = jointLimits.getJointSoftPositionUpperLimit(jointName);
            double effortLimit = jointLimits.getJointEffortLimit(jointName);
            setJointEffortLimits(jointName, -effortLimit, effortLimit);
            setJointPositionLimits(jointName, positionLowerLimit, positionUpperLimit);
            setJointPositionLimitStiffness(jointName, parameters.getDefaultJointPositionLimitStiffness());
            setJointPositionLimitDamping(jointName, parameters.getDefaultJointPositionLimitDamping());
         }
         // initialize contact state
         setContactState(robotQuadrant, true);
         setContactPressureLimits(robotQuadrant, 0, Double.MAX_VALUE);
         setContactCoefficientOfFriction(robotQuadrant, parameters.getDefaultSoleCoefficientOfFriction());
      }

      // initialize commands
      comForceCommand.setToZero();
      comTorqueCommand.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingForceCommand.get(robotQuadrant).setToZero();
      }

      // initialize optimization weights
      for (int i = 0; i < 3; i++)
      {
         comForceCommandWeights[i] = 1.0;
         comTorqueCommandWeights[i] = 1.0;
      }
      contactForceRegularizationWeight = MINIMUM_CONTACT_FORCE_REGULARIZATION_WEIGHT;

      // initialize visualizers
      setVisible(false);
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public void setJointEffortLimits(String jointName, double lower, double upper)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointEffortLowerLimit.get(robotQuadrant)[index] = lower;
               jointEffortUpperLimit.get(robotQuadrant)[index] = upper;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimits(String jointName, double lower, double upper)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLowerLimit.get(robotQuadrant)[index] = lower;
               jointPositionUpperLimit.get(robotQuadrant)[index] = upper;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimitStiffness(String jointName, double stiffness)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLimitStiffness.get(robotQuadrant)[index] = Math.max(stiffness, 0);
            }
            index++;
         }
      }
   }

   public void setJointPositionLimitDamping(String jointName, double damping)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLimitDamping.get(robotQuadrant)[index] = Math.max(damping, 0);
            }
            index++;
         }
      }
   }

   public void setComForceCommand(FrameVector comForce)
   {
      comForceCommand.setIncludingFrame(comForce);
   }

   public void setComForceCommandWeights(double weights[])
   {
      for (int i = 0; i < 3; i++)
      {
         comForceCommandWeights[i] = Math.max(weights[i], 0);
      }
   }

   public void getComForceOptimal(FrameVector comForce)
   {
      comForce.setIncludingFrame(comForceOptimal);
   }

   public void setComTorqueCommand(FrameVector comTorque)
   {
      comTorqueCommand.setIncludingFrame(comTorque);
   }

   public void setComTorqueCommandWeights(double weights[])
   {
      for (int i = 0; i < 3; i++)
      {
         comTorqueCommandWeights[i] = Math.max(weights[i], 0);
      }
   }

   public void getComTorqueOptimal(FrameVector comTorque)
   {
      comTorque.setIncludingFrame(comTorqueOptimal);
   }

   public void setSwingForceCommand(RobotQuadrant robotQuadrant, FrameVector swingForce)
   {
      swingForceCommand.get(robotQuadrant).set(swingForce);
   }

   public void getSwingForceOptimal(RobotQuadrant robotQuadrant, FrameVector swingForce)
   {
      swingForce.set(swingForceOptimal.get(robotQuadrant));
   }

   public void setContactState(RobotQuadrant robotQuadrant, boolean inContact)
   {
      contactState.get(robotQuadrant)[0] = inContact;
   }

   public void setContactPressureLimits(RobotQuadrant robotQuadrant, double lower, double upper)
   {
      contactPressureLowerLimit.get(robotQuadrant)[0] = Math.max(lower, 0);
      contactPressureUpperLimit.get(robotQuadrant)[0] = Math.max(upper, 0);
   }

   public void setContactCoefficientOfFriction(RobotQuadrant robotQuadrant, double coefficientOfFriction)
   {
      contactCoefficientOfFriction.get(robotQuadrant)[0] = coefficientOfFriction;
   }

   public void setContactForceRegularizationWeight(double weight)
   {
      contactForceRegularizationWeight = Math.max(weight, MINIMUM_CONTACT_FORCE_REGULARIZATION_WEIGHT);
   }

   public void getContactForceOptimal(RobotQuadrant robotQuadrant, FrameVector contactForce)
   {
      contactForce.set(contactForceOptimal.get(robotQuadrant));
   }

   public int getNumberOfActiveContacts()
   {
      int numberOfActiveContacts = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            numberOfActiveContacts++;
         }
      }
      return numberOfActiveContacts;
   }

   public void compute()
   {
      // rotate desired forces and torques to center of mass frame
      comForceCommand.changeFrame(comFrame);
      comTorqueCommand.changeFrame(comFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingForceCommand.get(robotQuadrant).changeFrame(comFrame);
      }

      // compute sole positions and jacobians in center of mass frame
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(comFrame);
         footJacobian.get(robotQuadrant).compute();
         soleJacobian.get(robotQuadrant).set(footJacobian.get(robotQuadrant), solePosition.get(robotQuadrant));
         soleJacobian.get(robotQuadrant).compute();
      }

      int numberOfActiveContacts = getNumberOfActiveContacts();
      if (numberOfActiveContacts > 0)
      {
         // compute map from contact forces to centroidal forces and torques
         comWrenchMap.zero();
         comWrenchMap.reshape(6, 3 * numberOfActiveContacts);
         int columnOffset = 0;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (contactState.get(robotQuadrant)[0])
            {
               comWrenchMap.set(0, 1 + columnOffset, -solePosition.get(robotQuadrant).getZ()); // mX row
               comWrenchMap.set(0, 2 + columnOffset, solePosition.get(robotQuadrant).getY());
               comWrenchMap.set(1, 0 + columnOffset, solePosition.get(robotQuadrant).getZ()); // mY row
               comWrenchMap.set(1, 2 + columnOffset, -solePosition.get(robotQuadrant).getX());
               comWrenchMap.set(2, 0 + columnOffset, -solePosition.get(robotQuadrant).getY()); // mZ row
               comWrenchMap.set(2, 1 + columnOffset, solePosition.get(robotQuadrant).getX());
               comWrenchMap.set(3, 0 + columnOffset, 1.0); // fX row
               comWrenchMap.set(4, 1 + columnOffset, 1.0); // fY row
               comWrenchMap.set(5, 2 + columnOffset, 1.0); // fZ row
               columnOffset += 3;
            }
         }

         // compute centroidal wrench vector
         comWrenchVector.set(0, 0, comTorqueCommand.getX());
         comWrenchVector.set(1, 0, comTorqueCommand.getY());
         comWrenchVector.set(2, 0, comTorqueCommand.getZ());
         comWrenchVector.set(3, 0, comForceCommand.getX());
         comWrenchVector.set(4, 0, comForceCommand.getY());
         comWrenchVector.set(5, 0, comForceCommand.getZ());

         // compute centroidal wrench weight matrix
         comWrenchWeightMatrix.reshape(6, 6);
         for (int i = 0; i < 3; i++)
         {
            comWrenchWeightMatrix.set(i, i, comTorqueCommandWeights[i]);
         }
         for (int i = 0; i < 3; i++)
         {
            comWrenchWeightMatrix.set(i + 3, i + 3, comForceCommandWeights[i]);
         }

         // compute contact forces regularization matrix
         contactForceRegularizationMatrix.reshape(3 * numberOfActiveContacts, 3 * numberOfActiveContacts);
         for (int i = 0; i < 3 * numberOfActiveContacts; i++)
         {
            contactForceRegularizationMatrix.set(i, i, contactForceRegularizationWeight);
         }

         // initialize contact forces vector
         contactForceVector.reshape(3 * numberOfActiveContacts, 1);
      }

      // compute optimal contact forces
      if (getNumberOfActiveContacts() > 0)
      {
         // min_u 1/2 * (Mu - w)'Q(Mu - w) + 1/2 * u'Ru

         // output
         DenseMatrix64F u = contactForceVector;

         // inputs
         DenseMatrix64F w = comWrenchVector;
         DenseMatrix64F M = comWrenchMap;
         DenseMatrix64F Q = comWrenchWeightMatrix;
         DenseMatrix64F R = contactForceRegularizationMatrix;

         // temporary
         DenseMatrix64F b = leastSquaresConstraintVector;
         DenseMatrix64F A = leastSquaresConstraintMatrix;
         DenseMatrix64F MtQM = leastSquaresTemporaryMatrixA;
         DenseMatrix64F MtQ = leastSquaresTemporaryMatrixB;

         int n = w.getNumRows();
         int m = u.getNumRows();

         // b = M'Qw
         MtQ.reshape(m, n);
         CommonOps.multTransA(M, Q, MtQ);
         b.reshape(m, 1);
         CommonOps.mult(MtQ, w, b);

         // A = M'QM + R
         MtQM.reshape(m, m);
         CommonOps.mult(MtQ, M, MtQM);
         A.reshape(m, m);
         CommonOps.add(MtQM, R, A);

         // u = inverse(A)*b
         CommonOps.invert(A);
         CommonOps.mult(A, b, u);
      }

      // TODO
      // compute joint torque inequality constraints
      // compute friction pyramid inequality constraints
      // compute min / max sole pressure constraints
      // compute sole forces using quadratic program

      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            contactForceOptimal.get(robotQuadrant).changeFrame(comFrame);
            contactForceOptimal.get(robotQuadrant).setX(contactForceVector.get(0 + rowOffset, 0));
            contactForceOptimal.get(robotQuadrant).setY(contactForceVector.get(1 + rowOffset, 0));
            contactForceOptimal.get(robotQuadrant).setZ(contactForceVector.get(2 + rowOffset, 0));

            // apply contact force limits
            double mu = contactCoefficientOfFriction.get(robotQuadrant)[0];
            double fx = contactForceOptimal.get(robotQuadrant).getX();
            double fy = contactForceOptimal.get(robotQuadrant).getY();
            double fz = contactForceOptimal.get(robotQuadrant).getZ();
            fz = Math.max(fz, contactPressureLowerLimit.get(robotQuadrant)[0]);
            fz = Math.min(fz, contactPressureUpperLimit.get(robotQuadrant)[0]);
            fx = Math.max(fx, -mu * fz / Math.sqrt(2));
            fx = Math.min(fx, mu * fz / Math.sqrt(2));
            fy = Math.max(fy, -mu * fz / Math.sqrt(2));
            fy = Math.min(fy, mu * fz / Math.sqrt(2));
            contactForceOptimal.get(robotQuadrant).setX(fx);
            contactForceOptimal.get(robotQuadrant).setY(fy);
            contactForceOptimal.get(robotQuadrant).setZ(fz);

            rowOffset += 3;
         }
         else
         {
            contactForceOptimal.get(robotQuadrant).changeFrame(comFrame);
            contactForceOptimal.get(robotQuadrant).setToZero();
         }
      }

      // compute optimal swing forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            swingForceOptimal.get(robotQuadrant).setToZero();
            swingForceOptimal.get(robotQuadrant).changeFrame(comFrame);
         }
         else
         {
            swingForceOptimal.get(robotQuadrant).setIncludingFrame(swingForceCommand.get(robotQuadrant));
            swingForceOptimal.get(robotQuadrant).changeFrame(comFrame);
         }
      }

      // compute optimal CoM forces and torques
      CommonOps.mult(comWrenchMap, contactForceVector, comWrenchVector);
      comTorqueOptimal.changeFrame(comFrame);
      comTorqueOptimal.setX(comWrenchVector.get(0, 0));
      comTorqueOptimal.setY(comWrenchVector.get(1, 0));
      comTorqueOptimal.setZ(comWrenchVector.get(2, 0));
      comForceOptimal.changeFrame(comFrame);
      comForceOptimal.setX(comWrenchVector.get(3, 0));
      comForceOptimal.setY(comWrenchVector.get(4, 0));
      comForceOptimal.setZ(comWrenchVector.get(5, 0));

      // compute joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         DenseMatrix64F jacobianMatrix = soleJacobian.get(robotQuadrant).getJacobianMatrix();
         ReferenceFrame jacobianFrame = soleJacobian.get(robotQuadrant).getFrame();
         if (contactState.get(robotQuadrant)[0])
         {
            contactForceOptimal.get(robotQuadrant).changeFrame(jacobianFrame);
            virtualForceVector.set(0, 0, -contactForceOptimal.get(robotQuadrant).getX());
            virtualForceVector.set(1, 0, -contactForceOptimal.get(robotQuadrant).getY());
            virtualForceVector.set(2, 0, -contactForceOptimal.get(robotQuadrant).getZ());
            contactForceOptimal.get(robotQuadrant).changeFrame(comFrame);
         }
         else
         {
            swingForceOptimal.get(robotQuadrant).changeFrame(jacobianFrame);
            virtualForceVector.set(0, 0, swingForceOptimal.get(robotQuadrant).getX());
            virtualForceVector.set(1, 0, swingForceOptimal.get(robotQuadrant).getY());
            virtualForceVector.set(2, 0, swingForceOptimal.get(robotQuadrant).getZ());
            swingForceOptimal.get(robotQuadrant).changeFrame(comFrame);
         }
         CommonOps.multTransA(jacobianMatrix, virtualForceVector, legEffortVector.get(robotQuadrant));

         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            // apply joint position and torque limits
            double tauPositionLowerLimit = jointPositionLimitStiffness.get(robotQuadrant)[index]
                  * (jointPositionLowerLimit.get(robotQuadrant)[index] - joint.getQ()) - jointPositionLimitDamping.get(robotQuadrant)[index] * joint.getQd();
            double tauPositionUpperLimit = jointPositionLimitStiffness.get(robotQuadrant)[index]
                  * (jointPositionUpperLimit.get(robotQuadrant)[index] - joint.getQ()) - jointPositionLimitDamping.get(robotQuadrant)[index] * joint.getQd();
            double tauEffortLowerLimit = jointEffortLowerLimit.get(robotQuadrant)[index];
            double tauEffortUpperLimit = jointEffortUpperLimit.get(robotQuadrant)[index];
            double tau = legEffortVector.get(robotQuadrant).get(index, 0);
            tau = Math.min(Math.max(tau, tauPositionLowerLimit), tauPositionUpperLimit);
            tau = Math.min(Math.max(tau, tauEffortLowerLimit), tauEffortUpperLimit);

            // update joint torques in full robot model
            joint.setTau(tau);
            index++;
         }
      }

      // update yo variables
      comForceCommand.changeFrame(yoComForceCommand.getReferenceFrame());
      yoComForceCommand.set(comForceCommand);
      comForceOptimal.changeFrame(yoComForceOptimal.getReferenceFrame());
      yoComForceOptimal.set(comForceOptimal);
      comTorqueCommand.changeFrame(yoComTorqueCommand.getReferenceFrame());
      yoComTorqueCommand.set(comTorqueCommand);
      comTorqueOptimal.changeFrame(yoComTorqueOptimal.getReferenceFrame());
      yoComTorqueOptimal.set(comTorqueOptimal);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).changeFrame(yoSolePosition.get(robotQuadrant).getReferenceFrame());
         yoSolePosition.get(robotQuadrant).set(solePosition.get(robotQuadrant));
         swingForceCommand.get(robotQuadrant).changeFrame(yoSwingForceCommand.get(robotQuadrant).getReferenceFrame());
         yoSwingForceCommand.get(robotQuadrant).set(swingForceCommand.get(robotQuadrant));
         swingForceOptimal.get(robotQuadrant).changeFrame(yoSwingForceOptimal.get(robotQuadrant).getReferenceFrame());
         yoSwingForceOptimal.get(robotQuadrant).set(swingForceOptimal.get(robotQuadrant));
         yoContactState.get(robotQuadrant).set(contactState.get(robotQuadrant)[0]);
         yoContactPressureLowerLimit.get(robotQuadrant).set(contactPressureLowerLimit.get(robotQuadrant)[0]);
         yoContactPressureUpperLimit.get(robotQuadrant).set(contactPressureUpperLimit.get(robotQuadrant)[0]);
         yoContactCoefficientOfFriction.get(robotQuadrant).set(contactCoefficientOfFriction.get(robotQuadrant)[0]);
         contactForceOptimal.get(robotQuadrant).changeFrame(yoContactForceOptimal.get(robotQuadrant).getReferenceFrame());
         yoContactForceOptimal.get(robotQuadrant).set(contactForceOptimal.get(robotQuadrant));
      }

      // update graphics
      yoGraphicsList.setVisible(false);
      if (contactForceIsVisible)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (contactState.get(robotQuadrant)[0])
            {
               yoContactForceOptimalViz.get(robotQuadrant).setVisible(true);
            }
         }
      }
   }

   public void setVisible(boolean visible)
   {
      setContactForceVisible(visible);
      yoGraphicsList.setVisible(false);
   }

   public void setContactForceVisible(boolean visible)
   {
      contactForceIsVisible = visible;
      yoGraphicsList.setVisible(false);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}