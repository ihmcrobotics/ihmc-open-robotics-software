package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.collections.map.HashedMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.SelectionCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WrenchMatrixCalculator implements SCS2YoGraphicHolder
{
   private final int nContactableBodies;
   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;
   private final int rhoSize;
   private final int copTaskSize;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean useForceRateHighWeight = new YoBoolean("useForceRateHighWeight", registry);

   private final YoDouble rhoWeight = new YoDouble("rhoWeight", registry);
   private final YoDouble rhoRateDefaultWeight = new YoDouble("rhoRateDefaultWeight", registry);
   @Deprecated
   private final YoDouble rhoRateHighWeight = new YoDouble("rhoRateHighWeight", registry);
   private final YoFrameVector2D desiredCoPWeight = new YoFrameVector2D("desiredCoPWeight", null, registry);

   private final YoFrameVector2D copRateDefaultWeight = new YoFrameVector2D("copRateDefaultWeight", null, registry);
   @Deprecated
   private final YoFrameVector2D copRateHighWeight = new YoFrameVector2D("copRateHighWeight", null, registry);

   private final CoPObjectiveCalculator copObjectiveCalculator = new CoPObjectiveCalculator();

   private final DMatrixRMaj rhoJacobianMatrix;
   private final DMatrixRMaj copJacobianMatrix;
   private final DMatrixRMaj rhoPreviousMatrix;

   private final DMatrixRMaj copRegularizationJacobian;
   private final DMatrixRMaj copRegularizationObjective;
   private final DMatrixRMaj copRateRegularizationJacobian;
   private final DMatrixRMaj copRateRegularizationObjective;
   private final DMatrixRMaj activeRhoMatrix;

   private final DMatrixRMaj rhoMaxMatrix;
   private final DMatrixRMaj rhoWeightMatrix;
   private final DMatrixRMaj rhoRateWeightMatrix;
   private final DMatrixRMaj copRegularizationWeight;
   private final DMatrixRMaj copRateRegularizationWeight;

   private final ReferenceFrame centerOfMassFrame;
   private final Map<RigidBodyBasics, Wrench> wrenchesFromRho = new HashMap<>();

   private final List<RigidBodyBasics> rigidBodies = new ArrayList<>();
   private final Map<RigidBodyBasics, PlaneContactStateToWrenchMatrixHelper> planeContactStateToWrenchMatrixHelpers = new HashMap<>();
   private final TObjectIntMap<RigidBodyBasics> bodyRhoOffsets = new TObjectIntHashMap<RigidBodyBasics>();

   private final Map<RigidBodyBasics, YoFramePoint3D>  commandedCoPs = new HashMap<>();

   private final List<FramePoint3D> basisVectorsOrigin = new ArrayList<>();
   private final List<FrameVector3D> basisVectors = new ArrayList<>();

   private final double dtSquaredInv;

   private final DMatrixRMaj bodyWrenchJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj fullWrenchJacobian = new DMatrixRMaj(0, 0);
   private final FrameVector2D weight = new FrameVector2D();

   public WrenchMatrixCalculator(WholeBodyControlCoreToolbox toolbox, YoRegistry parentRegistry)
   {
      this(toolbox.getCenterOfMassFrame(),
           toolbox.getContactablePlaneBodies(),
           toolbox.getOptimizationSettings(),
           toolbox.getControlDT(),
           parentRegistry,
           toolbox.getYoGraphicsListRegistry());
   }

   public WrenchMatrixCalculator(ReferenceFrame centerOfMassFrame,
                                 List<? extends ContactablePlaneBody> contactablePlaneBodies,
                                 ControllerCoreOptimizationSettings optimizationSettings,
                                 double dt,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry graphicsListRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.dtSquaredInv = 1.0 / (dt * dt);
      this.nContactableBodies = optimizationSettings.getNumberOfContactableBodies();
      this.maxNumberOfContactPoints = optimizationSettings.getNumberOfContactPointsPerContactableBody();
      this.numberOfBasisVectorsPerContactPoint = optimizationSettings.getNumberOfBasisVectorsPerContactPoint();
      this.rhoSize = optimizationSettings.getRhoSize();
      copTaskSize = 2 * nContactableBodies;

      rhoJacobianMatrix = new DMatrixRMaj(SpatialForce.SIZE, rhoSize);
      copJacobianMatrix = new DMatrixRMaj(copTaskSize, rhoSize);
      rhoPreviousMatrix = new DMatrixRMaj(rhoSize, 1);

      copRegularizationJacobian = new DMatrixRMaj(copTaskSize, rhoSize);
      copRegularizationObjective = new DMatrixRMaj(copTaskSize, 1);
      copRateRegularizationJacobian = new DMatrixRMaj(copTaskSize, rhoSize);
      copRateRegularizationObjective = new DMatrixRMaj(copTaskSize, 1);
      activeRhoMatrix = new DMatrixRMaj(rhoSize, 1);

      rhoMaxMatrix = new DMatrixRMaj(rhoSize, 1);
      rhoWeightMatrix = new DMatrixRMaj(rhoSize, rhoSize);
      rhoRateWeightMatrix = new DMatrixRMaj(rhoSize, rhoSize);
      copRegularizationWeight = new DMatrixRMaj(copTaskSize, copTaskSize);
      copRateRegularizationWeight = new DMatrixRMaj(copTaskSize, copTaskSize);

      fullWrenchJacobian.reshape(Wrench.SIZE, rhoSize);

      if (contactablePlaneBodies.size() > nContactableBodies)
         throw new RuntimeException("Unexpected number of contactable plane bodies: " + contactablePlaneBodies.size());

      int rhoOffset = 0;
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         RigidBodyBasics rigidBody = contactablePlaneBody.getRigidBody();

         String copName = contactablePlaneBody.getSoleFrame().getName() + "CommandedCoP";
         YoFramePoint3D cop = new YoFramePoint3D(copName, ReferenceFrame.getWorldFrame(), registry);
         commandedCoPs.put(rigidBody, cop);

         rigidBodies.add(rigidBody);

         FrictionConeRotationCalculator frictionConeRotation = optimizationSettings.getFrictionConeRotation();
         PlaneContactStateToWrenchMatrixHelper helper = new PlaneContactStateToWrenchMatrixHelper(contactablePlaneBody,
                                                                                                  centerOfMassFrame,
                                                                                                  maxNumberOfContactPoints,
                                                                                                  numberOfBasisVectorsPerContactPoint,
                                                                                                  frictionConeRotation,
                                                                                                  registry);
         helper.setDeactivateRhoWhenNotInContact(optimizationSettings.getDeactivateRhoWhenNotInContact());
         planeContactStateToWrenchMatrixHelpers.put(rigidBody, helper);
         bodyRhoOffsets.put(rigidBody, rhoOffset);
         rhoOffset += helper.getRhoSize();

         for (int rhoIndex = 0; rhoIndex < helper.getRhoSize(); rhoIndex++)
         {
            basisVectorsOrigin.add(helper.getBasisVectorsOrigin()[rhoIndex]);
            basisVectors.add(helper.getBasisVectors()[rhoIndex]);
         }

         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

         Wrench wrench = new Wrench(bodyFixedFrame, bodyFixedFrame);
         wrenchesFromRho.put(rigidBody, wrench);

         if (graphicsListRegistry != null)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(copName, cop, 0.005, YoAppearance.Navy(), YoGraphicPosition.GraphicType.BALL);
            copViz.setVisible(PlaneContactWrenchProcessor.VISUALIZE);
         }
      }

      rhoWeight.set(optimizationSettings.getRhoWeight());
      rhoRateDefaultWeight.set(optimizationSettings.getRhoRateDefaultWeight());
      rhoRateHighWeight.set(optimizationSettings.getRhoRateHighWeight());
      desiredCoPWeight.set(optimizationSettings.getCoPWeight());
      copRateDefaultWeight.set(optimizationSettings.getCoPRateDefaultWeight());
      copRateHighWeight.set(optimizationSettings.getCoPRateHighWeight());

      parentRegistry.addChild(registry);
   }

   public void setRhoWeight(double rhoWeight)
   {
      this.rhoWeight.set(rhoWeight);
   }

   public void setRhoRateWeight(double rhoRateWeight)
   {
      this.rhoRateDefaultWeight.set(rhoRateWeight);
   }

   public void setDesiredCoPWeight(Tuple2DReadOnly centerOfPressureWeight)
   {
      this.desiredCoPWeight.set(centerOfPressureWeight);
   }

   public void setCoPRateWeight(Tuple2DReadOnly centerOfPressureRateWeight)
   {
      this.copRateDefaultWeight.set(centerOfPressureRateWeight);
   }

   public void submitPlaneContactStateCommand(PlaneContactStateCommand command)
   {
      PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(command.getContactingRigidBody());
      helper.setPlaneContactStateCommand(command);
      useForceRateHighWeight.set(command.isUseHighCoPDamping());
   }

   private final RecyclingArrayList<CenterOfPressureCommand> centerOfPressureCommands = new RecyclingArrayList<>(CenterOfPressureCommand.class);

   public void submitCenterOfPressureCommand(CenterOfPressureCommand command)
   {
      centerOfPressureCommands.add().set(command);
   }

   public void collectRigidBodiesWithCoPCommands(List<RigidBodyReadOnly> rigidBodies)
   {
      rigidBodies.clear();

      for (int i = 0; i < centerOfPressureCommands.size(); i++)
      {
         CenterOfPressureCommand command = centerOfPressureCommands.get(i);
         RigidBodyBasics rigidBody = command.getContactingRigidBody();
         if (rigidBody != null)
         {
            rigidBodies.add(rigidBody);
            commandedCoPs.get(rigidBody).setMatchingFrame(command.getDesiredCoP(), 0.0);
         }
      }
   }

   public boolean getNextCenterOfPressureInput(NativeQPInputTypeA inputToPack)
   {
      int commands = centerOfPressureCommands.size();
      if (commands <= 0)
      {
         return false;
      }

      CenterOfPressureCommand command = centerOfPressureCommands.get(commands - 1);
      RigidBodyBasics rigidBody = command.getContactingRigidBody();
      FramePoint2DReadOnly desiredCoP = command.getDesiredCoP();

      // Compute the wrench jacobian for the command plane frame:
      // wrench = J * rho
      ReferenceFrame planeFrame = desiredCoP.getReferenceFrame();
      fullWrenchJacobian.zero();
      int rhoStartIndex = 0;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBodies.get(i));

         // If the body is null assume the command is for the full robot.
         if (rigidBody == null || (rigidBodies.get(i) == rigidBody && helper.canHandleCoPCommand()))
         {
            helper.computeWrenchJacobianInFrame(planeFrame, bodyWrenchJacobian);
            CommonOps_DDRM.insert(bodyWrenchJacobian, fullWrenchJacobian, 0, rhoStartIndex);
         }

         rhoStartIndex += helper.getRhoSize();
      }

      // This reshape call also zeros the task objectives, jacobians, and weights
      inputToPack.reshape(2);
      inputToPack.setConstraintType(command.getConstraintType());

      copObjectiveCalculator.computeTask(fullWrenchJacobian,
                                         desiredCoP,
                                         rhoSize,
                                         inputToPack.getTaskJacobian(),
                                         inputToPack.getTaskObjective());

      if (command.getConstraintType() == ConstraintType.OBJECTIVE)
      {
         weight.setIncludingFrame(command.getWeight());
         weight.changeFrameAndProjectToXYPlane(planeFrame);
         inputToPack.getTaskWeightMatrix().set(0, 0, weight.getX());
         inputToPack.getTaskWeightMatrix().set(1, 1, weight.getY());
      }
      else if (command.getConstraintType() != ConstraintType.EQUALITY)
      {
         throw new RuntimeException("Inequalities are not supported by this command.");
      }

      centerOfPressureCommands.remove(commands - 1);
      return true;
   }

   private final RecyclingArrayList<ContactWrenchCommand> contactWrenchCommands = new RecyclingArrayList<>(ContactWrenchCommand.class);

   public void submitContactWrenchCommand(ContactWrenchCommand command)
   {
      // At this point we can not yet compute the matrices since the order of the inverse dynamics commands is not guaranteed. This means
      // the contact state might change. So we need to hold on to the command and wait with computing the task matrices until all inverse
      // dynamics commands are handled.
      contactWrenchCommands.add().set(command);
   }

   private final NativeMatrix rigidBodyRhoTaskJacobian = new NativeMatrix(0, 0);
   private final DMatrixRMaj tempTaskJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj tempTaskObjective = new DMatrixRMaj(Wrench.SIZE, 1);
   private final SelectionCalculator selectionCalculator = new SelectionCalculator();

   public boolean getContactWrenchInput(NativeQPInputTypeA inputToPack)
   {
      int commands = contactWrenchCommands.size();
      if (commands <= 0)
      {
         return false;
      }

      ContactWrenchCommand command = contactWrenchCommands.get(commands - 1);
      int taskSize = command.getSelectionMatrix().getNumberOfSelectedAxes();
      inputToPack.setConstraintType(command.getConstraintType());
      inputToPack.reshape(taskSize);

      computeCommandMatrices(command, rigidBodyRhoTaskJacobian, inputToPack.getTaskObjective(), inputToPack.getTaskWeightMatrix());

      // Add the Jacobian for the body at the right place in the big Jacobian for all bodies:
      int rhoOffset = bodyRhoOffsets.get(command.getRigidBody());
      NativeMatrix fullJacobian = inputToPack.getTaskJacobian();
      fullJacobian.zero();
      fullJacobian.insert(rigidBodyRhoTaskJacobian, 0, rhoOffset);

      contactWrenchCommands.remove(commands - 1);
      return true;
   }

   private void computeCommandMatrices(ContactWrenchCommand command, NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight)
   {
      PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(command.getRigidBody());
      ReferenceFrame planeFrame = helper.getPlaneFrame();
      Wrench wrench = command.getWrench();
      wrench.getBodyFrame().checkReferenceFrameMatch(helper.getRigidBody().getBodyFixedFrame());

      // Get the matrices without considering the selection:
      wrench.changeFrame(planeFrame);
      wrench.get(tempTaskObjective);
      tempTaskJacobian.set(helper.getWrenchJacobianMatrix());

      SelectionMatrix6D selectionMatrix = command.getSelectionMatrix();
      WeightMatrix6D weightMatrix = command.getWeightMatrix();
      if (command.getConstraintType() != ConstraintType.OBJECTIVE)
      {
         weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
         weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
      }
      selectionCalculator.applySelectionToTask(selectionMatrix,
                                               weightMatrix,
                                               planeFrame,
                                               tempTaskJacobian,
                                               tempTaskObjective,
                                               taskJacobian,
                                               taskObjective,
                                               taskWeight);
   }

   public boolean hasContactStateChanged()
   {
      boolean hasContactStateChanged = false;
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodies.get(i);
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBody);

         if (helper.hasReset())
            hasContactStateChanged = true;
      }

      return hasContactStateChanged;
   }

   private final Vector2D tempDeisredCoPWeight = new Vector2D();
   private final Vector2D tempCoPRateWeight = new Vector2D();

   public void computeMatrices(List<RigidBodyReadOnly> rigidBodiesWithCoPCommands)
   {
      double rhoWeight = this.rhoWeight.getDoubleValue();
      double rhoRateWeight;
      tempDeisredCoPWeight.set(desiredCoPWeight);
      if (useForceRateHighWeight.getBooleanValue())
      {
         rhoRateWeight = rhoRateHighWeight.getDoubleValue();
         tempCoPRateWeight.set(copRateHighWeight);
      }
      else
      {
         rhoRateWeight = rhoRateDefaultWeight.getDoubleValue();
         tempCoPRateWeight.set(copRateDefaultWeight);
      }

      int rhoStartIndex = 0;
      int copStartIndex = 0;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodies.get(i);
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBody);

         helper.computeMatrices(rhoWeight, rhoRateWeight, tempDeisredCoPWeight, tempCoPRateWeight);

         CommonOps_DDRM.insert(helper.getLastRho(), rhoPreviousMatrix, rhoStartIndex, 0);
         CommonOps_DDRM.insert(helper.getActiveRhoMatrix(), activeRhoMatrix, rhoStartIndex, 0);
         CommonOps_DDRM.insert(helper.getRhoJacobian(), rhoJacobianMatrix, 0, rhoStartIndex);

         CommonOps_DDRM.insert(helper.getRhoMax(), rhoMaxMatrix, rhoStartIndex, 0);
         CommonOps_DDRM.insert(helper.getRhoWeight(), rhoWeightMatrix, rhoStartIndex, rhoStartIndex);
         CommonOps_DDRM.insert(helper.getRhoRateWeight(), rhoRateWeightMatrix, rhoStartIndex, rhoStartIndex);

         if (rigidBodiesWithCoPCommands == null || !rigidBodiesWithCoPCommands.contains(rigidBody))
         {
            CommonOps_DDRM.insert(helper.getCoPRegularizationJacobian(), copRegularizationJacobian, copStartIndex, rhoStartIndex);
            CommonOps_DDRM.insert(helper.getCoPRegularizationObjective(), copRegularizationObjective, copStartIndex, 0);
            CommonOps_DDRM.insert(helper.getCoPRegularizationWeight(), copRegularizationWeight, copStartIndex, copStartIndex);

            commandedCoPs.get(rigidBody).setMatchingFrame(helper.getDesiredCoP(), 0.0);
         }

         CommonOps_DDRM.insert(helper.getCoPRateRegularizationJacobian(), copRateRegularizationJacobian, copStartIndex, rhoStartIndex);
         CommonOps_DDRM.insert(helper.getCoPRateRegularizationObjective(), copRateRegularizationObjective, copStartIndex, 0);
         CommonOps_DDRM.insert(helper.getCoPRateRegularizationWeight(), copRateRegularizationWeight, copStartIndex, copStartIndex);

         rhoStartIndex += helper.getRhoSize();
         copStartIndex += 2;
      }

      CommonOps_DDRM.scale(dtSquaredInv, rhoRateWeightMatrix);
      CommonOps_DDRM.scale(dtSquaredInv, copRateRegularizationWeight);
   }

   public Map<RigidBodyBasics, Wrench> computeWrenchesFromRho(DMatrixRMaj rho)
   {
      // Reinintialize wrenches
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodies.get(i);
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
         wrenchesFromRho.get(rigidBody).setToZero(bodyFixedFrame, bodyFixedFrame);
      }

      int rhoStartIndex = 0;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodies.get(i);
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
         PlaneContactStateToWrenchMatrixHelper helper = planeContactStateToWrenchMatrixHelpers.get(rigidBody);

         helper.computeWrenchFromRho(rhoStartIndex, rho);
         Wrench wrenchFromRho = helper.getWrenchFromRho();
         wrenchFromRho.changeFrame(bodyFixedFrame);
         wrenchesFromRho.get(rigidBody).add(wrenchFromRho);

         rhoStartIndex += helper.getRhoSize();
      }

      return wrenchesFromRho;
   }

   /**
    * Gets the list of rigid-bodies that are registered as contactable.
    * <p>
    * The ordering of the list indicate the ordering of &rho;s in the matrices.
    * </p>
    * 
    * @return the list of rigid-bodies that can contact with environment and are part of the
    *         optimization.
    */
   public List<RigidBodyBasics> getRigidBodies()
   {
      return rigidBodies;
   }

   public DMatrixRMaj getRhoJacobianMatrix()
   {
      return rhoJacobianMatrix;
   }

   public void getRhoJacobianMatrix(DMatrixRMaj rhoJacobianMatrix)
   {
      rhoJacobianMatrix.set(this.rhoJacobianMatrix);
   }

   public DMatrixRMaj getCopJacobianMatrix()
   {
      return copJacobianMatrix;
   }

   public DMatrixRMaj getRhoPreviousMatrix()
   {
      return rhoPreviousMatrix;
   }

   public DMatrixRMaj getActiveRhoMatrix()
   {
      return activeRhoMatrix;
   }

   public DMatrixRMaj getRhoMaxMatrix()
   {
      return rhoMaxMatrix;
   }

   public DMatrixRMaj getRhoWeightMatrix()
   {
      return rhoWeightMatrix;
   }

   public DMatrixRMaj getRhoRateWeightMatrix()
   {
      return rhoRateWeightMatrix;
   }

   public DMatrixRMaj getCoPRegularizationJacobian()
   {
      return copRegularizationJacobian;
   }

   public DMatrixRMaj getCoPRegularizationObjective()
   {
      return copRegularizationObjective;
   }

   public DMatrixRMaj getCoPRateRegularizationJacobian()
   {
      return copRateRegularizationJacobian;
   }

   public DMatrixRMaj getCoPRateRegularizationObjective()
   {
      return copRateRegularizationObjective;
   }

   public DMatrixRMaj getCoPRegularizationWeight()
   {
      return copRegularizationWeight;
   }

   public DMatrixRMaj getCoPRateRegularizationWeight()
   {
      return copRateRegularizationWeight;
   }

   public Map<RigidBodyBasics, Wrench> getWrenchesFromRho()
   {
      return wrenchesFromRho;
   }

   public List<FramePoint3D> getBasisVectorsOrigin()
   {
      return basisVectorsOrigin;
   }

   public DMatrixRMaj getRhoJacobianMatrix(RigidBodyBasics rigidBody)
   {
      return planeContactStateToWrenchMatrixHelpers.get(rigidBody).getRhoJacobian();
   }

   public ReferenceFrame getJacobianFrame()
   {
      return centerOfMassFrame;
   }

   public List<FrameVector3D> getBasisVectors()
   {
      return basisVectors;
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public int getCopTaskSize()
   {
      return copTaskSize;
   }

   public PlaneContactStateToWrenchMatrixHelper getPlaneContactStateToWrenchMatrixHelper(RigidBodyBasics rigidBody)
   {
      return planeContactStateToWrenchMatrixHelpers.get(rigidBody);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         YoFramePoint3D cop = commandedCoPs.get(rigidBodies.get(i));
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D(cop.getNamePrefix(), cop, 0.010, ColorDefinitions.Black(), YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE));
      }
      group.setVisible(PlaneContactWrenchProcessor.VISUALIZE);
      return group;
   }
}
