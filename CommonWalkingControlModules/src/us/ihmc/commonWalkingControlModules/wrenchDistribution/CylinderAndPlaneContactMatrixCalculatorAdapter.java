package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

import java.util.*;

/**
 * @author twan
 *         Date: 5/23/13
 */
public class CylinderAndPlaneContactMatrixCalculatorAdapter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final CylinderAndPlaneContactMatrixCalculator cylinderAndPlaneContactMatrixCalculator;
   private final CylinderAndPlaneContactSpatialForceVectorCalculator cylinderAndPlaneContactSpatialForceVectorCalculator;
   private final ReferenceFrame centerOfMassFrame;
   private final Map<RigidBody, Wrench> wrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   private final IntegerYoVariable planeContactIndex = new IntegerYoVariable("planeContactIndex", registry);


   public CylinderAndPlaneContactMatrixCalculatorAdapter(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, int rhoSize, int phiSize)
   {
      cylinderAndPlaneContactMatrixCalculator = new CylinderAndPlaneContactMatrixCalculator(centerOfMassFrame, registry, dynamicGraphicObjectsListRegistry,
              rhoSize, phiSize);
      cylinderAndPlaneContactSpatialForceVectorCalculator = new CylinderAndPlaneContactSpatialForceVectorCalculator(centerOfMassFrame, registry,
              dynamicGraphicObjectsListRegistry, rhoSize, phiSize);
      this.centerOfMassFrame = centerOfMassFrame;
      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;
      parentRegistry.addChild(registry);
   }

   public DenseMatrix64F getRhoMin()
   {
      return cylinderAndPlaneContactMatrixCalculator.getRhoMin();
   }

   public DenseMatrix64F getQRho()
   {
      return cylinderAndPlaneContactMatrixCalculator.getQRho();
   }

   public DenseMatrix64F getPhiMin()
   {
      return cylinderAndPlaneContactMatrixCalculator.getPhiMin();
   }

   public DenseMatrix64F getPhiMax()
   {
      return cylinderAndPlaneContactMatrixCalculator.getPhiMax();
   }

   public DenseMatrix64F getQPhi()
   {
      return cylinderAndPlaneContactMatrixCalculator.getQPhi();
   }

   public void computeMatrices(Map<RigidBody, ? extends PlaneContactState> planeContactStates,
                               Map<RigidBody, ? extends CylindricalContactState> cylindricalContactStates)
   {
      convertToEndEffectorsHack(planeContactStates, cylindricalContactStates);

      cylinderAndPlaneContactMatrixCalculator.computeMatrices(endEffectors.values());
   }

   public Map<RigidBody, Wrench> computeWrenches(DenseMatrix64F rho, DenseMatrix64F phi)
   {
      cylinderAndPlaneContactSpatialForceVectorCalculator.setQRho(cylinderAndPlaneContactMatrixCalculator.getQRho());
      cylinderAndPlaneContactSpatialForceVectorCalculator.setQPhi(cylinderAndPlaneContactMatrixCalculator.getQPhi());
      cylinderAndPlaneContactSpatialForceVectorCalculator.computeWrenches(endEffectors.values(), rho, phi);

      // TODO: garbage:
      wrenches.clear();

      for (RigidBody rigidBody : endEffectors.keySet())
      {
         SpatialForceVector spatialForceVector = cylinderAndPlaneContactSpatialForceVectorCalculator.getSpatialForceVector(endEffectors.get(rigidBody));
         Wrench wrench = new Wrench(rigidBody.getBodyFixedFrame(), spatialForceVector.getExpressedInFrame(), spatialForceVector.getLinearPartCopy(),
                                    spatialForceVector.getAngularPartCopy());
         wrench.changeFrame(rigidBody.getBodyFixedFrame());
         wrenches.put(rigidBody, wrench);
      }

      return wrenches;
   }

   // CONVERSION NASTINESS
   // TODO: clean up
   private final Map<RigidBody, EndEffector> endEffectors = new LinkedHashMap<RigidBody, EndEffector>();
   private Map<PlaneContactState, EndEffector> previouslyUsedPlaneEndEffectors = new LinkedHashMap<PlaneContactState, EndEffector>();
   private Map<CylindricalContactState, EndEffector> previouslyUsedCylinderEndEffectors = new LinkedHashMap<CylindricalContactState, EndEffector>();
   private final List<DynamicGraphicObject> endEffectorResultGraphics = new ArrayList<DynamicGraphicObject>();

   private void convertToEndEffectorsHack(Map<RigidBody, ? extends PlaneContactState> planeContactStates,
                                          Map<RigidBody, ? extends CylindricalContactState> cylindricalContactStates)
   {
      endEffectors.clear();

      if (planeContactStates != null)
      {
         for (RigidBody rigidBody : planeContactStates.keySet())
         {
            PlaneContactState planeContactState = planeContactStates.get(rigidBody);
            if (planeContactState.getNumberOfContactPoints() > 0)
            {
               endEffectors.put(rigidBody, getOrCreate(planeContactState));
            }
         }
      }

      if (cylindricalContactStates != null)
      {
         for (RigidBody rigidBody : cylindricalContactStates.keySet())
         {
            CylindricalContactState cylindricalContactState = cylindricalContactStates.get(rigidBody);
            if (cylindricalContactState.isInContact())
               endEffectors.put(rigidBody, getOrCreate(cylindricalContactState));
         }
      }
   }

   private EndEffector getOrCreate(PlaneContactState planeContactState)
   {
      if (previouslyUsedPlaneEndEffectors.containsKey(planeContactState))
      {
         EndEffector endEffector = previouslyUsedPlaneEndEffectors.get(planeContactState);
         endEffector.updateFromPlane(planeContactState);

         return endEffector;
      }

      EndEffector endEffector = EndEffector.fromPlane("" + planeContactIndex.getIntegerValue(), centerOfMassFrame, planeContactState, registry);
      previouslyUsedPlaneEndEffectors.put(planeContactState, endEffector);
      registerEndEffectorGraphic(endEffector);
      planeContactIndex.increment();

      return endEffector;
   }

   private EndEffector getOrCreate(CylindricalContactState cylinder)
   {
      if (previouslyUsedCylinderEndEffectors.containsKey(cylinder))
      {
         EndEffector endEffector = previouslyUsedCylinderEndEffectors.get(cylinder);
         endEffector.updateFromCylinder(cylinder);

         return endEffector;
      }

      EndEffector endEffector = EndEffector.fromCylinder(centerOfMassFrame, cylinder, registry);
      previouslyUsedCylinderEndEffectors.put(cylinder, endEffector);
      registerEndEffectorGraphic(endEffector);

      return endEffector;
   }

   public void registerEndEffectorGraphic(EndEffector endEffector)
   {
      DynamicGraphicVector wrenchAngularVectorGraphic = endEffector.getOutput().getWrenchAngularVectorGraphic();
      DynamicGraphicVector wrenchLinearVectorGraphic = endEffector.getOutput().getWrenchLinearVectorGraphic();
      endEffectorResultGraphics.add(wrenchAngularVectorGraphic);
      endEffectorResultGraphics.add(wrenchLinearVectorGraphic);
      String listName = this.getClass().getSimpleName() + "EndEffectorResultGraphics";
      if (dynamicGraphicObjectsListRegistry != null)
      {
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, wrenchAngularVectorGraphic);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, wrenchLinearVectorGraphic);
      }
   }
}
