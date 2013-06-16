package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;

/**
 * @author twan
 *         Date: 5/23/13
 */
public class CylinderAndPlaneContactMatrixCalculatorAdapter
{
   //TODO (Sylvain): change all variable to private (used for CarIngressEgressController)
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final CylinderAndPlaneContactMatrixCalculator cylinderAndPlaneContactMatrixCalculator;
   private final CylinderAndPlaneContactSpatialForceVectorCalculator cylinderAndPlaneContactSpatialForceVectorCalculator;
   protected final ReferenceFrame centerOfMassFrame;
   private final Map<RigidBody, Wrench> wrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   protected final IntegerYoVariable planeContactIndex = new IntegerYoVariable("planeContactIndex", registry);
   private final DoubleYoVariable wRhoCylinderContacts = new DoubleYoVariable("wRhoCylinderContacts", registry);
   private final DoubleYoVariable wPhiCylinderContacts = new DoubleYoVariable("wPhiCylinderContacts", registry);
   private final DoubleYoVariable wRhoPlaneContacts = new DoubleYoVariable("wRhoPlaneContacts", registry);
   protected final DoubleYoVariable rhoMinScalar = new DoubleYoVariable("rhoMinScalarInAdapter", registry);


   public CylinderAndPlaneContactMatrixCalculatorAdapter(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, int rhoSize, int phiSize, double wRhoCylinderContacts,
           double wPhiCylinderContacts, double wRhoPlaneContacts)
   {
      cylinderAndPlaneContactMatrixCalculator = new CylinderAndPlaneContactMatrixCalculator(centerOfMassFrame, registry, dynamicGraphicObjectsListRegistry,
              rhoSize, phiSize);
      cylinderAndPlaneContactSpatialForceVectorCalculator = new CylinderAndPlaneContactSpatialForceVectorCalculator(centerOfMassFrame, registry,
              dynamicGraphicObjectsListRegistry, rhoSize, phiSize);
      this.centerOfMassFrame = centerOfMassFrame;
      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;
      parentRegistry.addChild(registry);

      this.wRhoCylinderContacts.set(wRhoCylinderContacts);
      this.wPhiCylinderContacts.set(wPhiCylinderContacts);
      this.wRhoPlaneContacts.set(wRhoPlaneContacts);
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

   public void computeMatrices(LinkedHashMap<RigidBody,Set<PlaneContactState>> planeContactStates,
                               Map<RigidBody, ? extends CylindricalContactState> cylindricalContactStates)
   {
      convertToEndEffectorsHack(planeContactStates, cylindricalContactStates);

      cylinderAndPlaneContactMatrixCalculator.computeMatrices(computeEndEffectorCollection(endEffectors.values()));
   }

   private Collection<? extends EndEffector> computeEndEffectorCollection(Collection<Set<EndEffector>> values)
   {
      List<EndEffector> ret = new ArrayList<EndEffector>();
      for (Set<EndEffector> set : values)
      {
         ret.addAll(set);
      }
      return ret;
   }

   public Map<RigidBody, Wrench> computeWrenches(DenseMatrix64F rho, DenseMatrix64F phi)
   {
      cylinderAndPlaneContactSpatialForceVectorCalculator.setQRho(cylinderAndPlaneContactMatrixCalculator.getQRho());
      cylinderAndPlaneContactSpatialForceVectorCalculator.setQPhi(cylinderAndPlaneContactMatrixCalculator.getQPhi());
      cylinderAndPlaneContactSpatialForceVectorCalculator.computeWrenches(computeEndEffectorCollection(endEffectors.values()), rho, phi);

      // TODO: garbage:
      wrenches.clear();

      for (RigidBody rigidBody : endEffectors.keySet())
      {
         Set<EndEffector> set = endEffectors.get(rigidBody);
         
         for (EndEffector endEffector : set)
         {
            SpatialForceVector spatialForceVector = cylinderAndPlaneContactSpatialForceVectorCalculator.getSpatialForceVector(endEffector);
            Wrench newWrench = new Wrench(rigidBody.getBodyFixedFrame(), spatialForceVector.getExpressedInFrame(), spatialForceVector.getLinearPartCopy(),
                                       spatialForceVector.getAngularPartCopy());
            newWrench.changeFrame(rigidBody.getBodyFixedFrame());
            
            Wrench wrenchInMap = wrenches.get(rigidBody);
            if (wrenchInMap == null)
            {
               wrenches.put(rigidBody, newWrench);
            }
            else
            {
               wrenchInMap.add(newWrench);
            }
         }
      }

      return wrenches;
   }

   // CONVERSION NASTINESS
   // TODO: clean up
   private final Map<RigidBody, Set<EndEffector>> endEffectors = new LinkedHashMap<RigidBody, Set<EndEffector>>();
   protected Map<PlaneContactState, EndEffector> previouslyUsedPlaneEndEffectors = new LinkedHashMap<PlaneContactState, EndEffector>();
   protected Map<CylindricalContactState, EndEffector> previouslyUsedCylinderEndEffectors = new LinkedHashMap<CylindricalContactState, EndEffector>();
   private final List<DynamicGraphicObject> endEffectorResultGraphics = new ArrayList<DynamicGraphicObject>();

   private void convertToEndEffectorsHack(LinkedHashMap<RigidBody,Set<PlaneContactState>> planeContactStates,
           Map<RigidBody, ? extends CylindricalContactState> cylindricalContactStates)
   {
      endEffectors.clear();

      if (planeContactStates != null)
      {
         for (RigidBody rigidBody : planeContactStates.keySet())
         {
            Set<PlaneContactState> set = planeContactStates.get(rigidBody);
            
            for (PlaneContactState planeContactState : set)
            {               
               if (planeContactState.getNumberOfContactPoints() > 0)
               {
                  Set<EndEffector> endEffectorSet = endEffectors.get(rigidBody);
                  if (endEffectorSet == null)
                  {
                     endEffectorSet = new LinkedHashSet<EndEffector>();
                     endEffectors.put(rigidBody, endEffectorSet);
                  }
                  
                  endEffectorSet.add(getOrCreate(planeContactState));                  
               }
            }
         }
      }

      if (cylindricalContactStates != null)
      {
         for (RigidBody rigidBody : cylindricalContactStates.keySet())
         {
            CylindricalContactState cylindricalContactState = cylindricalContactStates.get(rigidBody);
            if (cylindricalContactState.isInContact())
            {
               Set<EndEffector> endEffectorSet = endEffectors.get(rigidBody);
               if (endEffectorSet == null)
               {
                  endEffectorSet = new LinkedHashSet<EndEffector>();
                  endEffectors.put(rigidBody, endEffectorSet);
               }
               endEffectorSet.add(getOrCreate(cylindricalContactState));
            }
         }
      }
   }

   //TODO (Sylvain): change to private (used for CarIngressEgressController)
   protected EndEffector getOrCreate(PlaneContactState planeContactState)
   {
      if (previouslyUsedPlaneEndEffectors.containsKey(planeContactState))
      {
         EndEffector endEffector = previouslyUsedPlaneEndEffectors.get(planeContactState);
         endEffector.updateFromPlane(planeContactState, wRhoPlaneContacts.getDoubleValue(), rhoMinScalar.getDoubleValue());

         return endEffector;
      }

      EndEffector endEffector = EndEffector.fromPlane("" + planeContactIndex.getIntegerValue(), centerOfMassFrame, planeContactState,
                                   wRhoPlaneContacts.getDoubleValue(), rhoMinScalar.getDoubleValue(), registry);
      previouslyUsedPlaneEndEffectors.put(planeContactState, endEffector);
      registerEndEffectorGraphic(endEffector);
      planeContactIndex.increment();

      return endEffector;
   }

   //TODO (Sylvain): change to private (used for CarIngressEgressController)
   protected EndEffector getOrCreate(CylindricalContactState cylinder)
   {
      if (previouslyUsedCylinderEndEffectors.containsKey(cylinder))
      {
         EndEffector endEffector = previouslyUsedCylinderEndEffectors.get(cylinder);
         endEffector.updateFromCylinder(cylinder, wRhoCylinderContacts.getDoubleValue(), wPhiCylinderContacts.getDoubleValue());

         return endEffector;
      }

      EndEffector endEffector = EndEffector.fromCylinder(centerOfMassFrame, cylinder, wRhoCylinderContacts.getDoubleValue(),
                                   wPhiCylinderContacts.getDoubleValue(), registry);
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

   public DenseMatrix64F getWRho()
   {
      return cylinderAndPlaneContactMatrixCalculator.getWRho();
   }

   public DenseMatrix64F getWPhi()
   {
      return cylinderAndPlaneContactMatrixCalculator.getWPhi();
   }

   public void setRhoMinScalar(double rhoMinScalar)
   {
      this.rhoMinScalar.set(rhoMinScalar);
   }
}
