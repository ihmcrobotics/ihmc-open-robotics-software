package us.ihmc.commonWalkingControlModules.sensors;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.kalman.YoKalmanFilter;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameLine;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicLineSegment;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;


public class MassMatrixEstimatingToolRigidBody
{
   private static final int numberOfLines = 200;
   
   private final YoVariableRegistry registry;
   
   private final double gravity;
//   private final RigidBodyInertia estimatedInertia;
   
   private final ReferenceFrame handFixedFrame;
   private final CenterOfMassCalculator comCalculator;
   
   private final Wrench calculatedObjectWrench;
   
   private final YoFramePoint objectCenterOfMass;
   
   private final YoKalmanFilter objectCoMFilter;
   private final DenseMatrix64F F = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F G = new DenseMatrix64F(3, 0);
   private final DenseMatrix64F H = new DenseMatrix64F(3, 3);
   
   private final DenseMatrix64F u = new DenseMatrix64F(0, 0);
   
   private final DenseMatrix64F R = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F Q = new DenseMatrix64F(3, 3);
   
   private final DenseMatrix64F y = new DenseMatrix64F(3, 1);
   
   // Visualization lines
   @SuppressWarnings("unchecked")
   private final Pair<YoFramePoint, YoFramePoint>[] yoLinePoints = new Pair[numberOfLines];
   private final FrameLine[] centerOfMassLines = new FrameLine[numberOfLines];
   
   private int currentIndex = 0;
   
   public MassMatrixEstimatingToolRigidBody(String name, InverseDynamicsJoint wristJoint, double gravity, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
//      super(name, new RigidBodyInertia(wristJoint.getFrameAfterJoint(), 0.0, 0.0, 0.0, 0.0), wristJoint);
      this.registry = new YoVariableRegistry(name);
      this.gravity = gravity;
//      this.estimatedInertia = getInertia();
      
      
      
      this.handFixedFrame = wristJoint.getSuccessor().getBodyFixedFrame();//.getFrameAfterJoint();
      
      
      RigidBody[] rigidBodies = ScrewTools.computeRigidBodiesInOrder(wristJoint);
      this.comCalculator = new CenterOfMassCalculator(rigidBodies, handFixedFrame);
      calculatedObjectWrench = new Wrench(handFixedFrame, handFixedFrame);
      
      
      objectCoMFilter = new YoKalmanFilter(name + "Filter", 3, 0, 3, registry);
      CommonOps.setIdentity(F);
      
      CommonOps.setIdentity(R);
      CommonOps.scale(0.1, R);
      
      CommonOps.setIdentity(Q);
      CommonOps.scale(0.01, Q);
      
      //Initialize
      DenseMatrix64F x = new DenseMatrix64F(3, 1);
      DenseMatrix64F P = new DenseMatrix64F(3, 3);
      CommonOps.setIdentity(P);
      CommonOps.scale(100.0, P);
      objectCoMFilter.setState(x, P);
      
      
      this.objectCenterOfMass = new YoFramePoint(name + "CenterOfMass", ReferenceFrame.getWorldFrame(), registry);
      
      
      for(int i = 0; i < numberOfLines; i++)
      {
         Pair<YoFramePoint, YoFramePoint> linePair = new Pair<YoFramePoint, YoFramePoint>(
               new YoFramePoint(name + "CoMStartPoint" + i, ReferenceFrame.getWorldFrame(), registry),
               new YoFramePoint(name + "CoMEndPoint" + i, ReferenceFrame.getWorldFrame(), registry));
         yoLinePoints[i] = linePair;
      }
      
      if(dynamicGraphicObjectsListRegistry != null)
      {
         
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList(name);
         DynamicGraphicObject comViz = objectCenterOfMass.createDynamicGraphicPosition(name + "CenterOfMassViz", 0.05, YoAppearance.Red());
         dynamicGraphicObjectsList.add(comViz);
         
         for(Pair<YoFramePoint, YoFramePoint> linePair : yoLinePoints)
         {
            DynamicGraphicObject comLineViz = new DynamicGraphicLineSegment(name + "CoMApplicationLineViz", linePair.first(), linePair.second(), 1.0, YoAppearance.Red(), false, 0.0002);
            dynamicGraphicObjectsList.add(comLineViz);
         }         
         
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      }
      parentRegistry.addChild(registry);
   }
   
   
   public void update(Wrench measuredWristWrench)
   {
      comCalculator.compute();
      FrameVector gravityVector = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -gravity);
      gravityVector.changeFrame(handFixedFrame);
      gravityVector.scale(comCalculator.getTotalMass());
      
      FramePoint com = comCalculator.getCenterOfMass();
      
      FrameVector torque = new FrameVector(handFixedFrame);
      torque.cross(com, gravityVector);
      
      calculatedObjectWrench.set(handFixedFrame, gravityVector.getVector(), torque.getVector());
      
      
      
//      measuredWristWrench.add(calculatedObjectWrench);
      
     
      // TODO: Use calculated wrench for testing, switch to measured to make it work!
//      measuredWristWrench = calculatedObjectWrench;
      
      
      FrameVector torqueDueObject = measuredWristWrench.getAngularPartAsFrameVectorCopy();
      FrameVector forceDueObject = measuredWristWrench.getLinearPartAsFrameVectorCopy();
      
      
      System.out.println(calculatedObjectWrench);
      System.out.println(measuredWristWrench);
      if(torqueDueObject.getVector().epsilonEquals(new Vector3d(), 1e-4) || forceDueObject.getVector().epsilonEquals(new Vector3d(), 1e-4))
      {
         return;
      }
      
      MatrixTools.insertTuple3dIntoEJMLVector(torqueDueObject.getVector(), y, 0);
      MatrixTools.vectorToSkewSymmetricMatrix(H, forceDueObject.getVector());
      CommonOps.scale(-1.0, H);
      
      objectCoMFilter.configure(F, G, H);
      
      objectCoMFilter.setProcessNoiseCovariance(Q);
      objectCoMFilter.setMeasurementNoiseCovariance(R);
      
      
      objectCoMFilter.predict(u);
      objectCoMFilter.update(y);

      FramePoint objectCoM = new FramePoint(handFixedFrame);
      MatrixTools.denseMatrixToVector3d(objectCoMFilter.getState(), objectCoM.getPoint(), 0, 0);
      
      
      
      // Visualization stuff
      objectCoM.changeFrame(ReferenceFrame.getWorldFrame());
      objectCenterOfMass.set(objectCoM);
      
      FrameVector radius = new FrameVector(measuredWristWrench.getExpressedInFrame());
      radius.cross(torqueDueObject, forceDueObject);
      radius.scale(-1.0/forceDueObject.dot(forceDueObject));
      
      
      FrameLine line = new FrameLine(radius.getReferenceFrame(), radius.getVector(), forceDueObject.getVector());
      centerOfMassLines[currentIndex] = line;
      
      currentIndex++;
      if(currentIndex >= numberOfLines)
      {
         currentIndex = 0;
      }
      
      updateVisuals();
   }
   
   private void updateVisuals()
   {
      for(int i = 0; i < numberOfLines; i++)
      {
         FrameLine line = centerOfMassLines[i];
         if(line != null)
         {
            FramePoint origin = line.getOriginInFrame(ReferenceFrame.getWorldFrame());
            FrameVector direction = line.getDirectionInFrame(ReferenceFrame.getWorldFrame());
            
            
            FramePoint end = new FramePoint(origin);
            end.add(direction);
            
            origin.sub(direction);
            
            
            yoLinePoints[i].first().set(origin);
            yoLinePoints[i].second().set(end);
            
            
         }
      }
   }
}
