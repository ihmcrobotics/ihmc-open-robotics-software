package us.ihmc.avatar.manipulating;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.RandomRestartInverseKinematicsCalculator;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.tools.io.printing.PrintTools;

public class GhostRobotArm
{
   public OneDoFJoint[] jointOfOrigin;
   public OneDoFJoint[] jointOfGhost;
   
   Point3d positionGhost = new Point3d();
   Quat4d orientationGhost = new Quat4d();
      
   private NumericalInverseKinematicsCalculator ikSolver;
   private RandomRestartInverseKinematicsCalculator ikRandomSolver;
   
   private final GeometricJacobian jacobianArm;   
   
   DRCBehaviorTestHelper testHelper;
   
   RigidBodyTransform offsetTransform;
   RigidBodyTransform offsetTransformInv;
   FullHumanoidRobotModel fullRobotModel;
   
   public class CapsuleModel
   {
      private Point3d positionOfFirst;
      private Point3d positionOfSecond;
      private double lengthOfCapsule;
      private double radiusOfCapsule;
      private double heightOfCapsule;
      private RigidBodyTransform transformOfCapsule = new RigidBodyTransform();
      private CapsuleModel(Point3d firstPoint, Point3d secondPoint, double radius)
      {
         positionOfFirst = new Point3d(firstPoint);
         positionOfSecond = new Point3d(secondPoint);
         lengthOfCapsule = positionOfFirst.distance(positionOfSecond);
         radiusOfCapsule = radius;
         heightOfCapsule = lengthOfCapsule + 2*radiusOfCapsule;
      }
      public void setTransform(RigidBodyTransform transform)
      {
         transformOfCapsule = new RigidBodyTransform(transform);
      }
      public Point3d getFirstPoint()
      {
         return positionOfFirst;
      }
      public Point3d getSecondPoint()
      {
         return positionOfSecond;
      }
      public double getLength()
      {
         return lengthOfCapsule;
      }
      public double getRadius()
      {
         return radiusOfCapsule;
      }
      public double getHeight()
      {
         return heightOfCapsule;
      }
      public RigidBodyTransform getTransform()
      {
         return transformOfCapsule;
      }
   }
   public CapsuleModel upperArmCapsule;
   public CapsuleModel lowerArmCapsule;
   public CapsuleModel endArmCapsule;
      
   public GhostRobotArm(DRCBehaviorTestHelper helper)
   {
      this.testHelper = helper; 
      
      fullRobotModel = testHelper.getControllerFullRobotModel();
      ReferenceFrame ikDesiredFrame = fullRobotModel.getEndEffector(RobotSide.RIGHT, LimbName.ARM).getBodyFixedFrame();
      ReferenceFrame controlFrame = fullRobotModel.getHandControlFrame(RobotSide.RIGHT);
      
      offsetTransform = controlFrame.getTransformToDesiredFrame(ikDesiredFrame);
      offsetTransformInv = new RigidBodyTransform(offsetTransform);
      offsetTransform.invert();      
      
      RigidBody hand = fullRobotModel.getEndEffector(RobotSide.RIGHT, LimbName.ARM);
      RigidBody chest = fullRobotModel.getChest();

      jointOfOrigin = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, hand), OneDoFJoint.class);
      jointOfGhost = ScrewTools.filterJoints(ScrewTools.cloneJointPath(jointOfOrigin), OneDoFJoint.class);

      RigidBody startBody = jointOfGhost[0].getPredecessor();
      RigidBody endBody = jointOfGhost[jointOfGhost.length - 1].getSuccessor();

      jacobianArm = new GeometricJacobian(startBody, endBody, endBody.getBodyFixedFrame());
      
      upperArmCapsule = new CapsuleModel(getCenterOfJoint(getTransformOfSHX()), getCenterOfJoint(getTransformOfEBX()), 0.045);
      lowerArmCapsule = new CapsuleModel(getCenterOfJoint(getTransformOfEBX()), getCenterOfJoint(getTransformOfWRX()), 0.045);
      endArmCapsule = new CapsuleModel(getCenterOfJoint(getTransformOfWRX()), getCenterOfJoint(getTransformOfEND()), 0.06);
   }
   
   
   private Graphics3DObject createCapsuleForJointToJoint(RigidBodyTransform transform1, RigidBodyTransform transform2, double radiusOfCapsule, AppearanceDefinition appearance)
   {
      Graphics3DObject capsule = new Graphics3DObject();
      AxisAngle4d axisAngle = new AxisAngle4d();
      Point3d centerJoint1 = new Point3d(transform1.mat03, transform1.mat13, transform1.mat23);
      Point3d centerJoint2 = new Point3d(transform2.mat03, transform2.mat13, transform2.mat23);
      
      Point3d translateJointtoCenterOfCapsule = new Point3d(0, -centerJoint1.distance(centerJoint2)*0.5, 0);
      
      capsule.translate(centerJoint1);
      
      Quat4d centerOrientationUpperCapsule = new Quat4d();
            
      Matrix3d matrixOfTransform1 = new Matrix3d(transform1.mat00, transform1.mat01, transform1.mat02, transform1.mat10, transform1.mat11,
                                               transform1.mat12, transform1.mat20, transform1.mat21, transform1.mat22);
      
      RotationTools.convertMatrixToQuaternion(matrixOfTransform1, centerOrientationUpperCapsule);
      
      Vector3d axisAngleRV = new Vector3d();
      RotationTools.convertQuaternionToRotationVector(centerOrientationUpperCapsule, axisAngleRV);
      RotationTools.convertRotationVectorToAxisAngle(axisAngleRV, axisAngle);
      capsule.rotate(axisAngle);
      
      capsule.translate(translateJointtoCenterOfCapsule);      
      capsule.rotate(new AxisAngle4d(1,0,0,Math.PI/2));      
      capsule.addCapsule(radiusOfCapsule, centerJoint1.distance(centerJoint2), appearance);
      
      return capsule;
   }

   
   private Graphics3DObject createSphereForJointToJoint(RigidBodyTransform transform1, RigidBodyTransform transform2, double radiusOfCapsule, AppearanceDefinition appearance)
   {
      Graphics3DObject sphere = new Graphics3DObject();
      AxisAngle4d axisAngle = new AxisAngle4d();
      Point3d centerJoint1 = new Point3d(transform1.mat03, transform1.mat13, transform1.mat23);
      Point3d centerJoint2 = new Point3d(transform2.mat03, transform2.mat13, transform2.mat23);
      
      Point3d translateJointtoCenterOfCapsule = new Point3d(0, -centerJoint1.distance(centerJoint2)*0.5, 0);
      
      sphere.translate(centerJoint1);
      
      Quat4d centerOrientationUpperCapsule = new Quat4d();
            
      Matrix3d matrixOfTransform1 = new Matrix3d(transform1.mat00, transform1.mat01, transform1.mat02, transform1.mat10, transform1.mat11,
                                               transform1.mat12, transform1.mat20, transform1.mat21, transform1.mat22);
      
      RotationTools.convertMatrixToQuaternion(matrixOfTransform1, centerOrientationUpperCapsule);
      
      Vector3d axisAngleRV = new Vector3d();
      RotationTools.convertQuaternionToRotationVector(centerOrientationUpperCapsule, axisAngleRV);
      RotationTools.convertRotationVectorToAxisAngle(axisAngleRV, axisAngle);
      sphere.rotate(axisAngle);
      
      sphere.translate(translateJointtoCenterOfCapsule);
      sphere.addSphere(radiusOfCapsule, appearance);
      
      return sphere;
   }
      
   private Graphics3DObject createCapsuleModel(CapsuleModel capsuleModel, AppearanceDefinition appearance)
   {
      Graphics3DObject capsule = new Graphics3DObject();
                  
      capsule.transform(capsuleModel.getTransform());
      capsule.addCapsule(capsuleModel.getRadius(), capsuleModel.getHeight(), appearance);
      
      return capsule;
   }

   public ArrayList<Graphics3DObject> getGhostCollisionModel()
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject upperArm = new Graphics3DObject();
      Graphics3DObject lowerArm = new Graphics3DObject();
      Graphics3DObject endArm = new Graphics3DObject();
      
      updateCollisionModel();
      
      upperArm = createCapsuleModel(upperArmCapsule, YoAppearance.Aqua());
      lowerArm = createCapsuleModel(lowerArmCapsule, YoAppearance.Azure());
      endArm = createCapsuleModel(endArmCapsule, YoAppearance.Yellow());
        
      ret.add(upperArm);
      ret.add(lowerArm);
      ret.add(endArm);

      return ret;
   }
   
   private void updateCollisionModel()
   {      
      upperArmCapsule.setTransform(getCenterTransformOfTwo(getTransformOfSHX(), getTransformOfEBX()));
      lowerArmCapsule.setTransform(getCenterTransformOfTwo(getTransformOfEBX(), getTransformOfWRX()));
      endArmCapsule.setTransform(getCenterTransformOfTwo(getTransformOfWRX(), getTransformOfEND()));
      upperArmCapsule.getTransform().applyRotationX(Math.PI/2);
      lowerArmCapsule.getTransform().applyRotationX(Math.PI/2);
      endArmCapsule.getTransform().applyRotationX(Math.PI/2);
   }
   
   public void updateJointOfGhost(RigidBodyTransform desToHandWorld)
   {
      ikRandomSolver = createInverseKinematics(jacobianArm);
      
      RigidBodyTransform desToHand = new RigidBodyTransform();
      FramePoint targetInWorld = new FramePoint(ReferenceFrame.getWorldFrame(), new Point3d(desToHandWorld.mat03, desToHandWorld.mat13, desToHandWorld.mat23));
      FramePoint targetInChestFrame = new FramePoint(targetInWorld);
      testHelper.updateRobotModel();

      targetInChestFrame.changeFrame(testHelper.getReferenceFrames().getChestFrame());
      
      desToHand.setTranslationAndIdentityRotation(targetInChestFrame.getPoint());
      
      Quat4d tempQT = new Quat4d();
      RotationTools.convertTransformToQuaternion(desToHandWorld, tempQT);
      desToHand.setRotation(tempQT);
      
      desToHand.multiply(offsetTransform);
      
      boolean ikResult = ikRandomSolver.solve(desToHand);
      //PrintTools.info("The IK result is " + ikResult);
      
      updateCollisionModel();      
   }
   public void updateJointOfGhost(Point3d positionGiven, Quat4d orientationGiven)
   {
      RigidBodyTransform desToHandWorld = new RigidBodyTransform(orientationGiven, positionGiven);
      
      updateJointOfGhost(desToHandWorld);
   }   
   public void setJointOfGhostPosition(Point3d position)
   {
      positionGhost = position;
   }
   public void setJointOfGhostOrientation(Quat4d orientation)
   {
      orientationGhost = orientation;
   }   
   public void updateJointOfGhost()
   {
      updateJointOfGhost(positionGhost, orientationGhost);
   }
   
   private RandomRestartInverseKinematicsCalculator createInverseKinematics(GeometricJacobian jacobian)
   {
      double lambdaLeastSquares = 0.0009;
      double tolerance = 0.001;
      double maxStepSize = 0.2;
      double minRandomSearchScalar = 0.02;
      double maxRandomSearchScalar = 0.8;
      int maxIterations = 200;   
      
      int maxRestarts = 20;
      double restartTolerance = 0.001;
      
      ikSolver = new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);

      ikSolver.setLimitJointAngles(true);
      return new RandomRestartInverseKinematicsCalculator(maxRestarts, restartTolerance, jacobianArm, ikSolver);      
   }
   
   public void printRigidBodyTR(ReferenceFrame rFrame)
   {
      RigidBodyTransform bodyTransform = rFrame.getTransformToWorldFrame();
      PrintTools.info("Joint Name is : " + rFrame.getName());
      PrintTools.info(bodyTransform.mat00 + " " + bodyTransform.mat01 + " " + bodyTransform.mat02 + " " + bodyTransform.mat03);
      PrintTools.info(bodyTransform.mat10 + " " + bodyTransform.mat11 + " " + bodyTransform.mat12 + " " + bodyTransform.mat13);
      PrintTools.info(bodyTransform.mat20 + " " + bodyTransform.mat21 + " " + bodyTransform.mat22 + " " + bodyTransform.mat23 + "\n");
   }
   
   public void printRigidBodyTR(RigidBodyTransform bodyTransform)
   {  
      PrintTools.info(bodyTransform.mat00 + " " + bodyTransform.mat01 + " " + bodyTransform.mat02 + " " + bodyTransform.mat03);
      PrintTools.info(bodyTransform.mat10 + " " + bodyTransform.mat11 + " " + bodyTransform.mat12 + " " + bodyTransform.mat13);
      PrintTools.info(bodyTransform.mat20 + " " + bodyTransform.mat21 + " " + bodyTransform.mat22 + " " + bodyTransform.mat23 + "\n");
   }
   
   
   public ArrayList<Graphics3DObject> createXYZAxis(RigidBodyTransform axisGetTransformToWorldFrame)
   {
      double axisHeight = 0.2;
      double axisRadius = 0.005;
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject retX = new Graphics3DObject();
      Graphics3DObject retY = new Graphics3DObject();
      Graphics3DObject retZ = new Graphics3DObject();

      Point3d centerPoint = new Point3d(axisGetTransformToWorldFrame.getM03(), axisGetTransformToWorldFrame.getM13(),
                                        axisGetTransformToWorldFrame.getM23());

      retX.translate(centerPoint);
      retY.translate(centerPoint);
      retZ.translate(centerPoint);
      
      Matrix3d centerTR = new Matrix3d(axisGetTransformToWorldFrame.getM00(), axisGetTransformToWorldFrame.getM01(),
                                       axisGetTransformToWorldFrame.getM02(), axisGetTransformToWorldFrame.getM10(),
                                       axisGetTransformToWorldFrame.getM11(), axisGetTransformToWorldFrame.getM12(),
                                       axisGetTransformToWorldFrame.getM20(), axisGetTransformToWorldFrame.getM21(),
                                       axisGetTransformToWorldFrame.getM22());

      Quat4d qtCenter = new Quat4d();
      Quat4d qtAlpha = new Quat4d();
      Quat4d qtAxis = new Quat4d();
      AxisAngle4d rvAxis = new AxisAngle4d();

      RotationTools.convertMatrixToQuaternion(centerTR, qtCenter);

      Vector3d rvTemp = new Vector3d();
      RotationTools.convertQuaternionToRotationVector(qtCenter, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retZ.rotate(rvAxis);
      retZ.addCylinder(axisHeight, axisRadius, YoAppearance.Blue());

      RotationTools.convertRotationVectorToQuaternion(new Vector3d(0, Math.PI / 2, 0), qtAlpha);
      qtAxis.mul(qtCenter, qtAlpha);

      RotationTools.convertQuaternionToRotationVector(qtAxis, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retX.rotate(rvAxis);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      RotationTools.convertRotationVectorToQuaternion(new Vector3d(-Math.PI / 2, 0, 0), qtAlpha);
      qtAxis.mul(qtCenter, qtAlpha);

      RotationTools.convertQuaternionToRotationVector(qtAxis, rvTemp);
      RotationTools.convertRotationVectorToAxisAngle(rvTemp, rvAxis);
      retY.rotate(rvAxis);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }

   public Point3d getCenterOfJoint(RigidBodyTransform jointTransform)
   {
      return new Point3d(jointTransform.mat03, jointTransform.mat13, jointTransform.mat23);
   }
   public RigidBodyTransform getTransformOfSHX()
   {
      return jointOfGhost[2].getPredecessor().getBodyFixedFrame().getTransformToWorldFrame();
   }
   public RigidBodyTransform getTransformOfEBX()
   {
      return jointOfGhost[4].getPredecessor().getBodyFixedFrame().getTransformToWorldFrame();
   }
   public RigidBodyTransform getTransformOfWRX()
   {
      return jointOfGhost[6].getPredecessor().getBodyFixedFrame().getTransformToWorldFrame();
   }
   public RigidBodyTransform getTransformOfEND()
   {
      RigidBodyTransform rtOftheGhostHandControl = new RigidBodyTransform();
      
      RigidBodyTransform rtOftheGhostEnd = jointOfGhost[6].getSuccessor().getBodyFixedFrame().getTransformToWorldFrame();      
      rtOftheGhostHandControl.multiply(rtOftheGhostEnd, offsetTransformInv);
      
      return rtOftheGhostHandControl;      
   }
   private RigidBodyTransform getCenterTransformOfTwo(RigidBodyTransform transform1, RigidBodyTransform transform2)
   {
      RigidBodyTransform bodyTransform = new RigidBodyTransform(transform1);
      
      bodyTransform.mat03 = (transform1.mat03 + transform2.mat03)/2;
      bodyTransform.mat13 = (transform1.mat13 + transform2.mat13)/2;
      bodyTransform.mat23 = (transform1.mat23 + transform2.mat23)/2;
      
      return bodyTransform;
   }
}
