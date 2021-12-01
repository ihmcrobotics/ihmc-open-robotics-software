package us.ihmc.robotModels.description;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;

public class CrossFourBarJointDescription extends OneDoFJointDescription
{
   private String jointNameA;
   private String jointNameB;
   private String jointNameC;
   private String jointNameD;
   private String bodyNameDA;
   private String bodyNameBC;
   private RigidBodyTransform transformAToPredecessor = new RigidBodyTransform();
   private RigidBodyTransform transformBToPredecessor = new RigidBodyTransform();
   private RigidBodyTransform transformCToB = new RigidBodyTransform();
   private RigidBodyTransform transformDToA = new RigidBodyTransform();
   private double bodyMassDA;
   private double bodyMassBC;
   private Matrix3D bodyInertiaDA = new Matrix3D();
   private Matrix3D bodyInertiaBC = new Matrix3D();
   private RigidBodyTransform bodyInertiaPoseDA = new RigidBodyTransform();
   private RigidBodyTransform bodyInertiaPoseBC = new RigidBodyTransform();
   private int actuatedJointIndex;
   private int loopClosureJointIndex;

   public CrossFourBarJointDescription(String name, Vector3DReadOnly jointAxis)
   {
      super(name, new Vector3D(Double.NaN, Double.NaN, Double.NaN), new Vector3D(Double.NaN, Double.NaN, Double.NaN));
   }

   public void setJointNameA(String jointNameA)
   {
      this.jointNameA = jointNameA;
   }

   public void setJointNameB(String jointNameB)
   {
      this.jointNameB = jointNameB;
   }

   public void setJointNameC(String jointNameC)
   {
      this.jointNameC = jointNameC;
   }

   public void setJointNameD(String jointNameD)
   {
      this.jointNameD = jointNameD;
   }

   public void setJointNames(String jointNameA, String jointNameB, String jointNameC, String jointNameD)
   {
      this.jointNameA = jointNameA;
      this.jointNameB = jointNameB;
      this.jointNameC = jointNameC;
      this.jointNameD = jointNameD;
   }

   public void setBodyNameDA(String bodyNameDA)
   {
      this.bodyNameDA = bodyNameDA;
   }

   public void setBodyNameBC(String bodyNameBC)
   {
      this.bodyNameBC = bodyNameBC;
   }

   public void setBodyNames(String bodyNameDA, String bodyNameBC)
   {
      this.bodyNameDA = bodyNameDA;
      this.bodyNameBC = bodyNameDA;
   }

   public void setTransformAToPredecessor(RigidBodyTransformReadOnly transformAToPredecessor)
   {
      this.transformAToPredecessor.set(transformAToPredecessor);
   }

   public void setTransformBToPredecessor(RigidBodyTransformReadOnly transformBToPredecessor)
   {
      this.transformBToPredecessor.set(transformBToPredecessor);
   }

   public void setTransformCToB(RigidBodyTransform transformCToB)
   {
      this.transformCToB.set(transformCToB);
   }

   public void setTransformDToA(RigidBodyTransform transformDToA)
   {
      this.transformDToA.set(transformDToA);
   }

   public void setJointTransforms(RigidBodyTransformReadOnly transformAToPredecessor,
                                  RigidBodyTransformReadOnly transformBToPredecessor,
                                  RigidBodyTransformReadOnly transformCToB,
                                  RigidBodyTransformReadOnly transformDToA)
   {
      this.transformAToPredecessor.set(transformAToPredecessor);
      this.transformBToPredecessor.set(transformBToPredecessor);
      this.transformCToB.set(transformCToB);
      this.transformDToA.set(transformDToA);
   }

   public void setBodyMassDA(double bodyMassDA)
   {
      this.bodyMassDA = bodyMassDA;
   }

   public void setBodyMassBC(double bodyMassBC)
   {
      this.bodyMassBC = bodyMassBC;
   }

   public void setBodyInertiaDA(Matrix3DReadOnly bodyInertiaDA)
   {
      this.bodyInertiaDA.set(bodyInertiaDA);
   }

   public void setBodyInertiaBC(Matrix3DReadOnly bodyInertiaBC)
   {
      this.bodyInertiaBC.set(bodyInertiaBC);
   }

   public void setBodyInertiaPoseDA(RigidBodyTransformReadOnly bodyInertiaPoseDA)
   {
      this.bodyInertiaPoseDA.set(bodyInertiaPoseDA);
   }

   public void setBodyInertiaPoseBC(RigidBodyTransformReadOnly bodyInertiaPoseBC)
   {
      this.bodyInertiaPoseBC.set(bodyInertiaPoseBC);
   }

   public void setBodyInertias(double bodyMassDA,
                               double bodyMassBC,
                               Matrix3DReadOnly bodyInertiaDA,
                               Matrix3DReadOnly bodyInertiaBC,
                               RigidBodyTransformReadOnly bodyInertiaPoseDA,
                               RigidBodyTransformReadOnly bodyInertiaPoseBC)
   {
      this.bodyMassDA = bodyMassDA;
      this.bodyMassBC = bodyMassBC;
      this.bodyInertiaDA.set(bodyInertiaDA);
      this.bodyInertiaBC.set(bodyInertiaBC);
      this.bodyInertiaPoseDA.set(bodyInertiaPoseDA);
      this.bodyInertiaPoseBC.set(bodyInertiaPoseBC);
   }

   public void setActuatedJointIndex(int actuatedJointIndex)
   {
      this.actuatedJointIndex = actuatedJointIndex;
   }

   public void setLoopClosureJointIndex(int loopClosureJointIndex)
   {
      this.loopClosureJointIndex = loopClosureJointIndex;
   }

   public String getJointNameA()
   {
      return jointNameA;
   }

   public String getJointNameB()
   {
      return jointNameB;
   }

   public String getJointNameC()
   {
      return jointNameC;
   }

   public String getJointNameD()
   {
      return jointNameD;
   }

   public String getBodyNameDA()
   {
      return bodyNameDA;
   }

   public String getBodyNameBC()
   {
      return bodyNameBC;
   }

   public RigidBodyTransform getTransformAToPredecessor()
   {
      return transformAToPredecessor;
   }

   public RigidBodyTransform getTransformBToPredecessor()
   {
      return transformBToPredecessor;
   }

   public RigidBodyTransform getTransformCToB()
   {
      return transformCToB;
   }

   public RigidBodyTransform getTransformDToA()
   {
      return transformDToA;
   }

   public double getBodyMassDA()
   {
      return bodyMassDA;
   }

   public double getBodyMassBC()
   {
      return bodyMassBC;
   }

   public Matrix3D getBodyInertiaDA()
   {
      return bodyInertiaDA;
   }

   public Matrix3D getBodyInertiaBC()
   {
      return bodyInertiaBC;
   }

   public RigidBodyTransform getBodyInertiaPoseDA()
   {
      return bodyInertiaPoseDA;
   }

   public RigidBodyTransform getBodyInertiaPoseBC()
   {
      return bodyInertiaPoseBC;
   }

   public int getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   public int getLoopClosureJointIndex()
   {
      return loopClosureJointIndex;
   }

   @Override
   public CrossFourBarJointDescription copy()
   {
      CrossFourBarJointDescription clone = new CrossFourBarJointDescription(getName(), getJointAxis());
      clone.jointNameA = jointNameA;
      clone.jointNameB = jointNameB;
      clone.jointNameC = jointNameC;
      clone.jointNameD = jointNameD;
      clone.bodyNameDA = bodyNameDA;
      clone.bodyNameBC = bodyNameBC;
      clone.transformAToPredecessor.set(transformAToPredecessor);
      clone.transformBToPredecessor.set(transformBToPredecessor);
      clone.transformCToB.set(transformCToB);
      clone.transformDToA.set(transformDToA);
      clone.bodyInertiaDA.set(bodyInertiaDA);
      clone.bodyInertiaBC.set(bodyInertiaBC);
      clone.bodyMassDA = bodyMassDA;
      clone.bodyMassBC = bodyMassBC;
      clone.bodyInertiaPoseDA.set(bodyInertiaPoseDA);
      clone.bodyInertiaPoseBC.set(bodyInertiaPoseBC);
      clone.actuatedJointIndex = actuatedJointIndex;
      clone.loopClosureJointIndex = loopClosureJointIndex;
      return clone;
   }
}
