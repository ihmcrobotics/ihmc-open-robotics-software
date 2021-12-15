package us.ihmc.robotModels.description;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;

public class CrossFourBarJointDescription extends OneDoFJointDescription
{
   private String jointNameA;
   private String jointNameB;
   private String jointNameC;
   private String jointNameD;
   private RigidBodyTransform transformAToPredecessor = new RigidBodyTransform();
   private RigidBodyTransform transformBToPredecessor = new RigidBodyTransform();
   private RigidBodyTransform transformCToB = new RigidBodyTransform();
   private RigidBodyTransform transformDToA = new RigidBodyTransform();

   private LinkDescription bodyDA = new LinkDescription("bodyDA");
   private LinkDescription bodyBC = new LinkDescription("bodyBC");

   private int actuatedJointIndex;
   private int loopClosureJointIndex;

   public CrossFourBarJointDescription(String name, Vector3DReadOnly jointAxis)
   {
      super(name, new Vector3D(Double.NaN, Double.NaN, Double.NaN), new Vector3D(Double.NaN, Double.NaN, Double.NaN));
      bodyDA.setName(name + "_DA");
      bodyBC.setName(name + "_BC");
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

   public void setBodyDA(LinkDescription bodyDA)
   {
      this.bodyDA = bodyDA;
   }

   public void setBodyBC(LinkDescription bodyBC)
   {
      this.bodyBC = bodyBC;
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
                                  RigidBodyTransformReadOnly transformDToA,
                                  RigidBodyTransformReadOnly transformCToB)
   {
      this.transformAToPredecessor.set(transformAToPredecessor);
      this.transformBToPredecessor.set(transformBToPredecessor);
      this.transformCToB.set(transformCToB);
      this.transformDToA.set(transformDToA);
   }

   /**
    * @param actuatedJointIndex the index of the joint that is actuated, i.e. torque source. 0 is for
    *                           joint A, 1 for B, 2 for C, and 3 for D.
    */
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

   public LinkDescription getBodyDA()
   {
      return bodyDA;
   }

   public LinkDescription getBodyBC()
   {
      return bodyBC;
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
      clone.transformAToPredecessor.set(transformAToPredecessor);
      clone.transformBToPredecessor.set(transformBToPredecessor);
      clone.transformCToB.set(transformCToB);
      clone.transformDToA.set(transformDToA);
      clone.bodyDA = new LinkDescription(bodyDA);
      clone.bodyBC = new LinkDescription(bodyBC);
      clone.actuatedJointIndex = actuatedJointIndex;
      clone.loopClosureJointIndex = loopClosureJointIndex;
      return clone;
   }
}
