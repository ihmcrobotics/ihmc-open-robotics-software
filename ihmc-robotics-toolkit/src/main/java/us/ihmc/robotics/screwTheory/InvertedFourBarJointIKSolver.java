package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.kinematics.fourbar.FourBarVertex;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;

public interface InvertedFourBarJointIKSolver
{
   /**
    * (Optional) Sets converters to use when comparing the four bar interior angles to the given
    * &theta; angle in {@link #solve(double, FourBarVertex)}.
    * <p>
    * This can be particularly useful when &theta; computed from joint configuration perspective
    * instead of four bar geometry perspective.
    * </p>
    * 
    * @param converters the 4 converters to use when comparing the interior angles to &theta;. They're
    *                   expected to be for the vertices A, B, C, and D in order.
    */
   void setConverters(FourBarToJointConverter[] converters);

   /**
    * Solve for the four bar configuration given &theta;, the angle between the two non-crossing edges,
    * more precisely from the non-flipped edge to the flipped edge.
    * <p>
    * The four bar is assumed to be an inverted four bar, i.e. two edges are crossing.
    * </p>
    * <p>
    * Given the configuration depicted below, let's define the point O being the intersection of the
    * edges AB and CD support lines. The angle &theta; is defined as the angle BOD. Given &theta;, this
    * method is expected to solve for the angle DAB and to return the value.
    * 
    * <pre>
    * +A------B+
    *   \    /  
    *    \  /   
    *     \/    
    *     /\    
    *    /  \   
    *   /    \  
    * -C------D-
    * </pre>
    * </p>
    * 
    * @param theta            the desired angle from the non-flipped edge (AB in the example) to the
    *                         flipped edge (CD in the example). In the given example it is the angle
    *                         BOD. It is expected to be positive if clockwise, and negative if
    *                         counter-clockwise.
    * @param vertexToSolveFor the vertex which angle is to be solved for. In the given example, it is
    *                         the vertex A.
    * @return the value of the angle DAB.
    */
   double solve(double theta, FourBarVertex vertexToSolveFor);
}
