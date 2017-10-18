package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ReachableFootstepsBasedExpansion;

/**
 * This class adds footstep neighbors specific to Valkyrie Robot
 * @author Shlok Agarwal
 *
 */
public class ValkyrieReachableFootstepExpansion extends ReachableFootstepsBasedExpansion
{
   /**
    * Adds footstep offsets for Valkyrie robot
    * Note: These values have been rigourously tested in simulation but not on real robot.
    */
   @Override
   public void addNeighbors()
   {
      /** Backward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/    
      addOffset(-0.2     ,     0.3     ,     -0.1);
      addOffset(-0.2     ,     0.3     ,     -0.2);
      addOffset(-0.2     ,     0.3     ,     -0.4);
      addOffset(-0.2     ,     0.2     ,        0);
      addOffset(-0.2     ,     0.3     ,      0.1);
      addOffset(-0.2     ,     0.3     ,      0.2);
      addOffset(-0.2     ,     0.3     ,      0.4);
      addOffset(-0.1     ,     0.3     ,        0);
      addOffset(-0.1     ,     0.3     ,     -0.1);
      addOffset(-0.1     ,     0.3     ,     -0.2);
      addOffset(-0.1     ,     0.3     ,     -0.3);
      addOffset(-0.1     ,     0.35    ,     -0.4);
      addOffset(-0.1     ,     0.3     ,      0.1);
      addOffset(-0.1     ,     0.3     ,      0.2);
      addOffset(-0.1     ,     0.3     ,      0.3);
      addOffset(-0.1     ,     0.35    ,      0.4);

      /** Side Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 
      addOffset(0        ,     0.3     ,        0);
      addOffset(0        ,     0.35    ,     -0.2);
      addOffset(0        ,     0.35    ,     -0.4);
      addOffset(0        ,     0.35    ,     -0.6);
      addOffset(0        ,     0.35    ,     -0.7);
      addOffset(0        ,     0.35    ,      0.2);
      addOffset(0        ,     0.35    ,      0.4);
      addOffset(0        ,     0.35    ,      0.6);
      addOffset(0        ,     0.35    ,      0.7);
      addOffset(0        ,     0.3     ,        0);

      /** Forward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 
      addOffset(0.1      ,     0.3     ,     -0.1);
      addOffset(0.1      ,     0.3     ,     -0.2);
      addOffset(0.1      ,     0.3     ,     -0.3);
      addOffset(0.1      ,     0.35    ,     -0.4);
      addOffset(0.1      ,     0.3     ,      0.1);
      addOffset(0.1      ,     0.3     ,      0.2);
      addOffset(0.1      ,     0.3     ,      0.3);
      addOffset(0.1      ,     0.3     ,      0.4);
      addOffset(0.2      ,     0.2     ,        0);
      addOffset(0.2      ,     0.3     ,     -0.1);
      addOffset(0.2      ,     0.3     ,     -0.2);
      addOffset(0.2      ,     0.3     ,     -0.4);
      addOffset(0.2      ,     0.3     ,      0.1);
      addOffset(0.2      ,     0.3     ,      0.2);
      addOffset(0.2      ,     0.3     ,      0.4);
      addOffset(0.3      ,     0.2     ,        0);
      addOffset(0.3      ,     0.3     ,     -0.1);
      addOffset(0.3      ,     0.3     ,     -0.2);
      addOffset(0.3      ,     0.3     ,      0.1);
      addOffset(0.3      ,     0.3     ,      0.2);

   }
   
}
