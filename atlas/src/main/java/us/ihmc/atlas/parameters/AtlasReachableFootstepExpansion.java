package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ReachableFootstepsBasedExpansion;
/**
 * This class adds footstep neighbors specific to Atlas Robot
 * @author Shlok Agarwal
 *
 */
public class AtlasReachableFootstepExpansion extends ReachableFootstepsBasedExpansion
{
   /**
    * Adds footstep offsets for Atlas robot
    * Note: These are exactly the parameters used in SimpleSideBasedExpansion 
    */
   @Override
   public void addNeighbors()
   {

      /** Side Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/    
      addOffset(0.0     ,      0.25     ,      0.0);
      addOffset(0.0     ,      0.25     ,      0.174);
      addOffset(0.0     ,      0.25     ,      0.392);

      addOffset(0.0     ,      0.15     ,      0.0);
      addOffset(0.0     ,      0.20     ,      0.0);
      addOffset(0.0     ,      0.30     ,      0.0);

      /** Forward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 

      addOffset(0.05     ,      0.25     ,      0.0);
      addOffset(0.05     ,      0.25     ,      0.174);
      addOffset(0.05     ,      0.25     ,      0.392);

      addOffset(0.1     ,      0.25     ,      0.0);
      addOffset(0.1     ,      0.25     ,      0.174);
      addOffset(0.1     ,      0.25     ,      0.392);

      addOffset(0.2     ,      0.25     ,      0.0);
      addOffset(0.2     ,      0.25     ,      0.174);
      addOffset(0.2     ,      0.25     ,      0.392);

      addOffset(0.3     ,      0.25     ,      0.0);
      addOffset(0.3     ,      0.25     ,      0.174);
      addOffset(0.3     ,      0.25     ,      0.392);

      addOffset(0.4     ,      0.25     ,      0.0);
      addOffset(0.4     ,      0.25     ,      0.174);
      addOffset(0.4     ,      0.25     ,      0.392);

      /** Backward Steps*/
      /** x offset*/ /** y offset*/ /** yaw offset*/ 

      addOffset(-0.05     ,      0.25     ,      0.0);
      addOffset(-0.05     ,      0.25     ,      0.174);
      addOffset(-0.05     ,      0.25     ,      0.392);

      addOffset(-0.1     ,      0.25     ,      0.0);
      addOffset(-0.1     ,      0.25     ,      0.174);
      addOffset(-0.1     ,      0.25     ,      0.392);

      addOffset(-0.2     ,      0.25     ,      0.0);
      addOffset(-0.2     ,      0.25     ,      0.174);
      addOffset(-0.2     ,      0.25     ,      0.392);

      addOffset(-0.3     ,      0.25     ,      0.0);
      addOffset(-0.3     ,      0.25     ,      0.174);
      addOffset(-0.3     ,      0.25     ,      0.392);

      addOffset(-0.4     ,      0.25     ,      0.0);
      addOffset(-0.4     ,      0.25     ,      0.174);
      addOffset(-0.4     ,      0.25     ,      0.392);

      /** Turn Step*/
      addOffset(-0.047     ,      0.25     ,      0.392);

   }
}
