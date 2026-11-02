package us.ihmc.rdx.simulation.scs2;

import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.SimulationSession;

public class RDXSCS2SimulationSession extends RDXSCS2Session
{
   public RDXSCS2SimulationSession(RDXBaseUI baseUI)
   {
      super(baseUI);
   }

   /**
    * Bring your own session.
    * This supports being called multiple times to switch between sessions.
    */
   public void startSession(SimulationSession simulationSession)
   {
      super.startSession(simulationSession);

      simulationSession.addAfterPhysicsCallback(time ->
      {
         // This technically belongs in RDXSCS2Session, but there's no after tick callback for a regular session
         if (pauseAtEndOfBuffer.get() && session.getBufferProperties().getCurrentIndex() == session.getBufferProperties().getSize() - 1)
         {
            session.setSessionMode(SessionMode.PAUSE);
         }
      });

      for (Runnable onSessionStartedRunnable : getOnSessionStartedRunnables())
      {
         onSessionStartedRunnable.run();
      }
   }

   @Override
   public SimulationSession getSession()
   {
      return (SimulationSession) session;
   }
}
