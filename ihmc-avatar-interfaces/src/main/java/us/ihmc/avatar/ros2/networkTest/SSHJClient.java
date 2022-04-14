package us.ihmc.avatar.ros2.networkTest;

import net.schmizz.sshj.SSHClient;
import net.schmizz.sshj.common.IOUtils;
import net.schmizz.sshj.connection.channel.direct.Session;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * These methods start sessions, which can support a command, shell, or subsystem.
 * Each session may only be used for a single command, shell, or subsystem.
 * However, a single client connection supports multiple sessions.
 */
public class SSHJClient
{
   private final SSHClient sshClient;

   public SSHJClient(SSHClient sshClient)
   {
      this.sshClient = sshClient;
   }

   public Integer exec(String command)
   {
      return exec(command, 5.0);
   }

   public Integer exec(String command, double timeout)
   {
      return exec(command, timeout, sshjCommand ->
      {
         LogTools.info(ExceptionTools.handle(() -> IOUtils.readFully(sshjCommand.getInputStream()), DefaultExceptionHandler.RUNTIME_EXCEPTION).toString());
         LogTools.error(ExceptionTools.handle(() -> IOUtils.readFully(sshjCommand.getErrorStream()), DefaultExceptionHandler.RUNTIME_EXCEPTION).toString());
      });
   }

   public Integer exec(String command, double timeout, Consumer<Session.Command> sshjCommandConsumer)
   {
      Session session = null;
      Session.Command sshjCommand;
      try
      {
         session = ExceptionTools.handle(sshClient::startSession, DefaultExceptionHandler.RUNTIME_EXCEPTION);
         Session finalSession = session;

         LogTools.info("Executing on {}: {}", sshClient.getRemoteHostname(), command);
         sshjCommand = ExceptionTools.handle(() -> finalSession.exec(command), DefaultExceptionHandler.RUNTIME_EXCEPTION);
         LogTools.info("Done with exec");
         Session.Command finalSSHJCommand = sshjCommand;

         sshjCommandConsumer.accept(finalSSHJCommand);

         ExceptionTools.handle(() -> finalSSHJCommand.join((long) (timeout * 1e9), TimeUnit.NANOSECONDS), DefaultExceptionHandler.RUNTIME_EXCEPTION);
         LogTools.info("** exit status: {}", sshjCommand.getExitStatus());
      }
      finally
      {
         try
         {
            if (session != null)
            {
               session.close();
            }
         }
         catch (IOException e)
         {
            // do nothing
         }
      }
      return sshjCommand.getExitStatus();
   }
}
