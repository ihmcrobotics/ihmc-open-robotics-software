package us.ihmc.avatar.ros.networkTest;

import net.schmizz.sshj.SSHClient;
import net.schmizz.sshj.common.IOUtils;
import net.schmizz.sshj.connection.channel.direct.Session;
import net.schmizz.sshj.sftp.SFTPClient;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

public class SSHJRemoteConnection
{
   private SSHClient ssh;
   private SFTPClient sftp;

   public SSHJRemoteConnection(SSHClient ssh, SFTPClient sftp)
   {
      this.ssh = ssh;
      this.sftp = sftp;
   }

   public void exec(String command)
   {
      exec(command, 5.0);
   }

   public void exec(String command, double timeout)
   {
      Session session = null;
      try
      {
         session = ExceptionTools.handle(() -> ssh.startSession(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

         LogTools.info("Executing on {}: {}", ssh.getRemoteHostname(), command);
         Session finalSession = session;
         Session.Command sshjCommand = ExceptionTools.handle(() -> finalSession.exec(command), DefaultExceptionHandler.RUNTIME_EXCEPTION);
         LogTools.info(ExceptionTools.handle(() -> IOUtils.readFully(sshjCommand.getInputStream()), DefaultExceptionHandler.RUNTIME_EXCEPTION).toString());
         LogTools.error(ExceptionTools.handle(() -> IOUtils.readFully(sshjCommand.getErrorStream()), DefaultExceptionHandler.RUNTIME_EXCEPTION).toString());
         ExceptionTools.handle(() -> sshjCommand.join((long) (timeout * 1e9), TimeUnit.NANOSECONDS), DefaultExceptionHandler.RUNTIME_EXCEPTION);
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
   }

   void put(String source, String dest)
   {
      LogTools.info("Putting $source to ${ssh.remoteHostname}:$dest");
      ExceptionTools.handle(() -> sftp.put(source, dest), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   void get(String source, String dest)
   {
      LogTools.info("Getting ${ssh.remoteHostname}:$source to $dest");
      ExceptionTools.handle(() -> sftp.get(source, dest), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }
}
