package us.ihmc.avatar.ros.networkTest;

import net.schmizz.sshj.SSHClient;
import net.schmizz.sshj.common.IOUtils;
import net.schmizz.sshj.connection.channel.direct.Session;
import net.schmizz.sshj.sftp.SFTPClient;
import net.schmizz.sshj.transport.verification.PromiscuousVerifier;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class RemoteSSHTools
{
   public void session(String address, String username, String password, Consumer<RemoteConnection> action)
   {
      ExceptionTools.handle(() -> session(address,
                                          sshClient -> ExceptionTools.handle(() -> sshClient.authPassword(username, password),
                                                                             DefaultExceptionHandler.RUNTIME_EXCEPTION),
                                          action), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   public void session(String address, String username, Consumer<RemoteConnection> action)
   {
      ExceptionTools.handle(() -> session(address, sshClient -> authWithSSHKey(username, sshClient), action), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   /**
    * Replicate OpenSSH functionality where users can name private keys whatever they want.
    */
   private void authWithSSHKey(String username, SSHClient sshClient)
   {
      Path userSSHConfigFolder = Paths.get(System.getProperty("user.home")).resolve(".ssh");

      Stream<Path> list = ExceptionTools.handle(() -> Files.list(userSSHConfigFolder), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      ArrayList<String> privateKeyFiles = new ArrayList<>();
      for (Path path : list.collect(Collectors.toList()))
      {
         if (Files.isRegularFile(path)
             && path.getFileName().toString() != "config"
             && path.getFileName().toString() != "known_hosts"
             && !path.getFileName().toString().endsWith(".pub"))
         {
            String absoluteNormalizedString = path.toAbsolutePath().normalize().toString();
            privateKeyFiles.add(absoluteNormalizedString);
         }
      }

      LogTools.info("Passing keys to authPublicKey: {}", privateKeyFiles);

      ExceptionTools.handle(() -> sshClient.authPublickey(username, privateKeyFiles.toArray(new String[0])), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   class RemoteConnection
   {
      private SSHClient ssh;
      private SFTPClient sftp;

      public RemoteConnection(SSHClient ssh, SFTPClient sftp)
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

   public void session(String address, Consumer<SSHClient> authenticate, Consumer<RemoteConnection> action) throws IOException
   {
      SSHClient sshClient = new SSHClient();
      ExceptionTools.handle(() -> sshClient.loadKnownHosts(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
      sshClient.addHostKeyVerifier(new PromiscuousVerifier()); // TODO: Try removing this again
      ExceptionTools.handle(() -> sshClient.connect(address), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      try
      {
         authenticate.accept(sshClient);

         try (SFTPClient sftpClient = sshClient.newSFTPClient())
         {
            action.accept(new RemoteConnection(sshClient, sftpClient));
         }
      }
      finally
      {
         sshClient.disconnect();
      }
   }
}
